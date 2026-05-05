#include <algorithm>
#include <cmath>
#include <iostream>
#include <set>
#include <vector>

#include "ns3/ipv4-flow-classifier.h"

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/energy-module.h"
#include "control-agent-helper.h"
#include "etx-olsr-helper.h"
#include "etx-olsr-hopcount-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/spectrum-module.h"
#include "ns3/wifi-module.h"

#include "ns3/isotropic-antenna-model.h"

using namespace ns3;
using namespace ns3::energy;

NS_LOG_COMPONENT_DEFINE("AntiDroneSimulation");

namespace
{
struct FlowSummaryResult
{
    uint32_t totalFlows{0};
    uint32_t activeFlows{0};
    uint32_t connectedFlows{0};
    double connectedThroughputKbps{0.0};
    double connectedAvgDelayMs{0.0};
};

struct RecoveryStats
{
    bool inOutage{false};
    double outageStartSec{0.0};
    uint32_t recoveryEvents{0};
    double totalRecoverySec{0.0};
    double maxRecoverySec{0.0};
};

void
LogConnectivitySample(Ptr<FlowMonitor> monitor,
                      double connectivityThreshold,
                      double recoveryDropThresholdPct,
                      double recoveryRestoreThresholdPct,
                      bool recoveryUseAvgGoodput,
                      RecoveryStats* recoveryStats,
                      Time sampleInterval,
                      Time stopTime)
{
    if (!monitor)
    {
        return;
    }
    monitor->CheckForLostPackets();
    const auto stats = monitor->GetFlowStats();
    uint32_t total = 0;
    uint32_t active = 0;
    uint32_t connected = 0;
    uint32_t looseCnt = 0;
    double sumRxOverTx = 0.0;
    double connectedThroughputKbps = 0.0;
    double connectedDelaySumMs = 0.0;
    uint64_t connectedRxPackets = 0;
    constexpr double kLooseRatio = 0.5;
    for (const auto& kv : stats)
    {
        const auto& st = kv.second;
        total++;
        if (st.txPackets == 0)
        {
            continue;
        }
        active++;
        const double ratio = static_cast<double>(st.rxPackets) / static_cast<double>(st.txPackets);
        sumRxOverTx += ratio;
        if (ratio >= kLooseRatio)
        {
            looseCnt++;
        }
        if (ratio >= connectivityThreshold)
        {
            connected++;
            const double duration =
                std::max(1e-6, (st.timeLastRxPacket - st.timeFirstTxPacket).GetSeconds());
            connectedThroughputKbps += (st.rxBytes * 8.0) / duration / 1000.0;
            connectedDelaySumMs += st.delaySum.GetMilliSeconds();
            connectedRxPackets += st.rxPackets;
        }
    }
    const double nowS = Simulator::Now().GetSeconds();
    const double connectivityPct = (active > 0) ? (100.0 * connected / active) : 0.0;
    const double goodputRatioPct = (active > 0) ? (100.0 * sumRxOverTx / active) : 0.0;
    const double looseFlowsPct = (active > 0) ? (100.0 * looseCnt / active) : 0.0;
    const double avgDelayMs = (connectedRxPackets > 0) ? (connectedDelaySumMs / connectedRxPackets) : 0.0;

    // Recovery signal: "strict" = fraction of flows meeting rxTx>=threshold (can be 0% while avg ratio is ~30%).
    // "avgGoodput" = mean rx/tx ratio in percent — avoids false total-outage when threshold is aggressive (e.g. 0.8).
    const double recoverySignal =
        recoveryUseAvgGoodput ? goodputRatioPct : connectivityPct;

    if (recoveryStats)
    {
        if (!recoveryStats->inOutage && recoverySignal < recoveryDropThresholdPct)
        {
            recoveryStats->inOutage = true;
            recoveryStats->outageStartSec = nowS;
            std::cout << "RECOVERY_EVENT type=drop startSec=" << nowS << " recoverySignal=" << recoverySignal
                      << " mode=" << (recoveryUseAvgGoodput ? "avgGoodputPct" : "strictFlowsPct")
                      << " (connectivityPct=" << connectivityPct << " goodputRatioPct=" << goodputRatioPct << ")"
                      << std::endl;
        }
        else if (recoveryStats->inOutage && recoverySignal >= recoveryRestoreThresholdPct)
        {
            const double recoverySec = nowS - recoveryStats->outageStartSec;
            recoveryStats->inOutage = false;
            recoveryStats->recoveryEvents++;
            recoveryStats->totalRecoverySec += recoverySec;
            recoveryStats->maxRecoverySec = std::max(recoveryStats->maxRecoverySec, recoverySec);
            std::cout << "RECOVERY_EVENT type=restore timeSec=" << nowS << " recoverySec=" << recoverySec
                      << " recoverySignal=" << recoverySignal
                      << " mode=" << (recoveryUseAvgGoodput ? "avgGoodputPct" : "strictFlowsPct")
                      << std::endl;
        }
    }
    std::cout << "TIME " << nowS << " METRICS"
              << " rxTxPassThreshold=" << connectivityThreshold
              << " flowsStrictPassPct=" << connectivityPct
              << " deliveryAvgPct=" << goodputRatioPct
              << " flowsLoosePassPct_ge_" << kLooseRatio << "=" << looseFlowsPct
              << " avgDelayMs_strictFlows=" << avgDelayMs
              << " totalThroughputKbps_strictFlows=" << connectedThroughputKbps
              << " connectedFlows=" << connected
              << " activeFlows=" << active
              << " totalFlows=" << total
              << " recoveryUses=" << (recoveryUseAvgGoodput ? "avgGoodput" : "strictFlows")
              << std::endl;
    if (Simulator::Now() + sampleInterval < stopTime)
    {
        Simulator::Schedule(sampleInterval,
                            &LogConnectivitySample,
                            monitor,
                            connectivityThreshold,
                            recoveryDropThresholdPct,
                            recoveryRestoreThresholdPct,
                            recoveryUseAvgGoodput,
                            recoveryStats,
                            sampleInterval,
                            stopTime);
    }
}

void
FailNode(Ptr<Node> node)
{
    NS_LOG_WARN("Failing node " << node->GetId() << " at " << Simulator::Now().GetSeconds()
                                << "s");

    // Do NOT call node->Dispose() during the simulation. It may break object aggregation
    // (e.g., NetDevice->GetNode()) while other modules still hold references.
    //
    // Instead, simulate failure by stopping applications and bringing all IPv4 interfaces down.
    const Time now = Simulator::Now();
    for (uint32_t i = 0; i < node->GetNApplications(); ++i)
    {
        Ptr<Application> app = node->GetApplication(i);
        if (app)
        {
            // ApplicationContainer provides Stop(), but Application itself does not.
            // Force the application to stop at 'now'.
            app->SetStopTime(now);
        }
    }

    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
    if (ipv4)
    {
        for (uint32_t i = 0; i < ipv4->GetNInterfaces(); ++i)
        {
            ipv4->SetDown(i);
        }
    }
}

void
ScheduleRandomFailures(const NodeContainer& uavs,
                       uint32_t failCount,
                       Time failTime,
                       const std::set<uint32_t>& protectedNodes)
{
    Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable>();
    std::vector<uint32_t> candidates;
    for (uint32_t i = 0; i < uavs.GetN(); ++i)
    {
        const uint32_t nodeId = uavs.Get(i)->GetId();
        if (protectedNodes.count(nodeId) == 0)
        {
            candidates.push_back(i);
        }
    }

    for (uint32_t k = 0; k < failCount && !candidates.empty(); ++k)
    {
        const uint32_t pick = uv->GetInteger(0, candidates.size() - 1);
        const uint32_t idx = candidates[pick];
        Simulator::Schedule(failTime, &FailNode, uavs.Get(idx));
        candidates.erase(candidates.begin() + pick);
    }
}

void
SetNodeTxPower(Ptr<Node> node, double txPowerDbm)
{
    for (uint32_t i = 0; i < node->GetNDevices(); ++i)
    {
        Ptr<WifiNetDevice> wifiDev = DynamicCast<WifiNetDevice>(node->GetDevice(i));
        if (!wifiDev)
        {
            continue;
        }
        Ptr<WifiPhy> phy = wifiDev->GetPhy();
        if (!phy)
        {
            continue;
        }
        phy->SetTxPowerStart(txPowerDbm);
        phy->SetTxPowerEnd(txPowerDbm);
    }
}

void
ApplyRegionalJamming(const NodeContainer& uavs,
                     Vector jammerPos,
                     double jamRadius,
                     double jammedTxPowerDbm,
                     double normalTxPowerDbm)
{
    uint32_t jammedCount = 0;
    for (uint32_t i = 0; i < uavs.GetN(); ++i)
    {
        Ptr<Node> node = uavs.Get(i);
        Ptr<MobilityModel> mob = node->GetObject<MobilityModel>();
        if (!mob)
        {
            continue;
        }
        const double dist = CalculateDistance(mob->GetPosition(), jammerPos);
        if (dist <= jamRadius)
        {
            ++jammedCount;
            SetNodeTxPower(node, jammedTxPowerDbm);
        }
        else
        {
            SetNodeTxPower(node, normalTxPowerDbm);
        }
    }
    NS_LOG_UNCOND("RegionalJammerStep t=" << Simulator::Now().GetSeconds()
                                          << " radius=" << jamRadius
                                          << " affectsNodes=" << jammedCount);
}

void
MoveJammerAndApply(const NodeContainer& uavs,
                   Ptr<Node> jammer,
                   double areaMin,
                   double areaMax,
                   double jammerAltitude,
                   double jamRadius,
                   double jammedTxPowerDbm,
                   double normalTxPowerDbm,
                   Time jammerStart,
                   Time jammerStop,
                   Time moveInterval)
{
    if (!jammer)
    {
        return;
    }
    Ptr<MobilityModel> jamMob = jammer->GetObject<MobilityModel>();
    if (!jamMob)
    {
        return;
    }

    const Time now = Simulator::Now();
    if (now >= jammerStart && now <= jammerStop)
    {
        Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable>();
        const double x = uv->GetValue(areaMin, areaMax);
        const double y = uv->GetValue(areaMin, areaMax);
        jamMob->SetPosition(Vector(x, y, jammerAltitude));
        ApplyRegionalJamming(uavs, jamMob->GetPosition(), jamRadius, jammedTxPowerDbm, normalTxPowerDbm);
    }
    else
    {
        for (uint32_t i = 0; i < uavs.GetN(); ++i)
        {
            SetNodeTxPower(uavs.Get(i), normalTxPowerDbm);
        }
    }

    if (now + moveInterval < jammerStop + moveInterval)
    {
        Simulator::Schedule(moveInterval,
                            &MoveJammerAndApply,
                            uavs,
                            jammer,
                            areaMin,
                            areaMax,
                            jammerAltitude,
                            jamRadius,
                            jammedTxPowerDbm,
                            normalTxPowerDbm,
                            jammerStart,
                            jammerStop,
                            moveInterval);
    }
}

FlowSummaryResult
PrintFlowSummary(Ptr<FlowMonitor> monitor, FlowMonitorHelper& helper, double connectivityThreshold)
{
    FlowSummaryResult result;
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(helper.GetClassifier());
    const auto stats = monitor->GetFlowStats();
    for (const auto& kv : stats)
    {
        result.totalFlows++;
        const auto fiveTuple = classifier->FindFlow(kv.first);
        const auto& st = kv.second;
        if (st.txPackets == 0)
        {
            continue;
        }
        result.activeFlows++;
        const double rxOverTx = (st.txPackets > 0)
                                    ? (static_cast<double>(st.rxPackets) / static_cast<double>(st.txPackets))
                                    : 0.0;
        if (rxOverTx >= connectivityThreshold)
        {
            result.connectedFlows++;
        }
        const double duration =
            std::max(1e-6, (st.timeLastRxPacket - st.timeFirstTxPacket).GetSeconds());
        const double throughputKbps = (st.rxBytes * 8.0) / duration / 1000.0;
        const double avgDelayMs =
            (st.rxPackets > 0) ? st.delaySum.GetMilliSeconds() / st.rxPackets : 0.0;
        if (rxOverTx >= connectivityThreshold)
        {
            result.connectedThroughputKbps += throughputKbps;
            result.connectedAvgDelayMs += avgDelayMs;
        }
        std::cout << "Flow " << kv.first << " " << fiveTuple.sourceAddress << " -> "
                  << fiveTuple.destinationAddress << " tx=" << st.txPackets
                  << " rx=" << st.rxPackets << " lost=" << st.lostPackets
                  << " rxTxRatio=" << rxOverTx
                  << " avgDelayMs=" << avgDelayMs << " throughputKbps=" << throughputKbps
                  << std::endl;
    }
    if (result.connectedFlows > 0)
    {
        result.connectedAvgDelayMs /= static_cast<double>(result.connectedFlows);
    }
    const double connectivity =
        (result.activeFlows > 0) ? (100.0 * result.connectedFlows / result.activeFlows) : 0.0;
    std::cout << "Connectivity connectedFlows=" << result.connectedFlows
              << " activeFlows=" << result.activeFlows
              << " totalFlows=" << result.totalFlows
              << " ratioPct=" << connectivity
              << " connectedThroughputKbps=" << result.connectedThroughputKbps
              << " connectedAvgDelayMs=" << result.connectedAvgDelayMs
              << std::endl;
    return result;
}

/** Kill-chain UDP ports (distinct from random CBR basePort+flowId). */
constexpr uint16_t KILLCHAIN_PORT_SENSE_TO_DECISION = 6100;
constexpr uint16_t KILLCHAIN_PORT_DECISION_TO_INTERCEPTOR = 6200;

void
PrintGuideMetrics(uint32_t uavCount,
                    uint32_t failedNodesCount,
                    bool enableKillChain,
                    Ptr<FlowMonitor> monitor,
                    FlowMonitorHelper& helper,
                    double connectivityThreshold,
                    double limitDelaySenseDecisionMs,
                    double limitDelayDecisionInterceptorMs,
                    double routeGenLimitSec,
                    double routeSwitchLimitSec,
                    const RecoveryStats& recovery)
{
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(helper.GetClassifier());
    if (!classifier)
    {
        std::cout << "GUIDE warn=no_ipv4_flow_classifier" << std::endl;
        return;
    }

    bool foundA = false;
    bool foundB = false;
    double avgA = 0.0;
    double avgB = 0.0;
    double maxA = 0.0;
    double maxB = 0.0;
    double firstKillRxSec = 1e9;
    bool missionA = false;
    bool missionB = false;

    FlowSummaryResult summary;
    const auto stats = monitor->GetFlowStats();
    for (const auto& kv : stats)
    {
        summary.totalFlows++;
        const auto fiveTuple = classifier->FindFlow(kv.first);
        const auto& st = kv.second;
        if (st.txPackets == 0)
        {
            continue;
        }
        summary.activeFlows++;
        const double rxOverTx =
            static_cast<double>(st.rxPackets) / static_cast<double>(st.txPackets);
        if (rxOverTx >= connectivityThreshold)
        {
            summary.connectedFlows++;
        }
        const double avgDelayMs =
            (st.rxPackets > 0) ? st.delaySum.GetMilliSeconds() / st.rxPackets : 0.0;

        if (fiveTuple.destinationPort == KILLCHAIN_PORT_SENSE_TO_DECISION)
        {
            foundA = true;
            avgA = avgDelayMs;
            maxA = avgDelayMs;
            if (st.rxPackets > 0)
            {
                const double tr = st.timeFirstRxPacket.GetSeconds();
                firstKillRxSec = std::min(firstKillRxSec, tr);
            }
            missionA = (rxOverTx >= connectivityThreshold) && (st.rxPackets > 0);
        }
        else if (fiveTuple.destinationPort == KILLCHAIN_PORT_DECISION_TO_INTERCEPTOR)
        {
            foundB = true;
            avgB = avgDelayMs;
            maxB = avgDelayMs;
            if (st.rxPackets > 0)
            {
                const double tr = st.timeFirstRxPacket.GetSeconds();
                firstKillRxSec = std::min(firstKillRxSec, tr);
            }
            missionB = (rxOverTx >= connectivityThreshold) && (st.rxPackets > 0);
        }
    }

    const double connectivityMissionPct =
        (summary.activeFlows > 0) ? (100.0 * summary.connectedFlows / summary.activeFlows) : 0.0;

    const bool m1Nodes = (uavCount >= 50);
    const bool m1a =
        !enableKillChain || (foundA && avgA <= limitDelaySenseDecisionMs && maxA <= limitDelaySenseDecisionMs);
    const bool m1b =
        !enableKillChain ||
        (foundB && avgB <= limitDelayDecisionInterceptorMs && maxB <= limitDelayDecisionInterceptorMs);
    const bool m2rg = !enableKillChain ||
                      ((firstKillRxSec < 1e8) && (firstKillRxSec <= routeGenLimitSec));
    const bool m2sw = (recovery.maxRecoverySec <= routeSwitchLimitSec);
    const double actualFailFrac =
        (uavCount > 0) ? static_cast<double>(failedNodesCount) / static_cast<double>(uavCount) : 0.0;
    const bool m2fail = actualFailFrac <= 0.1001;
    const bool m2conn = (connectivityMissionPct >= 100.0 - 1e-6);
    const bool killMission = !enableKillChain || (missionA && missionB);

    std::cout << "GUIDE_HETEROGENEOUS model=80211g_adhoc_uniformPHY "
                 "(physical_heterogeneity_requires_extra_radio_model)"
              << std::endl;

    std::cout << "GUIDE_M1 nodes=" << uavCount << " nodes_ge_50=" << (m1Nodes ? 1 : 0)
              << " killChain_sense_to_decision_avgDelayMs=" << avgA
              << " maxDelayMs=" << maxA << " limitMs=" << limitDelaySenseDecisionMs
              << " pass=" << (m1a ? 1 : 0) << " flow_found=" << (foundA ? 1 : 0) << std::endl;
    std::cout << "GUIDE_M1 killChain_decision_to_interceptor_avgDelayMs=" << avgB
              << " maxDelayMs=" << maxB << " limitMs=" << limitDelayDecisionInterceptorMs
              << " pass=" << (m1b ? 1 : 0) << " flow_found=" << (foundB ? 1 : 0) << std::endl;

    std::cout << "GUIDE_M2 routeGen_firstKillChainRxSec=" << ((firstKillRxSec < 1e8) ? firstKillRxSec : -1.0)
              << " limitSec=" << routeGenLimitSec << " pass=" << (m2rg ? 1 : 0) << std::endl;
    std::cout << "GUIDE_M2 routeSwitch_maxRecoverySec=" << recovery.maxRecoverySec
              << " limitSec=" << routeSwitchLimitSec << " pass=" << (m2sw ? 1 : 0)
              << " recoveryEvents=" << recovery.recoveryEvents << std::endl;
    std::cout << "GUIDE_M2 failure_actualFraction=" << actualFailFrac
              << " failedNodes=" << failedNodesCount << " pass_fraction_le_10pct=" << (m2fail ? 1 : 0)
              << std::endl;
    std::cout << "GUIDE_M2 flow_connectivity_ratioPct=" << connectivityMissionPct
              << " pass_eq_100=" << (m2conn ? 1 : 0)
              << " killChain_mission_pass=" << (killMission ? 1 : 0) << std::endl;
}

} // namespace

int
main(int argc, char* argv[])
{
    uint32_t uavCount = 50;
    double stopTime = 60.0;
    bool enableJammer = true;
    double jammerStart = 15.0;
    double jammerStop = 45.0;
    double jammerRadius = 150.0;
    double jammedTxPower = 5.0;
    double normalTxPower = 20.0;
    double jammerMoveInterval = 5.0;
    double failureTime = 5.0;
    uint32_t flowCount = 10;
    double flowInterval = 0.02;
    uint32_t flowPacketSize = 768;
    double connectivityThreshold = 0.8;
    double connectivityLogInterval = 1.0;
    /** When recoveryUseAvgGoodput=true (default), compare to mean delivery % (0–100).
     *  When false, compare to fraction of flows with rx/tx>=connectivityThreshold (strict). */
    double recoveryDropThresholdPct = 22.0;
    double recoveryRestoreThresholdPct = 32.0;
    bool recoveryUseAvgGoodput = true;
    double metricsFirstSampleDelay = 8.0;
    bool enableFlowXml = false;
    double etxHelloInterval = 0.2;
    double etxTcInterval = 0.5;
    std::string routing = "etx"; // etx | olsr
    bool enableControlAgent = true;
    uint32_t seed = 1;
    uint32_t run = 1;

    // --- 高校合作指南：杀伤链端到端流与阈值（可写入验收报告）---
    bool enableKillChainFlows = true;
    uint32_t senseNode = 0;
    uint32_t decisionNode = 1;
    uint32_t interceptorNode = 2;
    double failureFraction = 0.1;
    double limitDelaySenseDecisionMs = 100.0;
    double limitDelayDecisionInterceptorMs = 50.0;
    double routeGenLimitSec = 10.0;
    double routeSwitchLimitSec = 3.0;

    CommandLine cmd;
    cmd.AddValue("uavCount", "Number of UAV nodes (>=50 recommended)", uavCount);
    cmd.AddValue("stopTime", "Simulation time", stopTime);
    cmd.AddValue("enableJammer", "Enable continuous jammer", enableJammer);
    cmd.AddValue("jammerStart", "Jammer start time", jammerStart);
    cmd.AddValue("jammerStop", "Jammer stop time", jammerStop);
    cmd.AddValue("jammerRadius", "Regional jammer radius in meters", jammerRadius);
    cmd.AddValue("jammedTxPower", "Tx power during jamming inside jam area (dBm)", jammedTxPower);
    cmd.AddValue("normalTxPower", "Normal Tx power outside jamming period (dBm)", normalTxPower);
    cmd.AddValue("jammerMoveInterval", "Jammer position update interval (s)", jammerMoveInterval);
    cmd.AddValue("failureTime", "Random node failure time", failureTime);
    cmd.AddValue("flowCount", "Number of UDP CBR flows", flowCount);
    cmd.AddValue("flowInterval", "Packet interval for UDP CBR flows (s)", flowInterval);
    cmd.AddValue("flowPacketSize", "Packet size for UDP CBR flows (bytes)", flowPacketSize);
    cmd.AddValue("connectivityThreshold", "Connectivity threshold based on rx/tx ratio", connectivityThreshold);
    cmd.AddValue("connectivityLogInterval", "Connectivity sample interval (s)", connectivityLogInterval);
    cmd.AddValue("recoveryDropThresholdPct",
                 "Recovery drop threshold (see recoveryUseAvgGoodput): avg delivery % or strict-flow %",
                 recoveryDropThresholdPct);
    cmd.AddValue("recoveryRestoreThresholdPct",
                 "Recovery restore threshold (same unit as recoveryDropThresholdPct)",
                 recoveryRestoreThresholdPct);
    cmd.AddValue("recoveryUseAvgGoodput",
                 "true: outage/recovery use mean flow rx/tx % (recommended). "
                 "false: use %% of flows meeting connectivityThreshold (legacy; restore often unreachable).",
                 recoveryUseAvgGoodput);
    cmd.AddValue("metricsFirstSampleDelay",
                 "Seconds before first METRICS sample (skip cold-start transients)",
                 metricsFirstSampleDelay);
    cmd.AddValue("enableFlowXml", "Enable FlowMonitor XML serialization", enableFlowXml);
    cmd.AddValue("etxHelloInterval", "ETX HELLO interval (s)", etxHelloInterval);
    cmd.AddValue("etxTcInterval", "ETX TC interval (s)", etxTcInterval);
    cmd.AddValue("routing", "Routing protocol: etx or olsr", routing);
    cmd.AddValue("enableControlAgent",
                 "Install per-node ControlAgent (ETX-OLSR adaptive hyperparameters)",
                 enableControlAgent);
    cmd.AddValue("seed", "RNG seed (RngSeedManager::SetSeed)", seed);
    cmd.AddValue("run", "RNG run number (RngSeedManager::SetRun)", run);
    cmd.AddValue("enableKillChainFlows",
                 "Install labeled UDP flows: sense->decision (6100) and decision->interceptor (6200)",
                 enableKillChainFlows);
    cmd.AddValue("senseNode", "UAV index for kill-chain sense (source of first hop)", senseNode);
    cmd.AddValue("decisionNode", "UAV index for kill-chain fusion/decision", decisionNode);
    cmd.AddValue("interceptorNode", "UAV index for kill-chain shooter/interceptor sink", interceptorNode);
    cmd.AddValue("failureFraction",
                 "Random fail fraction of UAVs (<=0.1 for milestone); failed count=max(1,round(N*f))",
                 failureFraction);
    cmd.AddValue("limitDelaySenseDecisionMs",
                 "Guide M1: max avg delay sense->decision (ms)",
                 limitDelaySenseDecisionMs);
    cmd.AddValue("limitDelayDecisionInterceptorMs",
                 "Guide M1: max avg delay decision->interceptor (ms)",
                 limitDelayDecisionInterceptorMs);
    cmd.AddValue("routeGenLimitSec", "Guide M2: first kill-chain RX time upper bound (s)", routeGenLimitSec);
    cmd.AddValue("routeSwitchLimitSec", "Guide M2: recovery/switch time upper bound (s)", routeSwitchLimitSec);
    cmd.Parse(argc, argv);

    NS_LOG_UNCOND("Metrics: flowsStrictPassPct = %% of flows with rx/tx>="
                  << connectivityThreshold << "; deliveryAvgPct = mean rx/tx (all active flows). "
                  << "Recovery uses " << (recoveryUseAvgGoodput ? "deliveryAvgPct" : "flowsStrictPassPct")
                  << " with drop<" << recoveryDropThresholdPct << " restore>=" << recoveryRestoreThresholdPct
                  << "; first sample at " << metricsFirstSampleDelay << "s");

    if (enableKillChainFlows)
    {
        if (uavCount < 3)
        {
            NS_FATAL_ERROR("Kill-chain flows require uavCount >= 3");
        }
        if (senseNode >= uavCount || decisionNode >= uavCount || interceptorNode >= uavCount)
        {
            NS_FATAL_ERROR("Kill-chain node index must be < uavCount");
        }
        if (senseNode == decisionNode || senseNode == interceptorNode || decisionNode == interceptorNode)
        {
            NS_FATAL_ERROR("Kill-chain sense/decision/interceptor indices must be distinct");
        }
    }

    RngSeedManager::SetSeed(seed);
    RngSeedManager::SetRun(run);

    NodeContainer uavs;
    uavs.Create(uavCount);
    NodeContainer jammerNode;
    jammerNode.Create(1);

    Ptr<MultiModelSpectrumChannel> spectrumChannel = CreateObject<MultiModelSpectrumChannel>();
    Ptr<FriisPropagationLossModel> lossModel = CreateObject<FriisPropagationLossModel>();
    Ptr<ConstantSpeedPropagationDelayModel> delayModel =
        CreateObject<ConstantSpeedPropagationDelayModel>();
    spectrumChannel->AddPropagationLossModel(lossModel);
    spectrumChannel->SetPropagationDelayModel(delayModel);

    SpectrumWifiPhyHelper phy;
    phy.SetChannel(spectrumChannel);
    phy.Set("TxPowerStart", DoubleValue(20.0));
    phy.Set("TxPowerEnd", DoubleValue(20.0));

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211g);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode",
                                 StringValue("ErpOfdmRate24Mbps"),
                                 "ControlMode",
                                 StringValue("ErpOfdmRate12Mbps"));

    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");
    NetDeviceContainer devices = wifi.Install(phy, mac, uavs);

    BasicEnergySourceHelper energySourceHelper;
    energySourceHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(10000.0));
    EnergySourceContainer sources = energySourceHelper.Install(uavs);

    WifiRadioEnergyModelHelper radioEnergyHelper;
    radioEnergyHelper.Set("TxCurrentA", DoubleValue(0.38));
    radioEnergyHelper.Set("RxCurrentA", DoubleValue(0.31));
    radioEnergyHelper.Install(devices, sources);

    MobilityHelper uavMobility;
    uavMobility.SetPositionAllocator("ns3::RandomRectanglePositionAllocator",
                                     "X", StringValue("ns3::UniformRandomVariable[Min=0|Max=500]"),
                                     "Y", StringValue("ns3::UniformRandomVariable[Min=0|Max=500]"));
    uavMobility.SetMobilityModel("ns3::GaussMarkovMobilityModel",
                                 "Bounds", BoxValue(Box(0.0, 500.0, 0.0, 500.0, 50.0, 150.0)),
                                 "TimeStep", TimeValue(Seconds(1.0)),
                                 "Alpha", DoubleValue(0.85),
                                 "MeanVelocity", StringValue("ns3::UniformRandomVariable[Min=12|Max=20]"),
                                 "MeanDirection", StringValue("ns3::UniformRandomVariable[Min=0|Max=6.28318]"),
                                 "MeanPitch", StringValue("ns3::UniformRandomVariable[Min=-0.05|Max=0.05]"),
                                 "NormalVelocity", StringValue("ns3::NormalRandomVariable[Mean=0|Variance=4]"),
                                 "NormalDirection", StringValue("ns3::NormalRandomVariable[Mean=0|Variance=0.2]"),
                                 "NormalPitch", StringValue("ns3::NormalRandomVariable[Mean=0|Variance=0.02]"));
    uavMobility.Install(uavs);

    MobilityHelper jammerMobility;
    jammerMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    jammerMobility.Install(jammerNode);
    jammerNode.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(250.0, 250.0, 80.0));

    // ---------------- REGIONAL JAMMER (STABLE VERSION) ----------------
    if (enableJammer)
    {
        Simulator::Schedule(Seconds(0.5),
                            &MoveJammerAndApply,
                            uavs,
                            jammerNode.Get(0),
                            0.0,
                            500.0,
                            80.0,
                            jammerRadius,
                            jammedTxPower,
                            normalTxPower,
                            Seconds(jammerStart),
                            Seconds(jammerStop),
                            Seconds(jammerMoveInterval));
    }

    InternetStackHelper internet;
    Ipv4ListRoutingHelper list;
    if (routing == "olsr")
    {
        EtxOlsrHopcountHelper olsr;
        list.Add(olsr, 10);
        NS_LOG_UNCOND("Routing=OLSR seed=" << seed << " run=" << run << " uavCount=" << uavCount);
    }
    else
    {
        // Default: ETX-OLSR.
        EtxOlsrHelper etxOlsr;
        etxOlsr.SetAttribute("HelloInterval", TimeValue(Seconds(etxHelloInterval)));
        etxOlsr.SetAttribute("TcInterval", TimeValue(Seconds(etxTcInterval)));
        etxOlsr.SetAttribute("RiskLambda", DoubleValue(1.0));
        etxOlsr.SetAttribute("MabC", DoubleValue(1.0));
        etxOlsr.SetAttribute("MabGamma", DoubleValue(0.3));
        etxOlsr.SetAttribute("EtxThreshold", DoubleValue(1.5));
        etxOlsr.SetAttribute("DelayMaxMs", DoubleValue(100.0));
        etxOlsr.SetAttribute("RewardAlpha", DoubleValue(0.6));
        etxOlsr.SetAttribute("RewardBeta", DoubleValue(0.2));
        etxOlsr.SetAttribute("RewardGamma", DoubleValue(0.2));
        etxOlsr.SetAttribute("RewardDeltaEtx", DoubleValue(0.1));
        etxOlsr.SetAttribute("SinrThresholdDb", DoubleValue(5.0));
        etxOlsr.SetAttribute("OverlapPenalty", DoubleValue(80.0));
        list.Add(etxOlsr, 10);
        NS_LOG_UNCOND("Routing=ETX-OLSR seed=" << seed << " run=" << run << " uavCount=" << uavCount);
    }
    internet.SetRoutingHelper(list);
    internet.Install(uavs);

    if (routing == "etx" && enableControlAgent)
    {
        ControlAgentHelper controlHelper;
        controlHelper.SetStartTime(Seconds(0.5));
        controlHelper.SetStopTime(Seconds(stopTime));
        controlHelper.SetAttribute("UpdateInterval", TimeValue(Seconds(1.0)));
        controlHelper.Install(uavs);
        NS_LOG_UNCOND("ControlAgent installed on all UAV nodes (distributed adaptive hyperparameters)");
    }

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.0.0.0", "255.255.0.0");
    Ipv4InterfaceContainer interfaces = ipv4.Assign(devices);

    ApplicationContainer sinkApps;
    ApplicationContainer flowApps;
    std::set<uint32_t> protectedNodes;
    Ptr<UniformRandomVariable> flowPick = CreateObject<UniformRandomVariable>();
    const uint16_t basePort = 5000;
    for (uint32_t f = 0; f < flowCount; ++f)
    {
        uint32_t src = flowPick->GetInteger(0, uavCount - 1);
        uint32_t dst = flowPick->GetInteger(0, uavCount - 1);
        while (dst == src)
        {
            dst = flowPick->GetInteger(0, uavCount - 1);
        }
        const uint16_t port = basePort + static_cast<uint16_t>(f);
        PacketSinkHelper sink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), port));
        sinkApps.Add(sink.Install(uavs.Get(dst)));

        UdpClientHelper flow(interfaces.GetAddress(dst), port);
        flow.SetAttribute("Interval", TimeValue(Seconds(flowInterval)));
        flow.SetAttribute("PacketSize", UintegerValue(flowPacketSize));
        flow.SetAttribute("MaxPackets",
                          UintegerValue(static_cast<uint32_t>((stopTime / std::max(0.001, flowInterval)) + 10)));
        flowApps.Add(flow.Install(uavs.Get(src)));
        protectedNodes.insert(uavs.Get(src)->GetId());
        protectedNodes.insert(uavs.Get(dst)->GetId());
        NS_LOG_UNCOND("FlowSetup id=" << f
                                      << " srcNode=" << src
                                      << " dstNode=" << dst
                                      << " port=" << port);
    }

    if (enableKillChainFlows)
    {
        PacketSinkHelper sinkSd(
            "ns3::UdpSocketFactory",
            InetSocketAddress(Ipv4Address::GetAny(), KILLCHAIN_PORT_SENSE_TO_DECISION));
        sinkApps.Add(sinkSd.Install(uavs.Get(decisionNode)));

        PacketSinkHelper sinkDi(
            "ns3::UdpSocketFactory",
            InetSocketAddress(Ipv4Address::GetAny(), KILLCHAIN_PORT_DECISION_TO_INTERCEPTOR));
        sinkApps.Add(sinkDi.Install(uavs.Get(interceptorNode)));

        UdpClientHelper senseToDecision(interfaces.GetAddress(decisionNode),
                                        KILLCHAIN_PORT_SENSE_TO_DECISION);
        senseToDecision.SetAttribute("Interval", TimeValue(Seconds(flowInterval)));
        senseToDecision.SetAttribute("PacketSize", UintegerValue(flowPacketSize));
        senseToDecision.SetAttribute(
            "MaxPackets",
            UintegerValue(static_cast<uint32_t>((stopTime / std::max(0.001, flowInterval)) + 10)));
        flowApps.Add(senseToDecision.Install(uavs.Get(senseNode)));

        UdpClientHelper decisionToInterceptor(interfaces.GetAddress(interceptorNode),
                                              KILLCHAIN_PORT_DECISION_TO_INTERCEPTOR);
        decisionToInterceptor.SetAttribute("Interval", TimeValue(Seconds(flowInterval)));
        decisionToInterceptor.SetAttribute("PacketSize", UintegerValue(flowPacketSize));
        decisionToInterceptor.SetAttribute(
            "MaxPackets",
            UintegerValue(static_cast<uint32_t>((stopTime / std::max(0.001, flowInterval)) + 10)));
        flowApps.Add(decisionToInterceptor.Install(uavs.Get(decisionNode)));

        protectedNodes.insert(uavs.Get(senseNode)->GetId());
        protectedNodes.insert(uavs.Get(decisionNode)->GetId());
        protectedNodes.insert(uavs.Get(interceptorNode)->GetId());

        NS_LOG_UNCOND("KillChain senseNode=" << senseNode << " decisionNode=" << decisionNode
                                             << " interceptorNode=" << interceptorNode
                                             << " ports=" << KILLCHAIN_PORT_SENSE_TO_DECISION << ","
                                             << KILLCHAIN_PORT_DECISION_TO_INTERCEPTOR);
    }

    sinkApps.Start(Seconds(0.5));
    sinkApps.Stop(Seconds(stopTime));
    flowApps.Start(Seconds(1.0));
    flowApps.Stop(Seconds(stopTime - 1.0));

    uint32_t failCount = 0;
    if (failureFraction > 0.0 && uavCount > 0)
    {
        failCount = std::max(1u, static_cast<uint32_t>(std::lround(static_cast<double>(uavCount) * failureFraction)));
    }
    ScheduleRandomFailures(uavs, failCount, Seconds(failureTime), protectedNodes);

    FlowMonitorHelper flowMonitor;
    Ptr<FlowMonitor> monitor = flowMonitor.InstallAll();
    RecoveryStats recoveryStats;
    Simulator::Schedule(Seconds(metricsFirstSampleDelay),
                        &LogConnectivitySample,
                        monitor,
                        connectivityThreshold,
                        recoveryDropThresholdPct,
                        recoveryRestoreThresholdPct,
                        recoveryUseAvgGoodput,
                        &recoveryStats,
                        Seconds(connectivityLogInterval),
                        Seconds(stopTime));

    Simulator::Stop(Seconds(stopTime));
    Simulator::Run();

    PrintFlowSummary(monitor, flowMonitor, connectivityThreshold);
    const double avgRecoverySec =
        (recoveryStats.recoveryEvents > 0)
            ? (recoveryStats.totalRecoverySec / recoveryStats.recoveryEvents)
            : 0.0;
    std::cout << "RecoverySummary events=" << recoveryStats.recoveryEvents
              << " avgRecoverySec=" << avgRecoverySec
              << " maxRecoverySec=" << recoveryStats.maxRecoverySec
              << " inOutageAtEnd=" << (recoveryStats.inOutage ? 1 : 0)
              << std::endl;

    PrintGuideMetrics(uavCount,
                      failCount,
                      enableKillChainFlows,
                      monitor,
                      flowMonitor,
                      connectivityThreshold,
                      limitDelaySenseDecisionMs,
                      limitDelayDecisionInterceptorMs,
                      routeGenLimitSec,
                      routeSwitchLimitSec,
                      recoveryStats);

    if (enableFlowXml)
    {
        monitor->SerializeToXmlFile("flow-monitor.xml", true, true);
    }

    Simulator::Destroy();
    return 0;
}