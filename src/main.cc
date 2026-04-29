#include <algorithm>
#include <iostream>
#include <set>
#include <vector>

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/energy-module.h"
#include "ns3/etx-olsr-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/spectrum-module.h"
#include "ns3/wifi-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("AntiDroneSimulation");

namespace
{
void
DisposeNode(Ptr<Node> node)
{
    NS_LOG_WARN("Disabling node " << node->GetId() << " at " << Simulator::Now().GetSeconds()
                                  << "s");
    node->Dispose();
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
        Simulator::Schedule(failTime, &DisposeNode, uavs.Get(idx));
        candidates.erase(candidates.begin() + pick);
    }
}

void
PrintFlowSummary(Ptr<FlowMonitor> monitor, FlowMonitorHelper& helper)
{
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(helper.GetClassifier());
    const auto stats = monitor->GetFlowStats();
    for (const auto& kv : stats)
    {
        const auto fiveTuple = classifier->FindFlow(kv.first);
        const auto& st = kv.second;
        const double duration =
            std::max(1e-6, (st.timeLastRxPacket - st.timeFirstTxPacket).GetSeconds());
        const double throughputKbps = (st.rxBytes * 8.0) / duration / 1000.0;
        const double avgDelayMs =
            (st.rxPackets > 0) ? st.delaySum.GetMilliSeconds() / st.rxPackets : 0.0;
        std::cout << "Flow " << kv.first << " " << fiveTuple.sourceAddress << " -> "
                  << fiveTuple.destinationAddress << " tx=" << st.txPackets
                  << " rx=" << st.rxPackets << " lost=" << st.lostPackets
                  << " avgDelayMs=" << avgDelayMs << " throughputKbps=" << throughputKbps
                  << std::endl;
    }
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
    double failureTime = 5.0;

    CommandLine cmd;
    cmd.AddValue("uavCount", "Number of UAV nodes (>=50 recommended)", uavCount);
    cmd.AddValue("stopTime", "Simulation time", stopTime);
    cmd.AddValue("enableJammer", "Enable continuous jammer", enableJammer);
    cmd.AddValue("jammerStart", "Jammer start time", jammerStart);
    cmd.AddValue("jammerStop", "Jammer stop time", jammerStop);
    cmd.AddValue("failureTime", "Random node failure time", failureTime);
    cmd.Parse(argc, argv);

    NodeContainer uavs;
    uavs.Create(uavCount);
    NodeContainer jammer;
    jammer.Create(1);

    NodeContainer allNodes;
    allNodes.Add(uavs);
    allNodes.Add(jammer);

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
    wifi.SetStandard(WIFI_STANDARD_802_11g);
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
                                     "X",
                                     StringValue("ns3::UniformRandomVariable[Min=0|Max=500]"),
                                     "Y",
                                     StringValue("ns3::UniformRandomVariable[Min=0|Max=500]"));
    uavMobility.SetMobilityModel("ns3::GaussMarkovMobilityModel",
                                 "Bounds",
                                 BoxValue(Box(0.0, 500.0, 0.0, 500.0, 50.0, 150.0)),
                                 "TimeStep",
                                 TimeValue(Seconds(1.0)),
                                 "Alpha",
                                 DoubleValue(0.85),
                                 "MeanVelocity",
                                 StringValue("ns3::UniformRandomVariable[Min=12|Max=20]"),
                                 "MeanDirection",
                                 StringValue("ns3::UniformRandomVariable[Min=0|Max=6.28318]"),
                                 "MeanPitch",
                                 StringValue("ns3::UniformRandomVariable[Min=-0.05|Max=0.05]"),
                                 "NormalVelocity",
                                 StringValue("ns3::NormalRandomVariable[Mean=0|Variance=4]"),
                                 "NormalDirection",
                                 StringValue("ns3::NormalRandomVariable[Mean=0|Variance=0.2]"),
                                 "NormalPitch",
                                 StringValue("ns3::NormalRandomVariable[Mean=0|Variance=0.02]"));
    uavMobility.Install(uavs);

    MobilityHelper jammerMobility;
    jammerMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    jammerMobility.Install(jammer);
    jammer.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(250.0, 250.0, 80.0));

    InternetStackHelper internet;
    EtxOlsrHelper etxOlsr;
    etxOlsr.SetAttribute("HelloInterval", TimeValue(Seconds(0.5)));
    etxOlsr.SetAttribute("TcInterval", TimeValue(Seconds(1.0)));
    etxOlsr.SetAttribute("RiskLambda", DoubleValue(1.0));
    etxOlsr.SetAttribute("MabC", DoubleValue(1.0));
    etxOlsr.SetAttribute("MabGamma", DoubleValue(0.3));
    etxOlsr.SetAttribute("EtxThreshold", DoubleValue(1.5));
    etxOlsr.SetAttribute("DelayMaxMs", DoubleValue(100.0));
    etxOlsr.SetAttribute("RewardAlpha", DoubleValue(0.6));
    etxOlsr.SetAttribute("RewardBeta", DoubleValue(0.2));
    etxOlsr.SetAttribute("RewardGamma", DoubleValue(0.2));
    // Make ETX contribute to the data-plane reward (see RewardDeltaEtx in RoutingProtocol).
    etxOlsr.SetAttribute("RewardDeltaEtx", DoubleValue(0.1));
    etxOlsr.SetAttribute("SinrThresholdDb", DoubleValue(5.0));
    etxOlsr.SetAttribute("OverlapPenalty", DoubleValue(80.0));

    Ipv4ListRoutingHelper list;
    list.Add(etxOlsr, 10);
    internet.SetRoutingHelper(list);
    internet.Install(uavs);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.0.0.0", "255.255.0.0");
    Ipv4InterfaceContainer interfaces = ipv4.Assign(devices);

    uint16_t sensePort = 5000;
    uint16_t controlPort = 5001;

    PacketSinkHelper senseSink("ns3::UdpSocketFactory",
                               InetSocketAddress(Ipv4Address::GetAny(), sensePort));
    PacketSinkHelper controlSink("ns3::UdpSocketFactory",
                                 InetSocketAddress(Ipv4Address::GetAny(), controlPort));

    ApplicationContainer sinkApps;
    sinkApps.Add(senseSink.Install(uavs.Get(0)));
    sinkApps.Add(controlSink.Install(uavs.Get(1)));
    sinkApps.Start(Seconds(0.5));
    sinkApps.Stop(Seconds(stopTime));

    UdpClientHelper sensingFlow(interfaces.GetAddress(0), sensePort);
    sensingFlow.SetAttribute("Interval", TimeValue(Seconds(0.01)));
    sensingFlow.SetAttribute("PacketSize", UintegerValue(1024));
    sensingFlow.SetAttribute("MaxPackets", UintegerValue(static_cast<uint32_t>(stopTime * 1000)));

    UdpClientHelper controlFlow(interfaces.GetAddress(1), controlPort);
    controlFlow.SetAttribute("Interval", TimeValue(Seconds(0.05)));
    controlFlow.SetAttribute("PacketSize", UintegerValue(512));
    controlFlow.SetAttribute("MaxPackets", UintegerValue(static_cast<uint32_t>(stopTime * 200)));

    ApplicationContainer flowApps;
    flowApps.Add(sensingFlow.Install(uavs.Get(10)));
    flowApps.Add(controlFlow.Install(uavs.Get(20)));
    flowApps.Start(Seconds(1.0));
    flowApps.Stop(Seconds(stopTime - 1.0));

    if (enableJammer)
    {
        WaveformGeneratorHelper waveform;
        waveform.SetChannel(spectrumChannel);
        waveform.SetTxPowerSpectralDensity(IntegralValue(1.0));
        ApplicationContainer jammerApps = waveform.Install(jammer);
        jammerApps.Start(Seconds(jammerStart));
        jammerApps.Stop(Seconds(jammerStop));
    }

    std::set<uint32_t> protectedNodes = {uavs.Get(0)->GetId(),
                                         uavs.Get(1)->GetId(),
                                         uavs.Get(10)->GetId(),
                                         uavs.Get(20)->GetId()};
    ScheduleRandomFailures(uavs, std::max(1u, uavCount / 10), Seconds(failureTime), protectedNodes);

    FlowMonitorHelper flowMonitor;
    Ptr<FlowMonitor> monitor = flowMonitor.InstallAll();

    Simulator::Stop(Seconds(stopTime));
    Simulator::Run();

    PrintFlowSummary(monitor, flowMonitor);
    monitor->SerializeToXmlFile("flow-monitor.xml", true, true);

    Simulator::Destroy();
    return 0;
}
