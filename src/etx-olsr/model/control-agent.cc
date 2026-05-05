/*
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "control-agent.h"

#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/ipv4.h"
#include "ns3/ipv4-list-routing.h"
#include "ns3/log.h"
#include "ns3/node.h"
#include "ns3/nstime.h"
#include "ns3/simulator.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace ns3
{
namespace etxolsr
{

NS_LOG_COMPONENT_DEFINE("EtxOlsrControlAgent");

NS_OBJECT_ENSURE_REGISTERED(ControlAgent);

TypeId
ControlAgent::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::etxolsr::ControlAgent")
            .SetParent<Application>()
            .SetGroupName("EtxOlsr")
            .AddConstructor<ControlAgent>()
            .AddAttribute("UpdateInterval",
                          "Wall-clock interval between control updates.",
                          TimeValue(Seconds(1.0)),
                          MakeTimeAccessor(&ControlAgent::m_updateInterval),
                          MakeTimeChecker())
            .AddAttribute("SigmaHigh",
                          "If mean sqrt(variance of reward innovation) exceeds this, increase MabC.",
                          DoubleValue(0.25),
                          MakeDoubleAccessor(&ControlAgent::m_sigmaHigh),
                          MakeDoubleChecker<double>())
            .AddAttribute("SigmaLow",
                          "If mean sqrt(variance) is below this, decrease MabC.",
                          DoubleValue(0.08),
                          MakeDoubleAccessor(&ControlAgent::m_sigmaLow),
                          MakeDoubleChecker<double>())
            .AddAttribute("DelayHighMs",
                          "If mean EWMA delay (ms) exceeds this, increase MabC slightly.",
                          DoubleValue(80.0),
                          MakeDoubleAccessor(&ControlAgent::m_delayHighMs),
                          MakeDoubleChecker<double>())
            .AddAttribute("SwitchHighPerSec",
                          "If path switches in the last 1 s exceed this, increase switch penalty.",
                          DoubleValue(2.0),
                          MakeDoubleAccessor(&ControlAgent::m_switchHighPerSec),
                          MakeDoubleChecker<double>())
            .AddAttribute("SwitchLowPerSec",
                          "If path switches in the last 1 s are below this, decrease switch penalty.",
                          DoubleValue(0.25),
                          MakeDoubleAccessor(&ControlAgent::m_switchLowPerSec),
                          MakeDoubleChecker<double>())
            .AddAttribute("MabCMin",
                          "Lower clamp for MabC.",
                          DoubleValue(0.2),
                          MakeDoubleAccessor(&ControlAgent::m_mabCMin),
                          MakeDoubleChecker<double>())
            .AddAttribute("MabCMax",
                          "Upper clamp for MabC.",
                          DoubleValue(3.0),
                          MakeDoubleAccessor(&ControlAgent::m_mabCMax),
                          MakeDoubleChecker<double>())
            .AddAttribute("SwitchPenaltyMin",
                          "Lower clamp for switch penalty.",
                          DoubleValue(0.02),
                          MakeDoubleAccessor(&ControlAgent::m_switchPenaltyMin),
                          MakeDoubleChecker<double>())
            .AddAttribute("SwitchPenaltyMax",
                          "Upper clamp for switch penalty.",
                          DoubleValue(2.0),
                          MakeDoubleAccessor(&ControlAgent::m_switchPenaltyMax),
                          MakeDoubleChecker<double>())
            .AddAttribute("SmoothingBeta",
                          "Convex blend toward rule targets: out=(1-β)*current+β*target, β in [0,1].",
                          DoubleValue(0.35),
                          MakeDoubleAccessor(&ControlAgent::m_smoothingBeta),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("ContinuousSigmaScaling",
                          "If true, vary MabC scale factor linearly with meanSigma between SigmaLow and "
                          "SigmaHigh (smoother than discrete thresholds).",
                          BooleanValue(true),
                          MakeBooleanAccessor(&ControlAgent::m_continuousSigmaScaling),
                          MakeBooleanChecker())
            .AddAttribute("LogUpdates",
                          "Print each adaptation step to stdout.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&ControlAgent::m_logUpdates),
                          MakeBooleanChecker());
    return tid;
}

ControlAgent::ControlAgent()
    : m_routing(nullptr),
      m_updateInterval(Seconds(1.0)),
      m_baseMabC(1.0),
      m_baseSwitchPenalty(0.1),
      m_sigmaHigh(0.25),
      m_sigmaLow(0.08),
      m_delayHighMs(80.0),
      m_switchHighPerSec(2.0),
      m_switchLowPerSec(0.25),
      m_mabCMin(0.2),
      m_mabCMax(3.0),
      m_switchPenaltyMin(0.02),
      m_switchPenaltyMax(2.0),
      m_smoothingBeta(0.35),
      m_continuousSigmaScaling(true),
      m_logUpdates(false)
{
}

ControlAgent::~ControlAgent() = default;

void
ControlAgent::DoDispose()
{
    m_routing = nullptr;
    Simulator::Cancel(m_tickEvent);
    Application::DoDispose();
}

void
ControlAgent::StartApplication()
{
    Ptr<Node> node = GetNode();
    if (!node)
    {
        NS_LOG_WARN("ControlAgent started without a node");
        return;
    }

    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
    if (!ipv4)
    {
        NS_LOG_WARN("ControlAgent: no Ipv4 on node " << node->GetId());
        return;
    }

    Ptr<Ipv4RoutingProtocol> rp = ipv4->GetRoutingProtocol();
    Ptr<Ipv4ListRouting> listRp = DynamicCast<Ipv4ListRouting>(rp);
    if (listRp)
    {
        for (uint32_t i = 0; i < listRp->GetNRoutingProtocols(); ++i)
        {
            int16_t prio = 0;
            Ptr<Ipv4RoutingProtocol> sub = listRp->GetRoutingProtocol(i, prio);
            m_routing = DynamicCast<RoutingProtocol>(sub);
            if (m_routing)
            {
                break;
            }
        }
    }
    else
    {
        m_routing = DynamicCast<RoutingProtocol>(rp);
    }

    if (!m_routing)
    {
        NS_LOG_WARN("ControlAgent: ETX-OLSR not found on node " << node->GetId());
        return;
    }

    m_baseMabC = m_routing->GetMabC();
    m_baseSwitchPenalty = m_routing->GetSwitchPenalty();
    ScheduleNextTick();
}

void
ControlAgent::StopApplication()
{
    Simulator::Cancel(m_tickEvent);
}

void
ControlAgent::ScheduleNextTick()
{
    if (!m_routing || m_updateInterval <= Time(0))
    {
        return;
    }
    m_tickEvent = Simulator::Schedule(m_updateInterval, &ControlAgent::Tick, this);
}

void
ControlAgent::Tick()
{
    if (!m_routing)
    {
        return;
    }

    double meanSigma = 0.0;
    double meanDelayMs = 0.0;
    double switchRate = 0.0;
    uint32_t nArms = 0;
    m_routing->CollectControlPlaneSignals(meanSigma, meanDelayMs, switchRate, nArms);

    if (nArms == 0)
    {
        ScheduleNextTick();
        return;
    }

    // ---- Rule-based targets from deployment baseline (interpretable policy) ----
    double targetMabC = m_baseMabC;
    double targetSwitchPen = m_baseSwitchPenalty;

    if (m_continuousSigmaScaling && (m_sigmaHigh > m_sigmaLow + 1e-9))
    {
        const double span = m_sigmaHigh - m_sigmaLow;
        const double t = std::clamp((meanSigma - m_sigmaLow) / span, 0.0, 1.0);
        // Low uncertainty -> slightly less exploration; high -> more (smooth across the band).
        constexpr double kMabFactorLo = 0.92;
        constexpr double kMabFactorHi = 1.22;
        targetMabC *= (kMabFactorLo + t * (kMabFactorHi - kMabFactorLo));
    }
    else
    {
        if (meanSigma > m_sigmaHigh)
        {
            targetMabC *= 1.25;
        }
        else if (meanSigma < m_sigmaLow)
        {
            targetMabC *= 0.88;
        }
    }

    if (meanDelayMs > m_delayHighMs)
    {
        targetMabC *= 1.12;
    }

    if (switchRate > m_switchHighPerSec)
    {
        targetSwitchPen = std::min(m_switchPenaltyMax, targetSwitchPen * 1.2);
        targetMabC *= 0.92;
    }
    else if (switchRate < m_switchLowPerSec)
    {
        targetSwitchPen = std::max(m_switchPenaltyMin, targetSwitchPen * 0.9);
    }

    targetMabC = std::min(m_mabCMax, std::max(m_mabCMin, targetMabC));
    targetSwitchPen = std::min(m_switchPenaltyMax, std::max(m_switchPenaltyMin, targetSwitchPen));

    // ---- Memory + smooth tracking: blend targets with current hyperparameters ----
    const double curMabC = m_routing->GetMabC();
    const double curSwitchPen = m_routing->GetSwitchPenalty();
    const double beta = std::clamp(m_smoothingBeta, 0.0, 1.0);
    double mabC = (1.0 - beta) * curMabC + beta * targetMabC;
    double switchPen = (1.0 - beta) * curSwitchPen + beta * targetSwitchPen;

    mabC = std::min(m_mabCMax, std::max(m_mabCMin, mabC));
    switchPen = std::min(m_switchPenaltyMax, std::max(m_switchPenaltyMin, switchPen));

    m_routing->SetAdaptiveHyperparameters(mabC, switchPen);

    if (m_logUpdates)
    {
        std::cout << "CONTROL_AGENT node=" << GetNode()->GetId() << " t=" << Simulator::Now().GetSeconds()
                  << " arms=" << nArms << " meanSigmaSqrt=" << meanSigma << " meanDelayMs=" << meanDelayMs
                  << " switchPer1s=" << switchRate << " beta=" << beta << " targetMabC=" << targetMabC
                  << " targetSwitchPen=" << targetSwitchPen << " mabC=" << mabC << " switchPenalty=" << switchPen
                  << std::endl;
    }

    ScheduleNextTick();
}

} // namespace etxolsr
} // namespace ns3
