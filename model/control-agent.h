/*
 * Lightweight per-node control plane: adapts ETX-OLSR bandit hyperparameters
 * from local statistics (variance, delay, path switching).
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef ETX_OLSR_CONTROL_AGENT_H
#define ETX_OLSR_CONTROL_AGENT_H

#include "etx-olsr-routing-protocol.h"

#include "ns3/application.h"
#include "ns3/event-id.h"
#include "ns3/nstime.h"
#include "ns3/ptr.h"

namespace ns3
{
namespace etxolsr
{

/**
 * @ingroup etx-olsr
 *
 * Distributed rule-based controller: each node periodically reads
 * CollectControlPlaneSignals() from its local ETX-OLSR instance and
 * updates MabC / SwitchPenalty. No centralized state.
 */
class ControlAgent : public Application
{
  public:
    static TypeId GetTypeId();

    ControlAgent();
    ~ControlAgent() override;

  protected:
    void DoDispose() override;
    void StartApplication() override;
    void StopApplication() override;

  private:
    void ScheduleNextTick();
    void Tick();

    Ptr<RoutingProtocol> m_routing;
    EventId m_tickEvent;

    Time m_updateInterval;

    /** Baseline hyperparameters captured at StartApplication. */
    double m_baseMabC;
    double m_baseSwitchPenalty;

    /** Mean sqrt(sigma) above this → increase exploration. */
    double m_sigmaHigh;
    /** Mean sqrt(sigma) below this → decrease exploration. */
    double m_sigmaLow;

    /** Mean smoothed delay (ms) above this → bump exploration slightly. */
    double m_delayHighMs;

    /** Path switches in the last 1 s above this → raise switch penalty. */
    double m_switchHighPerSec;
    /** Path switches below this → allow lower switch penalty. */
    double m_switchLowPerSec;

    double m_mabCMin;
    double m_mabCMax;
    double m_switchPenaltyMin;
    double m_switchPenaltyMax;

    /**
     * Blend toward rule-based targets each tick: out = (1-β)*current + β*target.
     * β=1 reproduces per-tick reset from baseline rules; β<1 adds memory and smooths.
     */
    double m_smoothingBeta;

    /** If true, scale MabC factor continuously with meanSigma between SigmaLow and SigmaHigh. */
    bool m_continuousSigmaScaling;

    bool m_logUpdates;
};

} // namespace etxolsr
} // namespace ns3

#endif /* ETX_OLSR_CONTROL_AGENT_H */
