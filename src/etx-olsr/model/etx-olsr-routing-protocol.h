/*
 * ETX-OLSR: OLSR routing with ETX-based link metric.
 *
 * Based on the ns-3 OLSR module (src/olsr).
 * Copyright (c) 2004 Francisco J. Ros
 * Copyright (c) 2007 INESC Porto
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * ETX metric additions inspired by moonhwi/NS3_OLSR_ETX.
 */

#ifndef ETX_OLSR_ROUTING_PROTOCOL_H
#define ETX_OLSR_ROUTING_PROTOCOL_H

#include "ns3/olsr-header.h"
#include "ns3/olsr-repositories.h"
#include "ns3/olsr-state.h"

#include "ns3/event-garbage-collector.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/ipv4-static-routing.h"
#include "ns3/ipv4.h"
#include "ns3/node.h"
#include "ns3/object.h"
#include "ns3/packet.h"
#include "ns3/random-variable-stream.h"
#include "ns3/socket.h"
#include "ns3/timer.h"
#include "ns3/traced-callback.h"
#include "ns3/output-stream-wrapper.h"
#include "ns3/tag.h"

#include <array>
#include <deque>
#include <map>
#include <set>
#include <vector>
#include <limits>
#include <cmath>

namespace ns3
{
namespace etxolsr
{

/**
 * @ingroup etx-olsr
 * Packet tag carrying a send-time timestamp for single-hop delay measurement.
 */
class TimestampTag : public Tag
{
public:
  static TypeId GetTypeId();
  TypeId GetInstanceTypeId() const override;
  uint32_t GetSerializedSize() const override;
  void Serialize(TagBuffer i) const override;
  void Deserialize(TagBuffer i) override;
  void Print(std::ostream& os) const override;
  void SetTimestamp(Time t);
  Time GetTimestamp() const;
private:
  Time m_timestamp;
};

/**
 * @ingroup etx-olsr
 *
 * Routing table entry for ETX-OLSR. Extends the standard OLSR entry with a
 * cumulative ETX path cost (etxDistance). The hop-count distance is kept
 * for debugging and tie-breaking purposes.
 */
struct RoutingTableEntry
{
  Ipv4Address destAddr;   //!< Address of the destination node.
  Ipv4Address nextAddr;   //!< Address of the next hop.
  uint32_t interface;     //!< Interface index.
  uint32_t distance;      //!< Hop-count distance to the destination.
  double etxDistance;     //!< Cumulative ETX path cost.

  RoutingTableEntry()
      : destAddr(),
        nextAddr(),
        interface(0),
        distance(0),
        etxDistance(0.0)
  {
  }
};

/**
 * @ingroup etx-olsr
 *
 * Per-neighbor ETX state maintained locally.
 */
struct EtxInfo
{
  double prr;           //!< EWMA packet reception ratio estimate [0..1].
  Time lastHelloTime;   //!< Simulator time when last HELLO was received from this neighbor.
  Time helloInterval;   //!< The sender's advertised HELLO interval.

  EtxInfo()
      : prr(0.0),
        lastHelloTime(Seconds(0.0)),
        helloInterval(Seconds(2.0))
  {
  }
};

class RoutingProtocol : public Ipv4RoutingProtocol
{
public:
  static const uint16_t OLSR_PORT_NUMBER;

  static TypeId GetTypeId();

  RoutingProtocol();
  ~RoutingProtocol() override;

  void SetMainInterface(uint32_t interface);
  void Dump();
  std::vector<RoutingTableEntry> GetRoutingTableEntries() const;

  olsr::MprSet GetMprSet() const;
  const olsr::MprSelectorSet& GetMprSelectors() const;
  const olsr::NeighborSet& GetNeighbors() const;
  const olsr::TwoHopNeighborSet& GetTwoHopNeighbors() const;
  const olsr::TopologySet& GetTopologySet() const;
  const olsr::OlsrState& GetOlsrState() const;

  int64_t AssignStreams(int64_t stream);

  typedef void (*PacketTxRxTracedCallback)(const olsr::PacketHeader& header,
                                           const olsr::MessageList& messages);
  typedef void (*TableChangeTracedCallback)(uint32_t size);

private:
  std::set<uint32_t> m_interfaceExclusions;
  Ptr<Ipv4StaticRouting> m_routingTableAssociation;

public:
  std::set<uint32_t> GetInterfaceExclusions() const
  {
    return m_interfaceExclusions;
  }

  void SetInterfaceExclusions(std::set<uint32_t> exceptions);

  void AddHostNetworkAssociation(Ipv4Address networkAddr, Ipv4Mask netmask);
  void RemoveHostNetworkAssociation(Ipv4Address networkAddr, Ipv4Mask netmask);
  void SetRoutingTableAssociation(Ptr<Ipv4StaticRouting> routingTable);
  Ptr<const Ipv4StaticRouting> GetRoutingTableAssociation() const;

protected:
  void DoInitialize() override;
  void DoDispose() override;

private:
  // ---- Routing table ----
  std::map<Ipv4Address, RoutingTableEntry> m_table;
  Ptr<Ipv4StaticRouting> m_hnaRoutingTable;
  EventGarbageCollector m_events;

  uint16_t m_packetSequenceNumber;
  uint16_t m_messageSequenceNumber;
  uint16_t m_ansn;

  // ---- OLSR timers / intervals ----
  Time m_helloInterval;
  Time m_tcInterval;
  Time m_midInterval;
  Time m_hnaInterval;
  olsr::Willingness m_willingness;

  olsr::OlsrState m_state;
  Ptr<Ipv4> m_ipv4;

  // ---- ETX state ----
  std::map<Ipv4Address, EtxInfo> m_etxMap;
  double m_etxAlpha;
  double m_initialEtx;

  // ---- Risk-aware routing state ----
  /// Per-next-hop EWMA variance of path-cost prediction error (σ²).
  std::map<Ipv4Address, double> m_sigma;
  /// Previous candidate cost through each next-hop, used to compute prediction error.
  std::map<Ipv4Address, double> m_prevCandCost;
  /// EWMA smoothing factor β for σ² update (0 < β ≤ 1).
  double m_beta;
  /// Risk weight λ: score = V̂ + λ·σ.
  double m_lambda;

  // ---- Phase-1 candidate routing (K=2) ----
  struct CandidateRoute
  {
    Ipv4Address nextHop;
    uint32_t interface;
    double ctrlCost;
    CandidateRoute()
        : nextHop(),
          interface(0),
          ctrlCost(std::numeric_limits<double>::infinity())
    {
    }
  };

  using CandidateSet = std::array<CandidateRoute, 3>;
  std::map<Ipv4Address, CandidateSet> m_candidateTable;

  // ---- MAB (SW-UCB) state for data-plane arm selection ----
  /**
   * Per-arm statistics for the Sliding-Window UCB (SW-UCB) bandit.
   *
   * The reward for each arm is the **instantaneous** link ETX observed at
   * forwarding time (a real data-plane measurement), NOT the control-plane
   * Dijkstra cost.  Only the W most recent rewards are kept, so the bandit
   * can track non-stationary interference without being anchored to stale
   * history (solves the non-stationarity problem of vanilla UCB1).
   *
   * SW-UCB score: mean(window) + c · sqrt( ln(t) / |window| )
   *   where t = cumulative total decisions (drives exploration bonus decay)
   *   and |window| = current window occupancy (≤ W).
   */
  struct MabArm
  {
    std::deque<double> window;        //!< Sliding window of recent instantaneous rewards.
    std::deque<double> rewardHistory; //!< Last 5 normalized rewards for trend detection.
    uint64_t totalCount;              //!< Cumulative selection count across all time.

    MabArm()
        : totalCount(0)
    {
    }
  };

  /// Per-next-hop MAB arm statistics.
  std::map<Ipv4Address, MabArm> m_mabArms;
  /// Total number of data-plane routing decisions made so far (t).
  uint64_t m_mabTotalDecisions;
  /// SW-UCB exploration constant c (0 disables exploration → pure exploitation).
  double m_mabC;
  /// Sliding-window size W for SW-UCB; only the W most recent rewards are retained.
  uint32_t m_mabWindow;
  /// Extra ETX penalty added when the backup path's penultimate node lies on the primary path.
  double m_diversityPenalty;

  // ---- Data-plane delay state ----
  /// Per-next-hop EWMA smoothed one-hop delay (seconds).
  std::map<Ipv4Address, double> m_smoothDelay;
  /// EWMA factor for delay smoothing (0=freeze old value, 1=no memory/use latest).
  double m_mabAlphaD;
  /// Normalization ceiling for delay (D_max = 0.1 s = 100 ms).
  double m_dMax;
  /// Normalization ceiling for ETX (ETX values above this are treated as maximum).
  double m_etxCeil;
  /// Weight of ETX term in the reward formula (α = 0.6).
  double m_rewardAlpha;
  /// Weight of delay term in the reward formula (β = 0.4).
  double m_rewardBeta;
  /// Trend penalty weight in UCB formula (γ = 0.3).
  double m_mabGamma;
  /// Soft overlap-ratio penalty weight P for K=3 path building.
  double m_softPenalty;

  // ---- Path-switch rate tracking ----
  /// Per-destination: last chosen next-hop (for switch detection).
  std::map<Ipv4Address, Ipv4Address> m_lastNextHop;
  /// Total path switches since m_switchWindowStart.
  uint32_t m_switchCount;
  /// Start of the current switch-rate measurement window.
  Time m_switchWindowStart;

  // ---- Random variable ----
  Ptr<UniformRandomVariable> m_uniformRandomVariable;

  // ---- Sockets ----
  Ptr<Socket> m_recvSocket;
  std::map<Ptr<Socket>, Ipv4InterfaceAddress> m_sendSockets;

  // ---- Trace sources ----
  TracedCallback<const olsr::PacketHeader&, const olsr::MessageList&> m_rxPacketTrace;
  TracedCallback<const olsr::PacketHeader&, const olsr::MessageList&> m_txPacketTrace;
  TracedCallback<uint32_t> m_routingTableChanged;

  // ---- Private helpers ----
  void Clear();
  uint32_t GetSize() const
  {
    return m_table.size();
  }

  void RemoveEntry(const Ipv4Address& dest);

  void AddEntry(const Ipv4Address& dest,
                const Ipv4Address& next,
                uint32_t interface,
                uint32_t distance,
                double etxDist);

  void AddEntry(const Ipv4Address& dest,
                const Ipv4Address& next,
                const Ipv4Address& interfaceAddress,
                uint32_t distance,
                double etxDist);

  bool Lookup(const Ipv4Address& dest, RoutingTableEntry& outEntry) const;
  bool FindSendEntry(const RoutingTableEntry& entry, RoutingTableEntry& outEntry) const;
  bool ChooseCandidate(const Ipv4Address& dest, CandidateRoute& out);

  /// Update MAB arm for @p nextHop with normalised reward derived from ETX and delay.
  void UpdateMabModel(const Ipv4Address& nextHop, double instantEtx, double smoothDelay);

  /// Compute trend from the last (up to 5) reward-history samples.
  double ComputeTrend(const std::deque<double>& hist) const;

  /**
   * Build the set of intermediate node addresses on the path from this node
   * to @p dest when using @p firstHop as the first-hop next-hop.
   * Generalisation of BuildPrimaryPathNodes for arbitrary first-hops.
   */
  std::set<Ipv4Address> BuildPathNodes(const Ipv4Address& dest,
                                       const Ipv4Address& firstHop) const;

  // ---- ETX helpers ----
  void UpdateNeighborEtx(const Ipv4Address& neighborIfaceAddr, Time helloInterval);
  double GetLinkEtx(const Ipv4Address& neighborIfaceAddr) const;

public:
  // ---- Ipv4RoutingProtocol interface ----
  Ptr<Ipv4Route> RouteOutput(Ptr<Packet> p,
                             const Ipv4Header& header,
                             Ptr<NetDevice> oif,
                             Socket::SocketErrno& sockerr) override;
  bool RouteInput(Ptr<const Packet> p,
                  const Ipv4Header& header,
                  Ptr<const NetDevice> idev,
                  const UnicastForwardCallback& ucb,
                  const MulticastForwardCallback& mcb,
                  const LocalDeliverCallback& lcb,
                  const ErrorCallback& ecb) override;
  void SetIpv4(Ptr<Ipv4> ipv4) override;
  void PrintRoutingTable(Ptr<OutputStreamWrapper> stream,
                         Time::Unit unit = Time::S) const override;

private:
  void NotifyInterfaceUp(uint32_t interface) override;
  void NotifyInterfaceDown(uint32_t interface) override;
  void NotifyAddAddress(uint32_t interface, Ipv4InterfaceAddress address) override;
  void NotifyRemoveAddress(uint32_t interface, Ipv4InterfaceAddress address) override;

  void SendPacket(Ptr<Packet> packet, const olsr::MessageList& containedMessages);

  inline uint16_t GetPacketSequenceNumber();
  inline uint16_t GetMessageSequenceNumber();

  void RecvOlsr(Ptr<Socket> socket);

  void MprComputation();
  void RoutingTableComputation();

public:
  Ipv4Address GetMainAddress(Ipv4Address iface_addr) const;

private:
  bool UsesNonOlsrOutgoingInterface(const Ipv4RoutingTableEntry& route);

  // ---- Timers ----
  Timer m_helloTimer;
  void HelloTimerExpire();

  Timer m_tcTimer;
  void TcTimerExpire();

  Timer m_midTimer;
  void MidTimerExpire();

  Timer m_hnaTimer;
  void HnaTimerExpire();

  void DupTupleTimerExpire(Ipv4Address address, uint16_t sequenceNumber);

  bool m_linkTupleTimerFirstTime;
  void LinkTupleTimerExpire(Ipv4Address neighborIfaceAddr);

  void Nb2hopTupleTimerExpire(Ipv4Address neighborMainAddr, Ipv4Address twoHopNeighborAddr);
  void MprSelTupleTimerExpire(Ipv4Address mainAddr);
  void TopologyTupleTimerExpire(Ipv4Address destAddr, Ipv4Address lastAddr);
  void IfaceAssocTupleTimerExpire(Ipv4Address ifaceAddr);
  void AssociationTupleTimerExpire(Ipv4Address gatewayAddr,
                                   Ipv4Address networkAddr,
                                   Ipv4Mask netmask);
  void IncrementAnsn();

  // ---- Queued messages ----
  olsr::MessageList m_queuedMessages;
  Timer m_queuedMessagesTimer;

  void ForwardDefault(olsr::MessageHeader olsrMessage,
                      olsr::DuplicateTuple* duplicated,
                      const Ipv4Address& localIface,
                      const Ipv4Address& senderAddress);

  void QueueMessage(const olsr::MessageHeader& message, Time delay);
  void SendQueuedMessages();

  void SendHello();
  void SendTc();
  void SendMid();
  void SendHna();

  void NeighborLoss(const olsr::LinkTuple& tuple);

  void AddDuplicateTuple(const olsr::DuplicateTuple& tuple);
  void RemoveDuplicateTuple(const olsr::DuplicateTuple& tuple);

  void LinkTupleAdded(const olsr::LinkTuple& tuple, olsr::Willingness willingness);
  void RemoveLinkTuple(const olsr::LinkTuple& tuple);
  void LinkTupleUpdated(const olsr::LinkTuple& tuple, olsr::Willingness willingness);

  void AddNeighborTuple(const olsr::NeighborTuple& tuple);
  void RemoveNeighborTuple(const olsr::NeighborTuple& tuple);
  void AddTwoHopNeighborTuple(const olsr::TwoHopNeighborTuple& tuple);
  void RemoveTwoHopNeighborTuple(const olsr::TwoHopNeighborTuple& tuple);
  void AddMprSelectorTuple(const olsr::MprSelectorTuple& tuple);
  void RemoveMprSelectorTuple(const olsr::MprSelectorTuple& tuple);
  void AddTopologyTuple(const olsr::TopologyTuple& tuple);
  void RemoveTopologyTuple(const olsr::TopologyTuple& tuple);
  void AddIfaceAssocTuple(const olsr::IfaceAssocTuple& tuple);
  void RemoveIfaceAssocTuple(const olsr::IfaceAssocTuple& tuple);
  void AddAssociationTuple(const olsr::AssociationTuple& tuple);
  void RemoveAssociationTuple(const olsr::AssociationTuple& tuple);

  void ProcessHello(const olsr::MessageHeader& msg,
                    const Ipv4Address& receiverIface,
                    const Ipv4Address& senderIface);
  void ProcessTc(const olsr::MessageHeader& msg, const Ipv4Address& senderIface);
  void ProcessMid(const olsr::MessageHeader& msg, const Ipv4Address& senderIface);
  void ProcessHna(const olsr::MessageHeader& msg, const Ipv4Address& senderIface);

  void LinkSensing(const olsr::MessageHeader& msg,
                   const olsr::MessageHeader::Hello& hello,
                   const Ipv4Address& receiverIface,
                   const Ipv4Address& senderIface);
  void PopulateNeighborSet(const olsr::MessageHeader& msg,
                           const olsr::MessageHeader::Hello& hello);
  void PopulateTwoHopNeighborSet(const olsr::MessageHeader& msg,
                                  const olsr::MessageHeader::Hello& hello);
  void PopulateMprSelectorSet(const olsr::MessageHeader& msg,
                              const olsr::MessageHeader::Hello& hello);

  bool IsMyOwnAddress(const Ipv4Address& a) const;

  int Degree(const olsr::NeighborTuple& tuple);

  Ipv4Address m_mainAddress;
};

} // namespace etxolsr
} // namespace ns3

#endif /* ETX_OLSR_ROUTING_PROTOCOL_H */
