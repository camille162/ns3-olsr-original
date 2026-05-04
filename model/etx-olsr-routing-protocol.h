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
 #include "ns3/wifi-phy.h"
 
 #include <array>
 #include <deque>
 #include <map>
 #include <set>
 #include <vector>
 #include <limits>
 #include <cmath>
 
 // Fast link-failure constants (AOMDV-inspired defaults).
 #define HELLO_INTERVAL 1.0
 #define ALLOWED_HELLO_LOSS 3
 #define NODE_TRAVERSAL_TIME 0.03
 
 namespace ns3
 {
 namespace etxolsr
 {
 
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
   std::vector<Ipv4Address> nextHops; //!< K candidate next hops from control plane.
 
   RoutingTableEntry()
       : destAddr(),
         nextAddr(),
         interface(0),
         distance(0),
         etxDistance(0.0),
         nextHops()
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
 
 struct LinkNeighbor
 {
   Ipv4Address neighborAddr;
   Time lastHeard;
   bool isLinkUp;
   uint32_t helloLossCount;
 
   LinkNeighbor()
       : neighborAddr(),
         lastHeard(Seconds(0.0)),
         isLinkUp(false),
         helloLossCount(0)
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
   
   Ptr<Ipv4> GetIpv4() const;
 
   void SetMainInterface(uint32_t interface);
   void Dump();
   std::vector<RoutingTableEntry> GetRoutingTableEntries() const;
 
   olsr::MprSet GetMprSet() const;
   const olsr::MprSelectorSet& GetMprSelectors() const;
   const olsr::NeighborSet& GetNeighbors() const;
   const olsr::TwoHopNeighborSet& GetTwoHopNeighbors() const;
   const olsr::TopologySet& GetTopologySet() const;
   const olsr::OlsrState& GetOlsrState() const;
 
   /**
    * Aggregate local bandit / risk signals for an external control plane (per-node).
    * meanSigmaSqrt: mean sqrt(σ²) over MAB arms with at least one observation.
    * meanDelayMs: mean EWMA delay (ms) over those arms.
    * pathSwitchRatePerSec: MAB path switches counted in the last 1 s window.
    * trackedArms: number of arms included in the means.
    */
   void CollectControlPlaneSignals(double& meanSigmaSqrt,
                                   double& meanDelayMs,
                                   double& pathSwitchRatePerSec,
                                   uint32_t& trackedArms) const;
 
   /** Clamp and apply UCB / stability hyperparameters (used by ControlAgent). */
   void SetAdaptiveHyperparameters(double mabC, double switchPenalty);
 
   double GetMabC() const;
   double GetSwitchPenalty() const;
 
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
   std::map<Ipv4Address, LinkNeighbor> m_linkNeighbors;
   double m_etxAlpha;
   double m_initialEtx;
   uint32_t m_maxQueueLen;
   Time m_maxQueueTime;
 
   struct BufferedPacketEntry
   {
     Ptr<Packet> packet;
     Ipv4Header ipHeader;
     UnicastForwardCallback ucb;
     ErrorCallback ecb;
     Time enqueueTime;
   };
   std::deque<BufferedPacketEntry> m_packetQueue;
 
   // ---- Risk-aware routing state ----
   /// Per-next-hop EWMA variance of path-cost prediction error (σ²).
   std::map<Ipv4Address, double> m_sigma;
   /// Previous candidate cost through each next-hop, used to compute prediction error.
   std::map<Ipv4Address, double> m_prevCandCost;
   /// EWMA smoothing factor β for σ² update (0 < β ≤ 1).
   double m_beta;
   /// Risk weight λ: score = V̂ + λ·σ.
   double m_lambda;
 
   // ---- Candidate routing (K=3) ----
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
   std::map<Ipv4Address, std::array<std::vector<Ipv4Address>, 3>> m_candidatePathTable;
 
   // ---- MAB (UCB + trend penalty) state for data-plane arm selection ----
   struct MabArm
   {
     uint64_t totalCount;             //!< N: cumulative selection count.
     double meanReward;               //!< mu: running mean reward.
     double smoothedDelayMs;          //!< EWMA-smoothed data-plane delay (ms).
     bool hasSmoothedDelay;           //!< Whether delay EWMA has been initialized.
     std::deque<double> rewardHistory; //!< Latest rewards for trend (size <= 5).
 
     MabArm()
         : totalCount(0),
           meanReward(0.0),
           smoothedDelayMs(0.0),
           hasSmoothedDelay(false)
     {
     }
   };
 
   /// Per-next-hop MAB arm statistics.
   std::map<Ipv4Address, MabArm> m_mabArms;
   /// Total number of data-plane routing decisions made so far (t).
   uint64_t m_mabTotalDecisions;
   /// UCB exploration constant c.
   double m_mabC;
   /// Trend-penalty coefficient gamma.
   double m_mabGamma;
  /// Switching penalty coefficient to reduce route flapping.
  double m_switchPenalty;
   /// Delay normalization cap D_max in milliseconds.
   double m_delayMaxMs;
   /// Maximum allowed delay used for normalized reward.
   double m_maxAllowedDelayMs;
   /// Delay EWMA smoothing alpha_d.
   double m_delayEwmaAlpha;
   /// ETX normalization base.
   double m_etxBase;
   /// Reward alpha for success term.
   double m_rewardAlpha;
   /// Reward beta for delay term.
   double m_rewardBeta;
   /// Reward weight for SINR risk term.
   double m_rewardGamma;
   /// Reward weight for ETX term (1/ETX).
   double m_rewardDeltaEtx;
   /// EWMA alpha for reward-value update.
   double m_rewardEwmaAlpha;
   /// Soft penalty coefficient P for overlapping backup paths.
   double m_overlapPenalty;
   /// EWMA alpha for SINR smoothing.
   double m_sinrAlpha;
   /// SINR threshold below which risk becomes infinite.
   double m_sinrThresholdDb;
   /// Target linear SINR used for normalized reward.
   double m_targetSinr;
  /// Maximum ETX used by reward normalization clamp.
  double m_rewardEtxMax;
  /// Logistic midpoint for SINR risk normalization (dB).
  double m_sinrRiskMidDb;
  /// Logistic slope for SINR risk normalization (dB).
  double m_sinrRiskSlopeDb;
   /// Maximum acceptable ETX for robust MPR/control selection.
   double m_etxThreshold;
   /// Enable placeholder smart path selector in RouteOutput for A/B experiments.
   bool m_enableSmartPath;
   /// RTT threshold (ms) for Fast label.
   double m_fastRttMs;
   /// Sliding window (s) with zero drops for Stable label.
   double m_stableWindowSec;
   /// Maximum SINR delta (dB) EWMA for Stable label.
   double m_stableSinrDeltaDb;
   /// SINR threshold (dB) below which link is considered Noisy.
   double m_noisySinrDb;
   /// Sliding window (s) for Noisy-drop detection.
   double m_noisyDropWindowSec;
 
   struct LinkState
   {
     double smoothSinrDb;
     double smoothRssiDbm;
     double smoothSinrDeltaDb;
     double prevSinrDb;
     bool hasSinr;
     bool hasPrevSinr;
 
     LinkState()
         : smoothSinrDb(0.0),
           smoothRssiDbm(0.0),
           smoothSinrDeltaDb(0.0),
           prevSinrDb(0.0),
           hasSinr(false),
           hasPrevSinr(false)
     {
     }
   };
 
   std::map<Ipv4Address, LinkState> m_linkState;
   std::map<uint32_t, Ptr<WifiPhy>> m_wifiPhys;
   std::map<Ipv4Address, Time> m_neighRealTimeRtt;
   std::map<Ipv4Address, int64_t> m_lastRttDelta;
   std::map<Ipv4Address, uint32_t> m_positiveDeltaStreak;
   std::map<Ipv4Address, std::deque<Time>> m_macTxDropEvents;
 
   enum NeighborLabel : uint8_t
   {
     LABEL_NONE = 0,
     LABEL_FAST = 1 << 0,
     LABEL_STABLE = 1 << 1,
     LABEL_NOISY = 1 << 2
   };
 
   // ---- Stability observability ----
   std::map<Ipv4Address, Ipv4Address> m_lastChosenNextHop;
  std::map<Ipv4Address, uint32_t> m_pathSwitchCountByDest;
  uint64_t m_totalPathSelections;
  uint64_t m_totalPathSwitches;
  std::map<uint64_t, uint32_t> m_lastChosenPathByFlow;
  std::map<uint64_t, uint32_t> m_pathSwitchCountByFlow;
   std::deque<Time> m_switchEvents;
 
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
   bool ChooseCandidate(const Ipv4Address& dest, CandidateRoute& out, uint32_t& pathIndex);
   void UpdateMabModel(const Ipv4Address& nextHop,
                       bool isSuccess,
                       double etxValue,
                       double rawDelayMs,
                       double sinrDb);
   double ComputeTrendPenalty(const MabArm& arm) const;
   std::set<Ipv4Address> BuildPathNodesForNextHop(const Ipv4Address& dest,
                                                  const Ipv4Address& firstHop) const;
   std::vector<Ipv4Address> BuildPathSequenceForNextHop(const Ipv4Address& dest,
                                                        const Ipv4Address& firstHop) const;
   double ComputeSinrRisk(const Ipv4Address& nextHop) const;
   void ResetArmForNextHop(const Ipv4Address& nextHop);
   void NotifyMonitorSnifferRx(Ptr<const Packet> packet,
                               uint16_t channelFreqMhz,
                               WifiTxVector txVector,
                               MpduInfo aMpdu,
                               SignalNoiseDbm signalNoise,
                               uint16_t staId);
   Ipv4Address ResolveIpv4FromMac(const Address& mac) const;
   void SetupRttTracer(Ptr<NetDevice> device, Ptr<Node> hostNode);
   void TraceRTTUpdate(Ipv4Address neighAddr, Time newRtt);
   Ipv4Address SmartPathSelect(Ipv4Address dest);
   void NotifyMacTxDrop(Ptr<const Packet> packet);
   void ClearNeighborTelemetry(Ipv4Address neighAddr);
   uint8_t GetNeighborLabels(Ipv4Address neighAddr) const;
   void PruneDropEvents(Ipv4Address neighAddr, double windowSec);
 
   // ---- ETX helpers ----
   void UpdateNeighborEtx(const Ipv4Address& neighborIfaceAddr, Time helloInterval);
   double GetLinkEtx(const Ipv4Address& neighborIfaceAddr) const;
   void UpdateNeighborHeard(const Ipv4Address& neighborAddr);
   bool FastLinkFailureDetection(const Ipv4Address& neighborAddr, bool linkLayerFeedback);
   void LinkLayerFeedbackHandler(const Ipv4Address& neighborAddr, bool isSuccess);
   void TriggerRouteRecalculation(const Ipv4Address& neighborAddr);
   bool EnqueueBufferedPacket(const Ptr<const Packet>& packet,
                              const Ipv4Header& header,
                              const UnicastForwardCallback& ucb,
                              const ErrorCallback& ecb);
   void SendBufferedPackets(const Ipv4Address& dst, const Ptr<Ipv4Route>& route);
   void CleanBufferedPacketQueue();
 
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