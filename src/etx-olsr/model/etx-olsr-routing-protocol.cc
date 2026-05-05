/*
 * ETX-OLSR: OLSR routing with ETX-based link metric.
 *
 * RFC 3626 core sources live in this module under etx-olsr-* filenames.
 * Copyright (c) 2004 Francisco J. Ros
 * Copyright (c) 2007 INESC Port
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

 #include "ns3/config.h"
 #define NS_LOG_APPEND_CONTEXT                                    \
   if (GetObject<Node>())                                         \
   {                                                              \
     std::clog << "[node " << GetObject<Node>()->GetId() << "] "; \
   }
 
 #include "etx-olsr-routing-protocol.h"
 #include "etx-tc-metric-trailer.h"
 
 #include "ns3/boolean.h"
 #include "ns3/double.h"
 #include "ns3/enum.h"
 #include "ns3/inet-socket-address.h"
 #include "ns3/ipv4-header.h"
 #include "ns3/ipv4-packet-info-tag.h"
 #include "ns3/ipv4-route.h"
 #include "ns3/ipv4-routing-protocol.h"
 #include "ns3/ipv4-routing-table-entry.h"
 #include "ns3/log.h"
 #include "ns3/names.h"
 #include "ns3/net-device.h"
 #include "ns3/node-list.h"
 #include "ns3/simulator.h"
 #include "ns3/socket-factory.h"
 #include "ns3/tag.h"
 #include "ns3/trace-source-accessor.h"
 #include "ns3/udp-socket-factory.h"
 #include "ns3/uinteger.h"
 #include "ns3/wifi-mac-header.h"
 #include "ns3/wifi-net-device.h"
 
 #include <algorithm>
 #include <cmath>
 #include <iomanip>
 #include <iostream>
 #include <limits>
 #include <map>
 
 #define DELAY(time)                                                                                \
   (((time) < (Simulator::Now())) ? Seconds(0.000001)                                               \
                                  : (time - Simulator::Now() + Seconds(0.000001)))
 
 #define OLSR_REFRESH_INTERVAL m_helloInterval
 #define OLSR_NEIGHB_HOLD_TIME Time(3 * OLSR_REFRESH_INTERVAL)
 #define OLSR_TOP_HOLD_TIME Time(3 * m_tcInterval)
 #define OLSR_DUP_HOLD_TIME Seconds(30)
 #define OLSR_MID_HOLD_TIME Time(3 * m_midInterval)
 #define OLSR_HNA_HOLD_TIME Time(3 * m_hnaInterval)
 
 #define OLSR_MAXJITTER (m_helloInterval.GetSeconds() / 4)
 #define OLSR_MAX_SEQ_NUM 65535
 #define JITTER (Seconds(m_uniformRandomVariable->GetValue(0, OLSR_MAXJITTER)))
 
 #define OLSR_MAX_MSGS 64
 #define OLSR_MAX_HELLOS 12
 #define OLSR_MAX_ADDRS 64
 
 namespace ns3
 {
 
 NS_LOG_COMPONENT_DEFINE("EtxOlsrRoutingProtocol");
 
 namespace etxolsr
 {
 
 namespace
 {
uint64_t
BuildFlowKey(const Ipv4Address& src, const Ipv4Address& dst, uint8_t protocol)
{
  const uint64_t srcPart = static_cast<uint64_t>(src.Get()) << 32;
  const uint64_t dstPart = static_cast<uint64_t>(dst.Get()) << 8;
  const uint64_t protoPart = static_cast<uint64_t>(protocol);
  return srcPart ^ dstPart ^ protoPart;
}

Ptr<Node>
ResolveHostNode(RoutingProtocol* rp, Ptr<NetDevice> device)
{
  Ptr<Node> n = rp->GetObject<Node>();
  if (!n && device)
  {
    n = device->GetNode();
  }
  return n;
}

 class MyRouteTag : public Tag
 {
 public:
   static TypeId GetTypeId()
   {
     static TypeId tid = TypeId("ns3::etxolsr::MyRouteTag")
                             .SetParent<Tag>()
                             .AddConstructor<MyRouteTag>();
     return tid;
   }
 
   TypeId GetInstanceTypeId() const override
   {
     return GetTypeId();
   }
 
   uint32_t GetSerializedSize() const override
   {
     return sizeof(uint32_t) + sizeof(int64_t);
   }
 
   void Serialize(TagBuffer i) const override
   {
     i.WriteU32(m_pathId);
     i.WriteU64(static_cast<uint64_t>(m_timestamp.GetNanoSeconds()));
   }
 
   void Deserialize(TagBuffer i) override
   {
     m_pathId = i.ReadU32();
     const uint64_t ns = i.ReadU64();
     m_timestamp = NanoSeconds(static_cast<int64_t>(ns));
   }
 
   void Print(std::ostream& os) const override
   {
     os << "pathId=" << m_pathId << " ts=" << m_timestamp.GetSeconds();
   }
 
   void SetPathId(uint32_t pathId)
   {
     m_pathId = pathId;
   }
 
   uint32_t GetPathId() const
   {
     return m_pathId;
   }
 
   void SetTimestamp(Time t)
   {
     m_timestamp = t;
   }
 
   Time GetTimestamp() const
   {
     return m_timestamp;
   }
 
 private:
   uint32_t m_pathId{0};
   Time m_timestamp;
 };
 } // namespace
 
 enum class LinkType : uint8_t
 {
   UNSPEC_LINK = 0,
   ASYM_LINK = 1,
   SYM_LINK = 2,
   LOST_LINK = 3,
 };
 
 enum class NeighborType : uint8_t
 {
   NOT_NEIGH = 0,
   SYM_NEIGH = 1,
   MPR_NEIGH = 2,
 };
 
 NS_OBJECT_ENSURE_REGISTERED(RoutingProtocol);
 
 const uint16_t RoutingProtocol::OLSR_PORT_NUMBER = 698;
 
 TypeId
 RoutingProtocol::GetTypeId()
 {
   static TypeId tid =
       TypeId("ns3::etxolsr::RoutingProtocol")
           .SetParent<Ipv4RoutingProtocol>()
           .SetGroupName("EtxOlsr")
           .AddConstructor<RoutingProtocol>()
           .AddAttribute("HelloInterval",
                         "HELLO messages emission interval.",
                         TimeValue(Seconds(0.5)),
                         MakeTimeAccessor(&RoutingProtocol::m_helloInterval),
                         MakeTimeChecker())
           .AddAttribute("TcInterval",
                         "TC messages emission interval.",
                         TimeValue(Seconds(1)),
                         MakeTimeAccessor(&RoutingProtocol::m_tcInterval),
                         MakeTimeChecker())
           .AddAttribute("MidInterval",
                         "MID messages emission interval.",
                         TimeValue(Seconds(5)),
                         MakeTimeAccessor(&RoutingProtocol::m_midInterval),
                         MakeTimeChecker())
           .AddAttribute("HnaInterval",
                         "HNA messages emission interval.",
                         TimeValue(Seconds(5)),
                         MakeTimeAccessor(&RoutingProtocol::m_hnaInterval),
                         MakeTimeChecker())
           .AddAttribute("Willingness",
                         "Willingness of a node to carry and forward traffic for other nodes.",
                         EnumValue(olsr::Willingness::DEFAULT),
                         MakeEnumAccessor<olsr::Willingness>(&RoutingProtocol::m_willingness),
                         MakeEnumChecker(olsr::Willingness::NEVER,
                                         "never",
                                         olsr::Willingness::LOW,
                                         "low",
                                         olsr::Willingness::DEFAULT,
                                         "default",
                                         olsr::Willingness::HIGH,
                                         "high",
                                         olsr::Willingness::ALWAYS,
                                         "always"))
           .AddAttribute("EtxAlpha",
                         "EWMA smoothing factor for ETX estimation.",
                         DoubleValue(0.7),
                         MakeDoubleAccessor(&RoutingProtocol::m_etxAlpha),
                         MakeDoubleChecker<double>(0.0, 1.0))
           .AddAttribute("InitialEtx",
                         "Default ETX value for unknown links.",
                         DoubleValue(3.0),
                         MakeDoubleAccessor(&RoutingProtocol::m_initialEtx),
                         MakeDoubleChecker<double>(1.0))
           .AddAttribute("EnableTcLinkEtx",
                         "If true, append an ETX-OLSR trailer after OLSR messages in the UDP "
                         "payload (RFC 3626 TC body unchanged; trailer ignored by vanilla ns-3 OLSR).",
                         BooleanValue(true),
                         MakeBooleanAccessor(&RoutingProtocol::m_enableTcLinkEtx),
                         MakeBooleanChecker())
           .AddAttribute("RiskBeta",
                         "EWMA smoothing factor β for per-next-hop variance (σ²) update. "
                         "Range [0,1]: 0 freezes variance at its initial value of 0 "
                         "(effectively disables risk-aware selection), 1 uses only the "
                         "most-recent squared error.",
                         DoubleValue(0.3),
                         MakeDoubleAccessor(&RoutingProtocol::m_beta),
                         MakeDoubleChecker<double>(0.0, 1.0))
           .AddAttribute("RiskLambda",
                         "Risk weight λ used in candidate scoring: score = V̂ + λ·σ.",
                         DoubleValue(1.0),
                         MakeDoubleAccessor(&RoutingProtocol::m_lambda),
                         MakeDoubleChecker<double>(0.0))
           .AddAttribute("MabC",
                         "UCB exploration constant c. score = mu + c*sqrt(ln(t)/N) - gamma*trend.",
                         DoubleValue(1.0),
                         MakeDoubleAccessor(&RoutingProtocol::m_mabC),
                         MakeDoubleChecker<double>(0.0))
           .AddAttribute("MabGamma",
                         "Trend penalty coefficient gamma in UCB score.",
                         DoubleValue(0.3),
                         MakeDoubleAccessor(&RoutingProtocol::m_mabGamma),
                         MakeDoubleChecker<double>(0.0))
          .AddAttribute("SwitchPenalty",
                        "Penalty applied when selected next-hop differs from previous one "
                        "for the same destination.",
                        DoubleValue(0.1),
                        MakeDoubleAccessor(&RoutingProtocol::m_switchPenalty),
                        MakeDoubleChecker<double>(0.0))
           .AddAttribute("DelayMaxMs",
                         "Delay normalization cap D_max in milliseconds.",
                         DoubleValue(100.0),
                         MakeDoubleAccessor(&RoutingProtocol::m_delayMaxMs),
                         MakeDoubleChecker<double>(1.0))
           .AddAttribute("MaxAllowedDelayMs",
                         "Maximum allowed delay for normalized reward.",
                         DoubleValue(100.0),
                         MakeDoubleAccessor(&RoutingProtocol::m_maxAllowedDelayMs),
                         MakeDoubleChecker<double>(1.0))
           .AddAttribute("DelayEwmaAlpha",
                         "EWMA alpha_d for delay smoothing: D_s = alpha_d*D_old + (1-alpha_d)*D_new.",
                         DoubleValue(0.8),
                         MakeDoubleAccessor(&RoutingProtocol::m_delayEwmaAlpha),
                         MakeDoubleChecker<double>(0.0, 1.0))
           .AddAttribute("EtxBase",
                         "ETX normalization base.",
                         DoubleValue(1.0),
                         MakeDoubleAccessor(&RoutingProtocol::m_etxBase),
                         MakeDoubleChecker<double>(0.1))
           .AddAttribute("RewardAlpha",
                         "Reward alpha for success term.",
                         DoubleValue(0.6),
                         MakeDoubleAccessor(&RoutingProtocol::m_rewardAlpha),
                         MakeDoubleChecker<double>(0.0, 1.0))
           .AddAttribute("RewardBeta",
                         "Reward beta for delay term.",
                         DoubleValue(0.4),
                         MakeDoubleAccessor(&RoutingProtocol::m_rewardBeta),
                         MakeDoubleChecker<double>(0.0, 1.0))
           .AddAttribute("RewardGamma",
                         "Reward gamma for SINR risk term.",
                         DoubleValue(0.2),
                         MakeDoubleAccessor(&RoutingProtocol::m_rewardGamma),
                         MakeDoubleChecker<double>(0.0, 1.0))
           .AddAttribute("RewardDeltaEtx",
                         "Reward delta for ETX term: 1/max(1,ETX).",
                         DoubleValue(0.0),
                         MakeDoubleAccessor(&RoutingProtocol::m_rewardDeltaEtx),
                         MakeDoubleChecker<double>(0.0, 1.0))
           .AddAttribute("RewardEwmaAlpha",
                         "EWMA alpha for reward-value update.",
                         DoubleValue(0.2),
                         MakeDoubleAccessor(&RoutingProtocol::m_rewardEwmaAlpha),
                         MakeDoubleChecker<double>(0.0, 1.0))
           .AddAttribute("OverlapPenalty",
                         "Soft overlap penalty P for backup path selection.",
                         DoubleValue(80.0),
                         MakeDoubleAccessor(&RoutingProtocol::m_overlapPenalty),
                         MakeDoubleChecker<double>(0.0))
           .AddAttribute("SinrAlpha",
                         "EWMA alpha for smoothing SINR samples.",
                         DoubleValue(0.8),
                         MakeDoubleAccessor(&RoutingProtocol::m_sinrAlpha),
                         MakeDoubleChecker<double>(0.0, 1.0))
           .AddAttribute("SinrThresholdDb",
                         "SINR threshold in dB below which risk becomes infinite.",
                         DoubleValue(5.0),
                         MakeDoubleAccessor(&RoutingProtocol::m_sinrThresholdDb),
                         MakeDoubleChecker<double>())
           .AddAttribute("TargetSinr",
                         "Target linear SINR used for normalized reward.",
                         DoubleValue(10.0),
                         MakeDoubleAccessor(&RoutingProtocol::m_targetSinr),
                         MakeDoubleChecker<double>(0.1))
          .AddAttribute("RewardEtxMax",
                        "Upper ETX clamp used in reward normalization.",
                        DoubleValue(5.0),
                        MakeDoubleAccessor(&RoutingProtocol::m_rewardEtxMax),
                        MakeDoubleChecker<double>(1.1))
          .AddAttribute("SinrRiskMidDb",
                        "Logistic SINR-risk midpoint (dB).",
                        DoubleValue(5.0),
                        MakeDoubleAccessor(&RoutingProtocol::m_sinrRiskMidDb),
                        MakeDoubleChecker<double>())
          .AddAttribute("SinrRiskSlopeDb",
                        "Logistic SINR-risk slope (dB). Smaller values mean steeper transition.",
                        DoubleValue(2.0),
                        MakeDoubleAccessor(&RoutingProtocol::m_sinrRiskSlopeDb),
                        MakeDoubleChecker<double>(0.1))
           .AddAttribute("EtxThreshold",
                         "Maximum ETX accepted for robust MPR/control selection.",
                         DoubleValue(1.5),
                         MakeDoubleAccessor(&RoutingProtocol::m_etxThreshold),
                         MakeDoubleChecker<double>(1.0))
           .AddAttribute("EnableSmartPath",
                         "Enable placeholder smart-path selection in RouteOutput.",
                         BooleanValue(false),
                         MakeBooleanAccessor(&RoutingProtocol::m_enableSmartPath),
                         MakeBooleanChecker())
           .AddAttribute("FastRttMs",
                         "RTT threshold in milliseconds for Fast neighbor label.",
                         DoubleValue(10.0),
                         MakeDoubleAccessor(&RoutingProtocol::m_fastRttMs),
                         MakeDoubleChecker<double>(0.0))
           .AddAttribute("StableWindowSec",
                         "Sliding window in seconds with zero drops for Stable label.",
                         DoubleValue(10.0),
                         MakeDoubleAccessor(&RoutingProtocol::m_stableWindowSec),
                         MakeDoubleChecker<double>(0.0))
           .AddAttribute("StableSinrDeltaDb",
                         "Maximum EWMA SINR delta (dB) for Stable label.",
                         DoubleValue(2.0),
                         MakeDoubleAccessor(&RoutingProtocol::m_stableSinrDeltaDb),
                         MakeDoubleChecker<double>(0.0))
           .AddAttribute("NoisySinrDb",
                         "SINR threshold (dB) below which link is tagged Noisy.",
                         DoubleValue(5.0),
                         MakeDoubleAccessor(&RoutingProtocol::m_noisySinrDb),
                         MakeDoubleChecker<double>())
           .AddAttribute("NoisyDropWindowSec",
                         "Sliding window in seconds to tag Noisy based on MacTxDrop events.",
                         DoubleValue(2.0),
                         MakeDoubleAccessor(&RoutingProtocol::m_noisyDropWindowSec),
                         MakeDoubleChecker<double>(0.0))
           .AddAttribute("MaxQueueLen",
                         "Maximum number of packets in ETX-OLSR micro buffer queue.",
                         UintegerValue(32),
                         MakeUintegerAccessor(&RoutingProtocol::m_maxQueueLen),
                         MakeUintegerChecker<uint32_t>(1))
           .AddAttribute("MaxQueueTime",
                         "Maximum buffering time for a queued packet before drop.",
                         TimeValue(Seconds(1.0)),
                         MakeTimeAccessor(&RoutingProtocol::m_maxQueueTime),
                         MakeTimeChecker())
           .AddTraceSource("Rx",
                           "Receive ETX-OLSR packet.",
                           MakeTraceSourceAccessor(&RoutingProtocol::m_rxPacketTrace),
                           "ns3::etxolsr::RoutingProtocol::PacketTxRxTracedCallback")
           .AddTraceSource("Tx",
                           "Send ETX-OLSR packet.",
                           MakeTraceSourceAccessor(&RoutingProtocol::m_txPacketTrace),
                           "ns3::etxolsr::RoutingProtocol::PacketTxRxTracedCallback")
           .AddTraceSource("RoutingTableChanged",
                           "The ETX-OLSR routing table has changed.",
                           MakeTraceSourceAccessor(&RoutingProtocol::m_routingTableChanged),
                           "ns3::etxolsr::RoutingProtocol::TableChangeTracedCallback");
   return tid;
 }
 
 RoutingProtocol::RoutingProtocol()
     : m_routingTableAssociation(nullptr),
       m_ipv4(nullptr),
       m_helloTimer(Timer::CANCEL_ON_DESTROY),
       m_tcTimer(Timer::CANCEL_ON_DESTROY),
       m_midTimer(Timer::CANCEL_ON_DESTROY),
       m_hnaTimer(Timer::CANCEL_ON_DESTROY),
       m_queuedMessagesTimer(Timer::CANCEL_ON_DESTROY),
       m_etxAlpha(0.7),
       m_initialEtx(3.0),
       m_enableTcLinkEtx(true),
       m_beta(0.3),
       m_lambda(1.0),
       m_mabTotalDecisions(0),
      m_totalPathSelections(0),
      m_totalPathSwitches(0),
       m_mabC(1.0),
       m_mabGamma(0.3),
      m_switchPenalty(0.1),
       m_delayMaxMs(100.0),
       m_maxAllowedDelayMs(100.0),
       m_delayEwmaAlpha(0.8),
       m_etxBase(1.0),
       m_rewardAlpha(0.6),
       m_rewardBeta(0.4),
       m_rewardGamma(0.2),
       m_rewardDeltaEtx(0.0),
       m_rewardEwmaAlpha(0.2),
       m_sinrAlpha(0.8),
       m_sinrThresholdDb(5.0),
       m_targetSinr(10.0),
      m_rewardEtxMax(5.0),
      m_sinrRiskMidDb(5.0),
      m_sinrRiskSlopeDb(2.0),
       m_etxThreshold(1.5),
       m_enableSmartPath(false),
       m_fastRttMs(10.0),
       m_stableWindowSec(10.0),
       m_stableSinrDeltaDb(2.0),
       m_noisySinrDb(5.0),
       m_noisyDropWindowSec(2.0),
       m_overlapPenalty(80.0),
       m_maxQueueLen(32),
       m_maxQueueTime(Seconds(1.0))
 {
   m_uniformRandomVariable = CreateObject<UniformRandomVariable>();
   m_hnaRoutingTable = Create<Ipv4StaticRouting>();
 }
 
 RoutingProtocol::~RoutingProtocol()
 {
 }
 
 void
 RoutingProtocol::SetIpv4(Ptr<Ipv4> ipv4)
 {
   NS_ASSERT(ipv4);
   NS_ASSERT(!m_ipv4);
   NS_LOG_DEBUG("Created etxolsr::RoutingProtocol");
   m_helloTimer.SetFunction(&RoutingProtocol::HelloTimerExpire, this);
   m_tcTimer.SetFunction(&RoutingProtocol::TcTimerExpire, this);
   m_midTimer.SetFunction(&RoutingProtocol::MidTimerExpire, this);
   m_hnaTimer.SetFunction(&RoutingProtocol::HnaTimerExpire, this);
   m_queuedMessagesTimer.SetFunction(&RoutingProtocol::SendQueuedMessages, this);
 
   m_packetSequenceNumber = OLSR_MAX_SEQ_NUM;
   m_messageSequenceNumber = OLSR_MAX_SEQ_NUM;
   m_ansn = OLSR_MAX_SEQ_NUM;
 
   m_linkTupleTimerFirstTime = true;
 
   m_ipv4 = ipv4;
   m_hnaRoutingTable->SetIpv4(ipv4);
 }
 
 Ptr<Ipv4>
 RoutingProtocol::GetIpv4() const
 {
   return m_ipv4;
 }
 
 void
 RoutingProtocol::DoDispose()
 {
  if (m_totalPathSelections > 0)
  {
    NS_LOG_UNCOND("PATH_STATS totalSelections=" << m_totalPathSelections
                                                << " totalSwitches=" << m_totalPathSwitches);
    for (const auto& kv : m_pathSwitchCountByDest)
    {
      NS_LOG_UNCOND("PATH_STATS_DEST dest=" << kv.first << " switches=" << kv.second);
    }
    for (const auto& kv : m_pathSwitchCountByFlow)
    {
      NS_LOG_UNCOND("PATH_STATS_FLOW flowId=" << kv.first << " switches=" << kv.second);
    }
  }

  // Defensive: cancel timers before releasing Ipv4/socket state to avoid callbacks
  // firing after DoDispose() has started tearing down members.
  m_helloTimer.Cancel();
  m_tcTimer.Cancel();
  m_midTimer.Cancel();
  m_hnaTimer.Cancel();
  m_queuedMessagesTimer.Cancel();

   m_ipv4 = nullptr;
   m_hnaRoutingTable = nullptr;
   m_routingTableAssociation = nullptr;
 
   if (m_recvSocket)
   {
     m_recvSocket->Close();
     m_recvSocket = nullptr;
   }
 
   for (auto iter = m_sendSockets.begin(); iter != m_sendSockets.end(); iter++)
   {
     iter->first->Close();
   }
   m_sendSockets.clear();
   m_table.clear();
   m_candidateTable.clear();
   m_etxMap.clear();
   m_sigma.clear();
   m_prevCandCost.clear();
   m_mabArms.clear();
   m_neighRealTimeRtt.clear();
   m_lastRttDelta.clear();
   m_positiveDeltaStreak.clear();
   m_macTxDropEvents.clear();
  m_pathSwitchCountByDest.clear();
  m_lastChosenPathByFlow.clear();
  m_pathSwitchCountByFlow.clear();
 
   Ipv4RoutingProtocol::DoDispose();
 }
 
 void
 RoutingProtocol::PrintRoutingTable(Ptr<OutputStreamWrapper> stream, Time::Unit unit) const
 {
   std::ostream* os = stream->GetStream();
   std::ios oldState(nullptr);
   oldState.copyfmt(*os);
 
   *os << std::resetiosflags(std::ios::adjustfield) << std::setiosflags(std::ios::left);
 
   *os << "Node: " << m_ipv4->GetObject<Node>()->GetId() << ", Time: " << Now().As(unit)
       << ", Local time: " << m_ipv4->GetObject<Node>()->GetLocalTime().As(unit)
       << ", ETX-OLSR Routing table" << std::endl;
 
   *os << std::setw(16) << "Destination";
   *os << std::setw(16) << "NextHop";
   *os << std::setw(16) << "Interface";
   *os << std::setw(8) << "Hops";
   *os << "ETXDist" << std::endl;
 
   for (auto iter = m_table.begin(); iter != m_table.end(); iter++)
   {
     std::ostringstream dest;
     std::ostringstream nextHop;
     dest << iter->first;
     nextHop << iter->second.nextAddr;
     *os << std::setw(16) << dest.str();
     *os << std::setw(16) << nextHop.str();
     *os << std::setw(16);
     if (!Names::FindName(m_ipv4->GetNetDevice(iter->second.interface)).empty())
     {
       *os << Names::FindName(m_ipv4->GetNetDevice(iter->second.interface));
     }
     else
     {
       *os << iter->second.interface;
     }
     *os << std::setw(8) << iter->second.distance;
     *os << std::fixed << std::setprecision(3) << iter->second.etxDistance << std::endl;
   }
   *os << std::endl;
 
   if (m_hnaRoutingTable->GetNRoutes() > 0)
   {
     *os << "HNA Routing Table:" << std::endl;
     m_hnaRoutingTable->PrintRoutingTable(stream, unit);
   }
   else
   {
     *os << "HNA Routing Table: empty" << std::endl << std::endl;
   }
   (*os).copyfmt(oldState);
 }
 
 void
 RoutingProtocol::DoInitialize()
 {
   if (m_mainAddress == Ipv4Address())
   {
     Ipv4Address loopback("127.0.0.1");
     for (uint32_t i = 0; i < m_ipv4->GetNInterfaces(); i++)
     {
       Ipv4Address addr = m_ipv4->GetAddress(i, 0).GetLocal();
       if (addr != loopback)
       {
         m_mainAddress = addr;
         break;
       }
     }
     NS_ASSERT(m_mainAddress != Ipv4Address());
   }
 
   NS_LOG_DEBUG("Starting ETX-OLSR on node " << m_mainAddress);
 
   Ipv4Address loopback("127.0.0.1");
   bool canRunOlsr = false;
 
   for (uint32_t i = 0; i < m_ipv4->GetNInterfaces(); i++)
   {
     Ipv4Address addr = m_ipv4->GetAddress(i, 0).GetLocal();
     if (addr == loopback)
     {
       continue;
     }
 
     if (addr != m_mainAddress)
     {
       olsr::IfaceAssocTuple tuple;
       tuple.ifaceAddr = addr;
       tuple.mainAddr = m_mainAddress;
       AddIfaceAssocTuple(tuple);
       NS_ASSERT(GetMainAddress(addr) == m_mainAddress);
     }
 
     if (m_interfaceExclusions.find(i) != m_interfaceExclusions.end())
     {
       continue;
     }
 
     if (!m_recvSocket)
     {
       m_recvSocket = Socket::CreateSocket(GetObject<Node>(), UdpSocketFactory::GetTypeId());
       m_recvSocket->SetAllowBroadcast(true);
       InetSocketAddress inetAddr(Ipv4Address::GetAny(), OLSR_PORT_NUMBER);
       m_recvSocket->SetRecvCallback(MakeCallback(&RoutingProtocol::RecvOlsr, this));
       if (m_recvSocket->Bind(inetAddr))
       {
         NS_FATAL_ERROR("Failed to bind() ETX-OLSR socket");
       }
       m_recvSocket->SetRecvPktInfo(true);
       m_recvSocket->ShutdownSend();
     }
 
     Ptr<Socket> socket = Socket::CreateSocket(GetObject<Node>(), UdpSocketFactory::GetTypeId());
     socket->SetAllowBroadcast(true);
     socket->SetIpTtl(1);
     InetSocketAddress inetAddr(m_ipv4->GetAddress(i, 0).GetLocal(), OLSR_PORT_NUMBER);
     socket->SetRecvCallback(MakeCallback(&RoutingProtocol::RecvOlsr, this));
     socket->BindToNetDevice(m_ipv4->GetNetDevice(i));
     if (socket->Bind(inetAddr))
     {
       NS_FATAL_ERROR("Failed to bind() ETX-OLSR socket");
     }
     socket->SetRecvPktInfo(true);
     m_sendSockets[socket] = m_ipv4->GetAddress(i, 0);
 
     canRunOlsr = true;
   }
 
   if (canRunOlsr)
   {
     HelloTimerExpire();
     TcTimerExpire();
     MidTimerExpire();
     HnaTimerExpire();
     NS_LOG_DEBUG("ETX-OLSR on node " << m_mainAddress << " started");
   }
 }
 
 void
 RoutingProtocol::SetMainInterface(uint32_t interface)
 {
   m_mainAddress = m_ipv4->GetAddress(interface, 0).GetLocal();
 }
 
 void
 RoutingProtocol::SetInterfaceExclusions(std::set<uint32_t> exceptions)
 {
   m_interfaceExclusions = exceptions;
 }
 
 void
 RoutingProtocol::UpdateNeighborEtx(const Ipv4Address& neighborIfaceAddr, Time helloInterval)
 {
   Time now = Simulator::Now();
   auto it = m_etxMap.find(neighborIfaceAddr);
 
   if (it == m_etxMap.end())
   {
     EtxInfo info;
     info.prr = (1.0 - m_etxAlpha);
     info.lastHelloTime = now;
     info.helloInterval = helloInterval.IsZero() ? Seconds(2.0) : helloInterval;
     m_etxMap[neighborIfaceAddr] = info;
     NS_LOG_DEBUG("ETX new neighbor " << neighborIfaceAddr << " prr=" << info.prr);
     return;
   }
 
   EtxInfo& info = it->second;
 
   Time hi = helloInterval.IsZero() ? info.helloInterval : helloInterval;
   if (hi.IsZero())
   {
     hi = Seconds(2.0);
   }
   info.helloInterval = hi;
 
   double elapsed = (now - info.lastHelloTime).GetSeconds();
   double hiSecs = hi.GetSeconds();
   int missed = 0;
   if (hiSecs > 0.0)
   {
     missed = static_cast<int>(elapsed / hiSecs) - 1;
     if (missed < 0)
     {
       missed = 0;
     }
   }
 
   for (int i = 0; i < missed; i++)
   {
     info.prr = m_etxAlpha * info.prr;
   }
 
   info.prr = m_etxAlpha * info.prr + (1.0 - m_etxAlpha) * 1.0;
 
   if (info.prr > 1.0)
   {
     info.prr = 1.0;
   }
   if (info.prr < 0.0)
   {
     info.prr = 0.0;
   }
 
   info.lastHelloTime = now;
 
   NS_LOG_DEBUG("ETX updated neighbor " << neighborIfaceAddr << " prr=" << info.prr
                                        << " etx=" << (1.0 / std::max(info.prr, 0.01))
                                        << " missed=" << missed);
 }
 
 double
 RoutingProtocol::GetLinkEtx(const Ipv4Address& neighborIfaceAddr) const
 {
   auto it = m_etxMap.find(neighborIfaceAddr);
   if (it == m_etxMap.end())
   {
     return m_initialEtx;
   }
   double prr = it->second.prr;
   if (prr < 0.01)
   {
     prr = 0.01;
   }
   if (prr > 1.0)
   {
     prr = 1.0;
   }
   return 1.0 / prr;
 }
 
 void
 RoutingProtocol::PurgeExpiredAdvertisedEdgeEtx()
 {
   const Time now = Simulator::Now();
   for (auto it = m_linkEtxByTcAdvert.begin(); it != m_linkEtxByTcAdvert.end();)
   {
     if (it->second.expire <= now)
     {
       it = m_linkEtxByTcAdvert.erase(it);
     }
     else
     {
       ++it;
     }
   }
 }
 
 double
 RoutingProtocol::GetAdvertisedEdgeEtx(Ipv4Address lastAddr, Ipv4Address destAddr) const
 {
   const auto it = m_linkEtxByTcAdvert.find({lastAddr, destAddr});
   if (it != m_linkEtxByTcAdvert.end() && it->second.expire > Simulator::Now())
   {
     return std::max(1.0, it->second.etx);
   }
   return m_initialEtx;
 }
 
 std::optional<TcEtxTrailerBlock>
 RoutingProtocol::ComputeTcTrailerForQueuedMessage(const olsr::MessageHeader& message) const
 {
   if (!m_enableTcLinkEtx || message.GetMessageType() != olsr::MessageHeader::TC_MESSAGE)
   {
     return std::nullopt;
   }
   const olsr::MessageHeader::Tc& tc = message.GetTc();
   const Ipv4Address originator = message.GetOriginatorAddress();
   const uint16_t msgSeq = message.GetMessageSequenceNumber();
   TcEtxTrailerBlock block;
   block.tcOriginator = originator;
   block.messageSequenceNumber = msgSeq;
   block.ansn = tc.ansn;
   if (originator == m_mainAddress)
   {
     const Time now = Simulator::Now();
     for (const auto& nbMain : tc.neighborAddresses)
     {
       double bestEtx = std::numeric_limits<double>::infinity();
       for (const auto& link : m_state.GetLinks())
       {
         if (GetMainAddress(link.neighborIfaceAddr) == nbMain && link.symTime > now)
         {
           bestEtx = std::min(bestEtx, GetLinkEtx(link.neighborIfaceAddr));
         }
       }
       if (!std::isfinite(bestEtx))
       {
         bestEtx = m_initialEtx;
       }
       block.neighborEtx.emplace_back(nbMain, bestEtx);
     }
     return block;
   }
   const auto key = std::make_pair(originator, msgSeq);
   auto it = m_retransmitTcEtx.find(key);
   if (it != m_retransmitTcEtx.end() && it->second.ansn == tc.ansn)
   {
     return it->second;
   }
   return std::nullopt;
 }
 
 void
 RoutingProtocol::ApplyTcEtxTrailerForMessage(const olsr::MessageHeader& msg)
 {
   if (msg.GetMessageType() != olsr::MessageHeader::TC_MESSAGE)
   {
     return;
   }
   const olsr::MessageHeader::Tc& tc = msg.GetTc();
   const Time exp = Simulator::Now() + msg.GetVTime();
   for (const auto& blk : m_rxTcEtxTrailerThisPkt)
   {
     if (blk.tcOriginator != msg.GetOriginatorAddress() ||
         blk.messageSequenceNumber != msg.GetMessageSequenceNumber() || blk.ansn != tc.ansn)
     {
       continue;
     }
     for (const auto& pe : blk.neighborEtx)
     {
       m_linkEtxByTcAdvert[{blk.tcOriginator, pe.first}] =
           AdvertisedEdgeEtx{std::max(1.0, pe.second), exp};
     }
     return;
   }
 }
 
 void
 RoutingProtocol::UpdateNeighborHeard(const Ipv4Address& neighborAddr)
 {
   LinkNeighbor& info = m_linkNeighbors[neighborAddr];
   info.neighborAddr = neighborAddr;
   info.lastHeard = Simulator::Now();
   info.isLinkUp = true;
   info.helloLossCount = 0;
 }
 
 bool
 RoutingProtocol::FastLinkFailureDetection(const Ipv4Address& neighborAddr, bool linkLayerFeedback)
 {
   if (!linkLayerFeedback)
   {
     auto it = m_linkNeighbors.find(neighborAddr);
     if (it != m_linkNeighbors.end())
     {
       it->second.isLinkUp = false;
       it->second.helloLossCount = ALLOWED_HELLO_LOSS + 1;
     }
     NS_LOG_WARN("FastLinkFailure: link-layer failure for neighbor " << neighborAddr);
     return false;
   }
 
   auto it = m_linkNeighbors.find(neighborAddr);
   if (it == m_linkNeighbors.end())
   {
     return false;
   }
 
   const Time linkTimeout = Seconds(ALLOWED_HELLO_LOSS * HELLO_INTERVAL);
   const Time now = Simulator::Now();
   if (now - it->second.lastHeard > linkTimeout)
   {
     it->second.isLinkUp = false;
     it->second.helloLossCount++;
     NS_LOG_WARN("FastLinkFailure: neighbor timeout " << neighborAddr
                                                      << " helloLossCount="
                                                      << it->second.helloLossCount);
     return false;
   }
 
   it->second.isLinkUp = true;
   it->second.helloLossCount = 0;
   return true;
 }
 
 void
 RoutingProtocol::LinkLayerFeedbackHandler(const Ipv4Address& neighborAddr, bool isSuccess)
 {
   if (isSuccess)
   {
     UpdateNeighborHeard(neighborAddr);
   }
 
   if (!FastLinkFailureDetection(neighborAddr, isSuccess))
   {
     TriggerRouteRecalculation(neighborAddr);
   }
 }
 
 void
 RoutingProtocol::TriggerRouteRecalculation(const Ipv4Address& neighborAddr)
 {
   NS_LOG_DEBUG("TriggerRouteRecalculation due to neighbor " << neighborAddr);
   m_state.EraseTwoHopNeighborTuples(GetMainAddress(neighborAddr));
   m_state.EraseMprSelectorTuples(GetMainAddress(neighborAddr));
   MprComputation();
   RoutingTableComputation();
 }
 
 void
 RoutingProtocol::CleanBufferedPacketQueue()
 {
   const Time now = Simulator::Now();
   auto it = m_packetQueue.begin();
   while (it != m_packetQueue.end())
   {
     if ((now - it->enqueueTime) > m_maxQueueTime)
     {
       if (!it->ecb.IsNull())
       {
         it->ecb(it->packet, it->ipHeader, Socket::ERROR_NOROUTETOHOST);
       }
       it = m_packetQueue.erase(it);
       continue;
     }
     ++it;
   }
 }
 
 bool
 RoutingProtocol::EnqueueBufferedPacket(const Ptr<const Packet>& packet,
                                        const Ipv4Header& header,
                                        const UnicastForwardCallback& ucb,
                                        const ErrorCallback& ecb)
 {
   if (!packet)
   {
     return false;
   }
 
   CleanBufferedPacketQueue();
   if (m_packetQueue.size() >= m_maxQueueLen)
   {
     BufferedPacketEntry drop = m_packetQueue.front();
     if (!drop.ecb.IsNull())
     {
       drop.ecb(drop.packet, drop.ipHeader, Socket::ERROR_NOROUTETOHOST);
     }
     m_packetQueue.pop_front();
   }
 
   BufferedPacketEntry entry;
   entry.packet = packet->Copy();
   entry.ipHeader = header;
   entry.ucb = ucb;
   entry.ecb = ecb;
   entry.enqueueTime = Simulator::Now();
   m_packetQueue.push_back(entry);
   return true;
 }
 
 void
 RoutingProtocol::SendBufferedPackets(const Ipv4Address& dst, const Ptr<Ipv4Route>& route)
 {
   CleanBufferedPacketQueue();
   if (!route)
   {
     return;
   }
 
   auto it = m_packetQueue.begin();
   while (it != m_packetQueue.end())
   {
     if (it->ipHeader.GetDestination() != dst)
     {
       ++it;
       continue;
     }
 
     if (!it->ucb.IsNull() && it->packet)
     {
       it->ucb(route, it->packet->Copy(), it->ipHeader);
     }
     it = m_packetQueue.erase(it);
   }
 }
 
 void
 RoutingProtocol::RecvOlsr(Ptr<Socket> socket)
 {
  // Defensive: sockets/timers may still trigger callbacks during teardown.
  if (!m_ipv4)
  {
    return;
  }

   Ptr<Packet> receivedPacket;
   Address sourceAddress;
   receivedPacket = socket->RecvFrom(sourceAddress);
 
   Ipv4PacketInfoTag interfaceInfo;
   if (!receivedPacket->RemovePacketTag(interfaceInfo))
   {
     NS_ABORT_MSG("No incoming interface on ETX-OLSR message, aborting.");
   }
   uint32_t incomingIf = interfaceInfo.GetRecvIf();
   Ptr<Node> node = this->GetObject<Node>();
  if (!node)
  {
    return;
  }
  if (incomingIf >= node->GetNDevices())
  {
    return;
  }
  Ptr<NetDevice> dev = node->GetDevice(incomingIf);
  if (!dev)
  {
    return;
  }
   uint32_t recvInterfaceIndex = m_ipv4->GetInterfaceForDevice(dev);
 
   if (m_interfaceExclusions.find(recvInterfaceIndex) != m_interfaceExclusions.end())
   {
     return;
   }
 
   InetSocketAddress inetSourceAddr = InetSocketAddress::ConvertFrom(sourceAddress);
   Ipv4Address senderIfaceAddr = inetSourceAddr.GetIpv4();
 
   int32_t interfaceForAddress = m_ipv4->GetInterfaceForAddress(senderIfaceAddr);
   if (interfaceForAddress != -1)
   {
     NS_LOG_LOGIC("Ignoring a packet sent by myself.");
     return;
   }
 
   Ipv4Address receiverIfaceAddr = m_ipv4->GetAddress(recvInterfaceIndex, 0).GetLocal();
   NS_ASSERT(receiverIfaceAddr != Ipv4Address());
   NS_LOG_DEBUG("ETX-OLSR node " << m_mainAddress << " received a packet from " << senderIfaceAddr
                                 << " to " << receiverIfaceAddr);
 
   NS_ASSERT(inetSourceAddr.GetPort() == OLSR_PORT_NUMBER);
 
   Ptr<Packet> packet = receivedPacket;
 
   olsr::PacketHeader olsrPacketHeader;
   packet->RemoveHeader(olsrPacketHeader);
   NS_ASSERT(olsrPacketHeader.GetPacketLength() >= olsrPacketHeader.GetSerializedSize());
   uint32_t sizeLeft = olsrPacketHeader.GetPacketLength() - olsrPacketHeader.GetSerializedSize();
 
   olsr::MessageList messages;
 
   while (sizeLeft)
   {
     olsr::MessageHeader messageHeader;
     if (packet->RemoveHeader(messageHeader) == 0)
     {
       NS_ASSERT(false);
     }
     sizeLeft -= messageHeader.GetSerializedSize();
 
     NS_LOG_DEBUG("ETX-OLSR Msg received with type "
                  << std::dec << int(messageHeader.GetMessageType())
                  << " TTL=" << int(messageHeader.GetTimeToLive())
                  << " origAddr=" << messageHeader.GetOriginatorAddress());
     messages.push_back(messageHeader);
   }
 
   std::vector<TcEtxTrailerBlock> rxTrailer;
   TryConsumeTcEtxTrailer(packet, rxTrailer);
   m_rxTcEtxTrailerThisPkt = std::move(rxTrailer);
   for (const auto& blk : m_rxTcEtxTrailerThisPkt)
   {
     m_retransmitTcEtx[{blk.tcOriginator, blk.messageSequenceNumber}] = blk;
   }
 
   m_rxPacketTrace(olsrPacketHeader, messages);
 
   for (auto messageIter = messages.begin(); messageIter != messages.end(); messageIter++)
   {
     const olsr::MessageHeader& messageHeader = *messageIter;
 
     if (messageHeader.GetTimeToLive() == 0 ||
         messageHeader.GetOriginatorAddress() == m_mainAddress)
     {
       continue;
     }
 
     bool do_forwarding = true;
     olsr::DuplicateTuple* duplicated =
         m_state.FindDuplicateTuple(messageHeader.GetOriginatorAddress(),
                                    messageHeader.GetMessageSequenceNumber());
 
     if (duplicated == nullptr)
     {
       switch (messageHeader.GetMessageType())
       {
       case olsr::MessageHeader::HELLO_MESSAGE:
         NS_LOG_DEBUG(Simulator::Now().As(Time::S)
                      << " ETX-OLSR node " << m_mainAddress
                      << " received HELLO message of size "
                      << messageHeader.GetSerializedSize());
         UpdateNeighborHeard(senderIfaceAddr);
         UpdateNeighborEtx(senderIfaceAddr, messageHeader.GetHello().GetHTime());
         ProcessHello(messageHeader, receiverIfaceAddr, senderIfaceAddr);
         break;
 
       case olsr::MessageHeader::TC_MESSAGE:
         NS_LOG_DEBUG(Simulator::Now().As(Time::S)
                      << " ETX-OLSR node " << m_mainAddress
                      << " received TC message of size "
                      << messageHeader.GetSerializedSize());
         ProcessTc(messageHeader, senderIfaceAddr);
         break;
 
       case olsr::MessageHeader::MID_MESSAGE:
         NS_LOG_DEBUG(Simulator::Now().As(Time::S)
                      << " ETX-OLSR node " << m_mainAddress
                      << " received MID message of size "
                      << messageHeader.GetSerializedSize());
         ProcessMid(messageHeader, senderIfaceAddr);
         break;
 
       case olsr::MessageHeader::HNA_MESSAGE:
         NS_LOG_DEBUG(Simulator::Now().As(Time::S)
                      << " ETX-OLSR node " << m_mainAddress
                      << " received HNA message of size "
                      << messageHeader.GetSerializedSize());
         ProcessHna(messageHeader, senderIfaceAddr);
         break;
 
       default:
         NS_LOG_DEBUG("ETX-OLSR message type " << int(messageHeader.GetMessageType())
                                               << " not implemented");
       }
     }
     else
     {
       NS_LOG_DEBUG("ETX-OLSR message is duplicated, not reading it.");
       for (auto it = duplicated->ifaceList.begin(); it != duplicated->ifaceList.end(); it++)
       {
         if (*it == receiverIfaceAddr)
         {
           do_forwarding = false;
           break;
         }
       }
     }
 
     if (do_forwarding)
     {
       if (messageHeader.GetMessageType() != olsr::MessageHeader::HELLO_MESSAGE)
       {
         ForwardDefault(messageHeader,
                        duplicated,
                        receiverIfaceAddr,
                        inetSourceAddr.GetIpv4());
       }
     }
   }
 
   RoutingTableComputation();
   m_rxTcEtxTrailerThisPkt.clear();
 }
 
 int
 RoutingProtocol::Degree(const olsr::NeighborTuple& tuple)
 {
   int degree = 0;
  const olsr::TwoHopNeighborSet& twoHopNeighbors = m_state.GetTwoHopNeighbors();
  for (auto nb2hop_tuple = twoHopNeighbors.begin(); nb2hop_tuple != twoHopNeighbors.end();
       ++nb2hop_tuple)
   {
     if (nb2hop_tuple->neighborMainAddr == tuple.neighborMainAddr)
     {
       const olsr::NeighborTuple* nb_tuple =
           m_state.FindSymNeighborTuple(nb2hop_tuple->twoHopNeighborAddr);
       if (nb_tuple == nullptr)
       {
         degree++;
       }
     }
   }
   return degree;
 }
 
 namespace
 {
 void
 CoverTwoHopNeighbors(Ipv4Address neighborMainAddr, olsr::TwoHopNeighborSet& N2)
 {
   olsr::TwoHopNeighborSet::iterator twoHopNeighbor = N2.begin();
   while (twoHopNeighbor != N2.end())
   {
     if (twoHopNeighbor->neighborMainAddr == neighborMainAddr)
     {
       twoHopNeighbor = N2.erase(twoHopNeighbor);
     }
     else
     {
       twoHopNeighbor++;
     }
   }
 }
 } // unnamed namespace
 
 void
 RoutingProtocol::MprComputation()
 {
   NS_LOG_FUNCTION(this);
 
   olsr::MprSet mprSet;
 
   const olsr::NeighborSet& N = m_state.GetNeighbors();
   olsr::TwoHopNeighborSet N2 = m_state.GetTwoHopNeighbors();
 
  // Pre-compute "degree" once to avoid repeatedly scanning the full 2-hop set.
  // Degree here matches the legacy Degree() behavior: count 2-hop tuples whose 2-hop neighbor
  // is not a symmetric 1-hop neighbor.
  std::map<Ipv4Address, int> degreeByNeighbor;
  {
    const olsr::TwoHopNeighborSet& twoHopAll = m_state.GetTwoHopNeighbors();
    for (auto it = twoHopAll.begin(); it != twoHopAll.end(); ++it)
    {
      const olsr::TwoHopNeighborTuple& nb2hop_tuple = *it;
      if (m_state.FindSymNeighborTuple(nb2hop_tuple.twoHopNeighborAddr) == nullptr)
      {
        degreeByNeighbor[nb2hop_tuple.neighborMainAddr]++;
      }
    }
  }

   for (auto nb_tuple = N.begin(); nb_tuple != N.end(); nb_tuple++)
   {
     if (nb_tuple->willingness == olsr::Willingness::ALWAYS)
     {
       mprSet.insert(nb_tuple->neighborMainAddr);
       CoverTwoHopNeighbors(nb_tuple->neighborMainAddr, N2);
     }
   }
 
   while (true)
   {
     olsr::TwoHopNeighborSet uncoveredN2;
     for (auto nb2hop = N2.begin(); nb2hop != N2.end(); nb2hop++)
     {
       bool isMpr = (mprSet.find(nb2hop->neighborMainAddr) != mprSet.end());
       bool isSym = (m_state.FindSymNeighborTuple(nb2hop->twoHopNeighborAddr) != nullptr);
       if (!isMpr && !isSym)
       {
         uncoveredN2.push_back(*nb2hop);
       }
     }
     N2 = uncoveredN2;
 
     if (N2.empty())
     {
       break;
     }
 
     Ipv4Address maxAddr;
     int maxReachability = -1;
     int maxDegree = -1;
 
     for (auto nb_tuple = N.begin(); nb_tuple != N.end(); nb_tuple++)
     {
       if (nb_tuple->status != olsr::NeighborTuple::STATUS_SYM ||
           nb_tuple->willingness == olsr::Willingness::NEVER)
       {
         continue;
       }
 
       double linkEtx = std::numeric_limits<double>::infinity();
       for (const auto& link : m_state.GetLinks())
       {
         if (GetMainAddress(link.neighborIfaceAddr) == nb_tuple->neighborMainAddr)
         {
           linkEtx = std::min(linkEtx, GetLinkEtx(link.neighborIfaceAddr));
         }
       }
       if (linkEtx > m_etxThreshold)
       {
         continue;
       }
 
       int r = 0;
       for (auto nb2hop = N2.begin(); nb2hop != N2.end(); nb2hop++)
       {
         if (nb2hop->neighborMainAddr == nb_tuple->neighborMainAddr)
         {
           r++;
         }
       }
 
      int d = 0;
      auto degIt = degreeByNeighbor.find(nb_tuple->neighborMainAddr);
      if (degIt != degreeByNeighbor.end())
      {
        d = degIt->second;
      }
 
       if (r > maxReachability || (r == maxReachability && d > maxDegree))
       {
         maxReachability = r;
         maxDegree = d;
         maxAddr = nb_tuple->neighborMainAddr;
       }
     }
 
     if (maxReachability <= 0)
     {
       break;
     }
 
     mprSet.insert(maxAddr);
     CoverTwoHopNeighbors(maxAddr, N2);
   }
 
   NS_LOG_DEBUG("Computed MPR set for " << m_mainAddress << " [size=" << mprSet.size() << "]");
   m_state.SetMprSet(mprSet);
 }
 
 Ipv4Address
 RoutingProtocol::GetMainAddress(Ipv4Address iface_addr) const
 {
   const olsr::IfaceAssocTuple* tuple = m_state.FindIfaceAssocTuple(iface_addr);
   if (tuple != nullptr)
   {
     return tuple->mainAddr;
   }
   return iface_addr;
 }
 
 void
 RoutingProtocol::RoutingTableComputation()
 {
  if (!m_ipv4 || !m_hnaRoutingTable)
  {
    return;
  }
   PurgeExpiredAdvertisedEdgeEtx();
   NS_LOG_DEBUG(Simulator::Now().As(Time::S)
                << " : Node " << m_mainAddress
                << ": ETX RoutingTableComputation begin...");
 
   Clear();
 
   const olsr::NeighborSet& neighborSet = m_state.GetNeighbors();
   for (auto it = neighborSet.begin(); it != neighborSet.end(); it++)
   {
     const olsr::NeighborTuple& nb_tuple = *it;
     if (nb_tuple.status == olsr::NeighborTuple::STATUS_SYM)
     {
       bool nb_main_addr = false;
       const olsr::LinkTuple* lt = nullptr;
       const olsr::LinkSet& linkSet = m_state.GetLinks();
 
       for (auto it2 = linkSet.begin(); it2 != linkSet.end(); it2++)
       {
         const olsr::LinkTuple& link_tuple = *it2;
 
         if ((GetMainAddress(link_tuple.neighborIfaceAddr) == nb_tuple.neighborMainAddr) &&
             link_tuple.time >= Simulator::Now())
         {
           double linkEtx = GetLinkEtx(link_tuple.neighborIfaceAddr);
           lt = &link_tuple;
 
           RoutingTableEntry existing;
           bool hasExisting = Lookup(link_tuple.neighborIfaceAddr, existing);
           if (!hasExisting || existing.etxDistance > linkEtx)
           {
             AddEntry(link_tuple.neighborIfaceAddr,
                      link_tuple.neighborIfaceAddr,
                      link_tuple.localIfaceAddr,
                      1,
                      linkEtx);
           }
 
           if (link_tuple.neighborIfaceAddr == nb_tuple.neighborMainAddr)
           {
             nb_main_addr = true;
           }
         }
       }
 
       if (!nb_main_addr && lt != nullptr)
       {
         double linkEtx = GetLinkEtx(lt->neighborIfaceAddr);
         RoutingTableEntry existing;
         bool hasExisting = Lookup(nb_tuple.neighborMainAddr, existing);
         if (!hasExisting || existing.etxDistance > linkEtx)
         {
           AddEntry(nb_tuple.neighborMainAddr,
                    lt->neighborIfaceAddr,
                    lt->localIfaceAddr,
                    1,
                    linkEtx);
         }
       }
     }
   }
 
   const olsr::TwoHopNeighborSet& twoHopNeighbors = m_state.GetTwoHopNeighbors();
   for (auto it = twoHopNeighbors.begin(); it != twoHopNeighbors.end(); it++)
   {
     const olsr::TwoHopNeighborTuple& nb2hop_tuple = *it;
 
     if (m_state.FindSymNeighborTuple(nb2hop_tuple.twoHopNeighborAddr))
     {
       continue;
     }
 
     if (nb2hop_tuple.twoHopNeighborAddr == m_mainAddress)
     {
       continue;
     }
 
     bool nb2hopOk = false;
     for (auto nb = neighborSet.begin(); nb != neighborSet.end(); nb++)
     {
       if (nb->neighborMainAddr == nb2hop_tuple.neighborMainAddr &&
           nb->willingness != olsr::Willingness::NEVER)
       {
         nb2hopOk = true;
         break;
       }
     }
     if (!nb2hopOk)
     {
       continue;
     }
 
     RoutingTableEntry entry;
     if (Lookup(nb2hop_tuple.neighborMainAddr, entry))
     {
       double etx = entry.etxDistance + m_initialEtx;
       RoutingTableEntry existing;
       bool hasExisting = Lookup(nb2hop_tuple.twoHopNeighborAddr, existing);
       if (!hasExisting || existing.etxDistance > etx ||
           (std::abs(existing.etxDistance - etx) < 1e-9 && existing.distance > 2))
       {
         AddEntry(nb2hop_tuple.twoHopNeighborAddr, entry.nextAddr, entry.interface, 2, etx);
       }
     }
   }
 
   bool improved = true;
   while (improved)
   {
     improved = false;
     const olsr::TopologySet& topology = m_state.GetTopologySet();
     for (auto it = topology.begin(); it != topology.end(); it++)
     {
       const olsr::TopologyTuple& topology_tuple = *it;
 
       RoutingTableEntry lastAddrEntry;
       bool have_lastAddrEntry = Lookup(topology_tuple.lastAddr, lastAddrEntry);
       if (!have_lastAddrEntry)
       {
         continue;
       }
 
       const double hopEtx =
           GetAdvertisedEdgeEtx(topology_tuple.lastAddr, topology_tuple.destAddr);
       double newEtx = lastAddrEntry.etxDistance + hopEtx;
       uint32_t newDist = lastAddrEntry.distance + 1;
 
       RoutingTableEntry destAddrEntry;
       bool have_destAddrEntry = Lookup(topology_tuple.destAddr, destAddrEntry);
 
       if (!have_destAddrEntry ||
           destAddrEntry.etxDistance > newEtx + 1e-9 ||
           (std::abs(destAddrEntry.etxDistance - newEtx) < 1e-9 &&
            destAddrEntry.distance > newDist))
       {
         AddEntry(topology_tuple.destAddr,
                  lastAddrEntry.nextAddr,
                  lastAddrEntry.interface,
                  newDist,
                  newEtx);
         improved = true;
       }
     }
   }
 
   const olsr::IfaceAssocSet& ifaceAssocSet = m_state.GetIfaceAssocSet();
   for (auto it = ifaceAssocSet.begin(); it != ifaceAssocSet.end(); it++)
   {
     const olsr::IfaceAssocTuple& tuple = *it;
     RoutingTableEntry entry1;
     RoutingTableEntry entry2;
     bool have_entry1 = Lookup(tuple.mainAddr, entry1);
     bool have_entry2 = Lookup(tuple.ifaceAddr, entry2);
     if (have_entry1 && !have_entry2)
     {
       AddEntry(tuple.ifaceAddr,
                entry1.nextAddr,
                entry1.interface,
                entry1.distance,
                entry1.etxDistance);
     }
   }
 
   const olsr::AssociationSet& associationSet = m_state.GetAssociationSet();
 
   for (uint32_t i = 0; i < m_hnaRoutingTable->GetNRoutes(); i++)
   {
     m_hnaRoutingTable->RemoveRoute(0);
   }
 
   for (auto it = associationSet.begin(); it != associationSet.end(); it++)
   {
     const olsr::AssociationTuple& tuple = *it;
 
     bool goToNextAssociationTuple = false;
     const olsr::Associations& localHnaAssociations = m_state.GetAssociations();
     for (auto assocIt = localHnaAssociations.begin(); assocIt != localHnaAssociations.end();
          assocIt++)
     {
       const olsr::Association& localHnaAssoc = *assocIt;
       if (localHnaAssoc.networkAddr == tuple.networkAddr &&
           localHnaAssoc.netmask == tuple.netmask)
       {
         goToNextAssociationTuple = true;
         break;
       }
     }
     if (goToNextAssociationTuple)
     {
       continue;
     }
 
     RoutingTableEntry gatewayEntry;
     bool gatewayEntryExists = Lookup(tuple.gatewayAddr, gatewayEntry);
     bool addRoute = false;
     uint32_t routeIndex = 0;
 
     for (routeIndex = 0; routeIndex < m_hnaRoutingTable->GetNRoutes(); routeIndex++)
     {
       Ipv4RoutingTableEntry route = m_hnaRoutingTable->GetRoute(routeIndex);
       if (route.GetDestNetwork() == tuple.networkAddr &&
           route.GetDestNetworkMask() == tuple.netmask)
       {
         break;
       }
     }
 
     if (routeIndex == m_hnaRoutingTable->GetNRoutes())
     {
       addRoute = true;
     }
     else if (gatewayEntryExists &&
              m_hnaRoutingTable->GetMetric(routeIndex) > gatewayEntry.distance)
     {
       m_hnaRoutingTable->RemoveRoute(routeIndex);
       addRoute = true;
     }
 
     if (addRoute && gatewayEntryExists)
     {
       m_hnaRoutingTable->AddNetworkRouteTo(tuple.networkAddr,
                                            tuple.netmask,
                                            gatewayEntry.nextAddr,
                                            gatewayEntry.interface,
                                            gatewayEntry.distance);
     }
   }
 
   // ---------------- Candidate table (K=3 from pure ETX control plane) ----------------
   const auto oldCandidatePathTable = m_candidatePathTable;
   m_candidateTable.clear();
   m_candidatePathTable.clear();
   const olsr::TopologySet& topology = m_state.GetTopologySet();
 
   for (const auto& kv : m_table)
   {
     const Ipv4Address dest = kv.first;
     const RoutingTableEntry& primaryEntry = kv.second;
 
     CandidateSet cset{};
     std::array<std::vector<Ipv4Address>, 3> pathSet;
     cset[0].nextHop = primaryEntry.nextAddr;
     cset[0].interface = primaryEntry.interface;
     cset[0].ctrlCost = primaryEntry.etxDistance;
 
     struct NextHopCandidate
     {
       Ipv4Address nextHop;
       uint32_t iface;
       double baseCost;
       std::set<std::pair<Ipv4Address, Ipv4Address>> pathEdges;
     };
     std::vector<NextHopCandidate> pool;
 
     NextHopCandidate primaryCand;
     primaryCand.nextHop = cset[0].nextHop;
     primaryCand.iface = cset[0].interface;
     primaryCand.baseCost = cset[0].ctrlCost;
     pathSet[0] = BuildPathSequenceForNextHop(dest, cset[0].nextHop);
     for (std::size_t ei = 0; ei + 1 < pathSet[0].size(); ++ei)
     {
       primaryCand.pathEdges.insert(std::make_pair(pathSet[0][ei], pathSet[0][ei + 1]));
     }
     pool.push_back(primaryCand);
 
     for (const auto& topo : topology)
     {
       if (topo.destAddr != dest)
       {
         continue;
       }
       RoutingTableEntry lastEntry;
       if (!Lookup(topo.lastAddr, lastEntry))
       {
         continue;
       }
       const Ipv4Address candNext = lastEntry.nextAddr;
       bool exists = false;
       for (const auto& e : pool)
       {
         if (e.nextHop == candNext)
         {
           exists = true;
           break;
         }
       }
       if (exists)
       {
         continue;
       }
       NextHopCandidate cand;
       cand.nextHop = candNext;
       cand.iface = lastEntry.interface;
       cand.baseCost = lastEntry.etxDistance + GetAdvertisedEdgeEtx(topo.lastAddr, topo.destAddr);
       const auto seq = BuildPathSequenceForNextHop(dest, candNext);
       for (std::size_t ei = 0; ei + 1 < seq.size(); ++ei)
       {
         cand.pathEdges.insert(std::make_pair(seq[ei], seq[ei + 1]));
       }
       pool.push_back(cand);
     }
 
     std::vector<NextHopCandidate> selected;
     selected.push_back(primaryCand);
     cset[1] = cset[0];
     cset[2] = cset[0];
 
     for (int idx = 1; idx < 3; ++idx)
     {
       // "Political correctness" constraints:
       // - idx=1: prefer Stable neighbors
       // - idx=2: avoid Noisy neighbors
       const bool requireStable = (idx == 1);
       const bool avoidNoisy = (idx == 2);
 
       const double pruneWin = std::max(m_stableWindowSec, m_noisyDropWindowSec);
       for (const auto& c : pool)
       {
         PruneDropEvents(c.nextHop, pruneWin);
       }
 
       double bestCost = std::numeric_limits<double>::infinity();
       int bestPoolIndex = -1;
 
       // Two-pass selection: if strict constraint yields no candidate, fall back to soft.
       const int passes = 2;
       for (int pass = 0; pass < passes && bestPoolIndex < 0; ++pass)
       {
         const bool strict = (pass == 0);
 
         for (int pi = 0; pi < static_cast<int>(pool.size()); ++pi)
         {
           const auto& cand = pool[pi];
           bool already = false;
           for (const auto& sel : selected)
           {
             if (sel.nextHop == cand.nextHop)
             {
               already = true;
               break;
             }
           }
           if (already)
           {
             continue;
           }
 
           const uint8_t labels = GetNeighborLabels(cand.nextHop);
           if (strict && requireStable && ((labels & LABEL_STABLE) == 0))
           {
             continue;
           }
           if (strict && avoidNoisy && ((labels & LABEL_NOISY) != 0))
           {
             continue;
           }
 
           double penaltySum = 0.0;
           for (const auto& sel : selected)
           {
             if (cand.pathEdges.empty())
             {
               continue;
             }
             uint32_t overlap = 0;
             for (const auto& e : cand.pathEdges)
             {
               if (sel.pathEdges.count(e) > 0)
               {
                 overlap++;
               }
             }
             const double overlapRatio =
                 static_cast<double>(overlap) / static_cast<double>(cand.pathEdges.size());
             penaltySum += m_overlapPenalty * overlapRatio;
           }
 
           // Soft preference: penalize Noisy candidates even if we had to fall back.
           if (!strict && avoidNoisy && ((labels & LABEL_NOISY) != 0))
           {
             penaltySum += 1e6;
           }
 
           const double softCost = cand.baseCost + penaltySum;
           if (softCost < bestCost)
           {
             bestCost = softCost;
             bestPoolIndex = pi;
           }
         }
       }
 
       if (bestPoolIndex >= 0)
       {
         const auto& chosenCand = pool[bestPoolIndex];
         cset[idx].nextHop = chosenCand.nextHop;
         cset[idx].interface = chosenCand.iface;
         cset[idx].ctrlCost = bestCost;
         pathSet[idx] = BuildPathSequenceForNextHop(dest, chosenCand.nextHop);
         selected.push_back(chosenCand);
       }
     }
 
     m_candidateTable[dest] = cset;
     m_candidatePathTable[dest] = pathSet;
     auto tableIt = m_table.find(dest);
     if (tableIt != m_table.end())
     {
       tableIt->second.nextHops.clear();
       for (const auto& cand : cset)
       {
         if (cand.nextHop != Ipv4Address() &&
             std::find(tableIt->second.nextHops.begin(), tableIt->second.nextHops.end(), cand.nextHop) ==
                 tableIt->second.nextHops.end())
         {
           tableIt->second.nextHops.push_back(cand.nextHop);
         }
       }
     }
 
     auto oldPathsIt = oldCandidatePathTable.find(dest);
     if (oldPathsIt != oldCandidatePathTable.end())
     {
       for (std::size_t idx = 0; idx < pathSet.size(); ++idx)
       {
         if (oldPathsIt->second[idx] != pathSet[idx] && cset[idx].nextHop != Ipv4Address())
         {
           ResetArmForNextHop(cset[idx].nextHop);
         }
       }
     }
 
     NS_LOG_DEBUG("CandidateTable: dest=" << dest
                   << " p1=" << cset[0].nextHop
                   << " p2=" << cset[1].nextHop
                   << " p3=" << cset[2].nextHop);
   }
 
   // ---------------- end candidate table ----------------
 
   NS_LOG_DEBUG("Node " << m_mainAddress << ": ETX RoutingTableComputation end. "
                        << GetSize() << " routes.");
   m_routingTableChanged(GetSize());
 }
 
 void
 RoutingProtocol::ProcessHello(const olsr::MessageHeader& msg,
                               const Ipv4Address& receiverIface,
                               const Ipv4Address& senderIface)
 {
   NS_LOG_FUNCTION(msg << receiverIface << senderIface);
   const olsr::MessageHeader::Hello& hello = msg.GetHello();
 
   LinkSensing(msg, hello, receiverIface, senderIface);
   PopulateNeighborSet(msg, hello);
   PopulateTwoHopNeighborSet(msg, hello);
   MprComputation();
   PopulateMprSelectorSet(msg, hello);
 }
 
 void
 RoutingProtocol::ProcessTc(const olsr::MessageHeader& msg, const Ipv4Address& senderIface)
 {
   const olsr::MessageHeader::Tc& tc = msg.GetTc();
   Time now = Simulator::Now();
 
   const olsr::LinkTuple* link_tuple = m_state.FindSymLinkTuple(senderIface, now);
   if (link_tuple == nullptr)
   {
     return;
   }
 
   const olsr::TopologyTuple* topologyTuple =
       m_state.FindNewerTopologyTuple(msg.GetOriginatorAddress(), tc.ansn);
   if (topologyTuple != nullptr)
   {
     return;
   }
 
   m_state.EraseOlderTopologyTuples(msg.GetOriginatorAddress(), tc.ansn);
 
   for (auto i = tc.neighborAddresses.begin(); i != tc.neighborAddresses.end(); i++)
   {
     const Ipv4Address& addr = *i;
     olsr::TopologyTuple* existingTuple =
         m_state.FindTopologyTuple(addr, msg.GetOriginatorAddress());
 
     if (existingTuple != nullptr)
     {
       existingTuple->expirationTime = now + msg.GetVTime();
     }
     else
     {
       olsr::TopologyTuple newTuple;
       newTuple.destAddr = addr;
       newTuple.lastAddr = msg.GetOriginatorAddress();
       newTuple.sequenceNumber = tc.ansn;
       newTuple.expirationTime = now + msg.GetVTime();
       AddTopologyTuple(newTuple);
 
       m_events.Track(Simulator::Schedule(DELAY(newTuple.expirationTime),
                                          &RoutingProtocol::TopologyTupleTimerExpire,
                                          this,
                                          newTuple.destAddr,
                                          newTuple.lastAddr));
     }
   }
 
   ApplyTcEtxTrailerForMessage(msg);
 }
 
 void
 RoutingProtocol::ProcessMid(const olsr::MessageHeader& msg, const Ipv4Address& senderIface)
 {
   const olsr::MessageHeader::Mid& mid = msg.GetMid();
   Time now = Simulator::Now();
 
   const olsr::LinkTuple* linkTuple = m_state.FindSymLinkTuple(senderIface, now);
   if (linkTuple == nullptr)
   {
     return;
   }
 
   for (auto i = mid.interfaceAddresses.begin(); i != mid.interfaceAddresses.end(); i++)
   {
     bool updated = false;
     olsr::IfaceAssocSet& ifaceAssoc = m_state.GetIfaceAssocSetMutable();
     for (auto tuple = ifaceAssoc.begin(); tuple != ifaceAssoc.end(); tuple++)
     {
       if (tuple->ifaceAddr == *i && tuple->mainAddr == msg.GetOriginatorAddress())
       {
         tuple->time = now + msg.GetVTime();
         updated = true;
       }
     }
     if (!updated)
     {
       olsr::IfaceAssocTuple tuple;
       tuple.ifaceAddr = *i;
       tuple.mainAddr = msg.GetOriginatorAddress();
       tuple.time = now + msg.GetVTime();
       AddIfaceAssocTuple(tuple);
       Simulator::Schedule(DELAY(tuple.time),
                           &RoutingProtocol::IfaceAssocTupleTimerExpire,
                           this,
                           tuple.ifaceAddr);
     }
   }
 
   olsr::NeighborSet& neighbors = m_state.GetNeighbors();
   for (auto nb = neighbors.begin(); nb != neighbors.end(); nb++)
   {
     nb->neighborMainAddr = GetMainAddress(nb->neighborMainAddr);
   }
 
   olsr::TwoHopNeighborSet& twoHopNeighbors = m_state.GetTwoHopNeighbors();
   for (auto nb2 = twoHopNeighbors.begin(); nb2 != twoHopNeighbors.end(); nb2++)
   {
     nb2->neighborMainAddr = GetMainAddress(nb2->neighborMainAddr);
     nb2->twoHopNeighborAddr = GetMainAddress(nb2->twoHopNeighborAddr);
   }
 }
 
 void
 RoutingProtocol::ProcessHna(const olsr::MessageHeader& msg, const Ipv4Address& senderIface)
 {
   const olsr::MessageHeader::Hna& hna = msg.GetHna();
   Time now = Simulator::Now();
 
   const olsr::LinkTuple* link_tuple = m_state.FindSymLinkTuple(senderIface, now);
   if (link_tuple == nullptr)
   {
     return;
   }
 
   for (auto it = hna.associations.begin(); it != hna.associations.end(); it++)
   {
     olsr::AssociationTuple* tuple =
         m_state.FindAssociationTuple(msg.GetOriginatorAddress(), it->address, it->mask);
 
     if (tuple != nullptr)
     {
       tuple->expirationTime = now + msg.GetVTime();
     }
     else
     {
       olsr::AssociationTuple assocTuple = {msg.GetOriginatorAddress(),
                                            it->address,
                                            it->mask,
                                            now + msg.GetVTime()};
       AddAssociationTuple(assocTuple);
       Simulator::Schedule(DELAY(assocTuple.expirationTime),
                           &RoutingProtocol::AssociationTupleTimerExpire,
                           this,
                           assocTuple.gatewayAddr,
                           assocTuple.networkAddr,
                           assocTuple.netmask);
     }
   }
 }
 
 void
 RoutingProtocol::ForwardDefault(olsr::MessageHeader olsrMessage,
                                 olsr::DuplicateTuple* duplicated,
                                 const Ipv4Address& localIface,
                                 const Ipv4Address& senderAddress)
 {
   Time now = Simulator::Now();
 
   const olsr::LinkTuple* linkTuple = m_state.FindSymLinkTuple(senderAddress, now);
   if (linkTuple == nullptr)
   {
     return;
   }
 
   if (duplicated != nullptr && duplicated->retransmitted)
   {
     return;
   }
 
   bool retransmitted = false;
   if (olsrMessage.GetTimeToLive() > 1)
   {
     const olsr::MprSelectorTuple* mprselTuple =
         m_state.FindMprSelectorTuple(GetMainAddress(senderAddress));
     if (mprselTuple != nullptr)
     {
       olsrMessage.SetTimeToLive(olsrMessage.GetTimeToLive() - 1);
       olsrMessage.SetHopCount(olsrMessage.GetHopCount() + 1);
       QueueMessage(olsrMessage, JITTER);
       retransmitted = true;
     }
   }
 
   if (duplicated != nullptr)
   {
     duplicated->expirationTime = now + OLSR_DUP_HOLD_TIME;
     duplicated->retransmitted = retransmitted;
     duplicated->ifaceList.push_back(localIface);
   }
   else
   {
     olsr::DuplicateTuple newDup;
     newDup.address = olsrMessage.GetOriginatorAddress();
     newDup.sequenceNumber = olsrMessage.GetMessageSequenceNumber();
     newDup.expirationTime = now + OLSR_DUP_HOLD_TIME;
     newDup.retransmitted = retransmitted;
     newDup.ifaceList.push_back(localIface);
     AddDuplicateTuple(newDup);
     Simulator::Schedule(OLSR_DUP_HOLD_TIME,
                         &RoutingProtocol::DupTupleTimerExpire,
                         this,
                         newDup.address,
                         newDup.sequenceNumber);
   }
 }
 
 void
 RoutingProtocol::QueueMessage(const olsr::MessageHeader& message, Time delay)
 {
   m_queuedMessages.push_back(message);
   m_queuedTcEtxExtensions.push_back(ComputeTcTrailerForQueuedMessage(message));
   NS_ASSERT(m_queuedTcEtxExtensions.size() == m_queuedMessages.size());
   if (!m_queuedMessagesTimer.IsRunning())
   {
     m_queuedMessagesTimer.SetDelay(delay);
     m_queuedMessagesTimer.Schedule();
   }
 }
 
 void
 RoutingProtocol::SendPacket(Ptr<Packet> packet,
                               const olsr::MessageList& containedMessages,
                               Ptr<Packet> tcEtxTrailer)
 {
   NS_LOG_DEBUG("ETX-OLSR node " << m_mainAddress << " sending a OLSR packet");
 
   const uint32_t olsrPayloadBytes = packet->GetSize();
   if (tcEtxTrailer && tcEtxTrailer->GetSize() > 0)
   {
     packet->AddAtEnd(tcEtxTrailer);
   }
 
   olsr::PacketHeader header;
   header.SetPacketLength(header.GetSerializedSize() + olsrPayloadBytes);
   header.SetPacketSequenceNumber(GetPacketSequenceNumber());
   packet->AddHeader(header);
 
   m_txPacketTrace(header, containedMessages);
 
   for (auto i = m_sendSockets.begin(); i != m_sendSockets.end(); i++)
   {
     Ptr<Packet> pkt = packet->Copy();
     Ipv4Address bcast = i->second.GetLocal().GetSubnetDirectedBroadcast(i->second.GetMask());
     i->first->SendTo(pkt, 0, InetSocketAddress(bcast, OLSR_PORT_NUMBER));
   }
 }
 
 void
 RoutingProtocol::SendQueuedMessages()
 {
   NS_LOG_DEBUG("ETX-OLSR node " << m_mainAddress << ": SendQueuedMessages");
 
   std::size_t idx = 0;
   const std::size_t total = m_queuedMessages.size();
   NS_ASSERT(m_queuedTcEtxExtensions.size() == total);
 
   while (idx < total)
   {
     Ptr<Packet> packet = Create<Packet>();
     olsr::MessageList msglist;
     int numMessages = 0;
     std::vector<std::optional<TcEtxTrailerBlock>> batchExt;
     while (idx < total && numMessages < OLSR_MAX_MSGS)
     {
       Ptr<Packet> p = Create<Packet>();
       p->AddHeader(m_queuedMessages[idx]);
       packet->AddAtEnd(p);
       msglist.push_back(m_queuedMessages[idx]);
       batchExt.push_back(m_queuedTcEtxExtensions[idx]);
       ++idx;
       ++numMessages;
     }
     Ptr<Packet> trailer = SerializeTcEtxTrailerFromOptionals(batchExt);
     SendPacket(packet, msglist, trailer);
   }
 
   m_queuedMessages.clear();
   m_queuedTcEtxExtensions.clear();
 }
 
 void
 RoutingProtocol::SendHello()
 {
   NS_LOG_FUNCTION(this);
 
   olsr::MessageHeader msg;
   Time now = Simulator::Now();
 
   msg.SetVTime(OLSR_NEIGHB_HOLD_TIME);
   msg.SetOriginatorAddress(m_mainAddress);
   msg.SetTimeToLive(1);
   msg.SetHopCount(0);
   msg.SetMessageSequenceNumber(GetMessageSequenceNumber());
   olsr::MessageHeader::Hello& hello = msg.GetHello();
 
   hello.SetHTime(m_helloInterval);
   hello.willingness = m_willingness;
 
   std::vector<olsr::MessageHeader::Hello::LinkMessage>& linkMessages = hello.linkMessages;
 
   const olsr::LinkSet& links = m_state.GetLinks();
   for (auto link_tuple = links.begin(); link_tuple != links.end(); link_tuple++)
   {
     if (!(GetMainAddress(link_tuple->localIfaceAddr) == m_mainAddress &&
           link_tuple->time >= now))
     {
       continue;
     }
 
     LinkType linkType;
     NeighborType neighborType;
 
     if (link_tuple->symTime >= now)
     {
       linkType = LinkType::SYM_LINK;
     }
     else if (link_tuple->asymTime >= now)
     {
       linkType = LinkType::ASYM_LINK;
     }
     else
     {
       linkType = LinkType::LOST_LINK;
     }
 
     if (m_state.FindMprAddress(GetMainAddress(link_tuple->neighborIfaceAddr)))
     {
       neighborType = NeighborType::MPR_NEIGH;
     }
     else
     {
       bool ok = false;
       for (auto nb_tuple = m_state.GetNeighbors().begin();
            nb_tuple != m_state.GetNeighbors().end();
            nb_tuple++)
       {
         if (nb_tuple->neighborMainAddr == GetMainAddress(link_tuple->neighborIfaceAddr))
         {
           if (nb_tuple->status == olsr::NeighborTuple::STATUS_SYM)
           {
             neighborType = NeighborType::SYM_NEIGH;
           }
           else
           {
             neighborType = NeighborType::NOT_NEIGH;
           }
           ok = true;
           break;
         }
       }
       if (!ok)
       {
         NS_LOG_WARN("Unknown neighbor " << GetMainAddress(link_tuple->neighborIfaceAddr));
         continue;
       }
     }
 
     olsr::MessageHeader::Hello::LinkMessage linkMessage;
     linkMessage.linkCode = (static_cast<uint8_t>(linkType) & 0x03) |
                            ((static_cast<uint8_t>(neighborType) << 2) & 0x0f);
     linkMessage.neighborInterfaceAddresses.push_back(link_tuple->neighborIfaceAddr);
 
     std::vector<Ipv4Address> interfaces =
         m_state.FindNeighborInterfaces(link_tuple->neighborIfaceAddr);
     linkMessage.neighborInterfaceAddresses.insert(
         linkMessage.neighborInterfaceAddresses.end(),
         interfaces.begin(),
         interfaces.end());
 
     linkMessages.push_back(linkMessage);
   }
   QueueMessage(msg, JITTER);
 }
 
 void
 RoutingProtocol::SendTc()
 {
   NS_LOG_FUNCTION(this);
 
   olsr::MessageHeader msg;
   msg.SetVTime(OLSR_TOP_HOLD_TIME);
   msg.SetOriginatorAddress(m_mainAddress);
   msg.SetTimeToLive(255);
   msg.SetHopCount(0);
   msg.SetMessageSequenceNumber(GetMessageSequenceNumber());
 
   olsr::MessageHeader::Tc& tc = msg.GetTc();
   tc.ansn = m_ansn;
   tc.neighborAddresses.clear();
 
   for (auto mprsel_tuple = m_state.GetMprSelectors().begin();
        mprsel_tuple != m_state.GetMprSelectors().end();
        mprsel_tuple++)
   {
     tc.neighborAddresses.push_back(mprsel_tuple->mainAddr);
   }
   QueueMessage(msg, JITTER);
 }
 
 void
 RoutingProtocol::SendMid()
 {
  if (!m_ipv4)
  {
    return;
  }

   olsr::MessageHeader msg;
   olsr::MessageHeader::Mid& mid = msg.GetMid();
 
   Ipv4Address loopback("127.0.0.1");
   for (uint32_t i = 0; i < m_ipv4->GetNInterfaces(); i++)
   {
     Ipv4Address addr = m_ipv4->GetAddress(i, 0).GetLocal();
     if (addr != m_mainAddress && addr != loopback &&
         m_interfaceExclusions.find(i) == m_interfaceExclusions.end())
     {
       mid.interfaceAddresses.push_back(addr);
     }
   }
   if (mid.interfaceAddresses.empty())
   {
     return;
   }
 
   msg.SetVTime(OLSR_MID_HOLD_TIME);
   msg.SetOriginatorAddress(m_mainAddress);
   msg.SetTimeToLive(255);
   msg.SetHopCount(0);
   msg.SetMessageSequenceNumber(GetMessageSequenceNumber());
   QueueMessage(msg, JITTER);
 }
 
 void
 RoutingProtocol::SendHna()
 {
   olsr::MessageHeader msg;
   msg.SetVTime(OLSR_HNA_HOLD_TIME);
   msg.SetOriginatorAddress(m_mainAddress);
   msg.SetTimeToLive(255);
   msg.SetHopCount(0);
   msg.SetMessageSequenceNumber(GetMessageSequenceNumber());
   olsr::MessageHeader::Hna& hna = msg.GetHna();
 
   const olsr::Associations& localHnaAssociations = m_state.GetAssociations();
   for (auto it = localHnaAssociations.begin(); it != localHnaAssociations.end(); it++)
   {
     olsr::MessageHeader::Hna::Association assoc = {it->networkAddr, it->netmask};
     hna.associations.push_back(assoc);
   }
   if (hna.associations.empty())
   {
     return;
   }
   QueueMessage(msg, JITTER);
 }
 
 void
 RoutingProtocol::AddHostNetworkAssociation(Ipv4Address networkAddr, Ipv4Mask netmask)
 {
   const olsr::Associations& localHnaAssociations = m_state.GetAssociations();
   for (auto it = localHnaAssociations.begin(); it != localHnaAssociations.end(); it++)
   {
     if (it->networkAddr == networkAddr && it->netmask == netmask)
     {
       return;
     }
   }
   m_state.InsertAssociation(olsr::Association{networkAddr, netmask});
 }
 
 void
 RoutingProtocol::RemoveHostNetworkAssociation(Ipv4Address networkAddr, Ipv4Mask netmask)
 {
   m_state.EraseAssociation(olsr::Association{networkAddr, netmask});
 }
 
 void
 RoutingProtocol::SetRoutingTableAssociation(Ptr<Ipv4StaticRouting> routingTable)
 {
   if (m_routingTableAssociation)
   {
     for (uint32_t i = 0; i < m_routingTableAssociation->GetNRoutes(); i++)
     {
       Ipv4RoutingTableEntry route = m_routingTableAssociation->GetRoute(i);
       if (UsesNonOlsrOutgoingInterface(route))
       {
         RemoveHostNetworkAssociation(route.GetDestNetwork(), route.GetDestNetworkMask());
       }
     }
   }
 
   m_routingTableAssociation = routingTable;
 
   for (uint32_t i = 0; i < m_routingTableAssociation->GetNRoutes(); i++)
   {
     Ipv4RoutingTableEntry route = m_routingTableAssociation->GetRoute(i);
     if (UsesNonOlsrOutgoingInterface(route))
     {
       AddHostNetworkAssociation(route.GetDestNetwork(), route.GetDestNetworkMask());
     }
   }
 }
 
 bool
 RoutingProtocol::UsesNonOlsrOutgoingInterface(const Ipv4RoutingTableEntry& route)
 {
   auto ci = m_interfaceExclusions.find(route.GetInterface());
   return ci != m_interfaceExclusions.end();
 }
 
 void
 RoutingProtocol::LinkSensing(const olsr::MessageHeader& msg,
                              const olsr::MessageHeader::Hello& hello,
                              const Ipv4Address& receiverIface,
                              const Ipv4Address& senderIface)
 {
   Time now = Simulator::Now();
   bool updated = false;
   bool created = false;
 
   NS_ASSERT(msg.GetVTime().IsStrictlyPositive());
   olsr::LinkTuple* link_tuple = m_state.FindLinkTuple(senderIface);
   if (link_tuple == nullptr)
   {
     olsr::LinkTuple newLinkTuple;
     newLinkTuple.neighborIfaceAddr = senderIface;
     newLinkTuple.localIfaceAddr = receiverIface;
     newLinkTuple.symTime = now - Seconds(1);
     newLinkTuple.time = now + msg.GetVTime();
     link_tuple = &m_state.InsertLinkTuple(newLinkTuple);
     created = true;
   }
   else
   {
     updated = true;
   }
 
   link_tuple->asymTime = now + msg.GetVTime();
   for (auto linkMessage = hello.linkMessages.begin(); linkMessage != hello.linkMessages.end();
        linkMessage++)
   {
     auto linkType = LinkType(linkMessage->linkCode & 0x03);
     auto neighborType = NeighborType((linkMessage->linkCode >> 2) & 0x03);
 
     if ((linkType == LinkType::SYM_LINK && neighborType == NeighborType::NOT_NEIGH) ||
         (neighborType != NeighborType::SYM_NEIGH &&
          neighborType != NeighborType::MPR_NEIGH &&
          neighborType != NeighborType::NOT_NEIGH))
     {
       continue;
     }
 
     for (auto neighIfaceAddr = linkMessage->neighborInterfaceAddresses.begin();
          neighIfaceAddr != linkMessage->neighborInterfaceAddresses.end();
          neighIfaceAddr++)
     {
       if (*neighIfaceAddr == receiverIface)
       {
         if (linkType == LinkType::LOST_LINK)
         {
           link_tuple->symTime = now - Seconds(1);
           updated = true;
         }
         else if (linkType == LinkType::SYM_LINK || linkType == LinkType::ASYM_LINK)
         {
           link_tuple->symTime = now + msg.GetVTime();
           link_tuple->time = link_tuple->symTime + OLSR_NEIGHB_HOLD_TIME;
           updated = true;
         }
         else
         {
           NS_FATAL_ERROR("Bad link type");
         }
         break;
       }
     }
   }
   link_tuple->time = std::max(link_tuple->time, link_tuple->asymTime);
 
   if (updated)
   {
     LinkTupleUpdated(*link_tuple, hello.willingness);
   }
 
   if (created)
   {
     LinkTupleAdded(*link_tuple, hello.willingness);
     m_events.Track(
         Simulator::Schedule(DELAY(std::min(link_tuple->time, link_tuple->symTime)),
                             &RoutingProtocol::LinkTupleTimerExpire,
                             this,
                             link_tuple->neighborIfaceAddr));
   }
 }
 
 void
 RoutingProtocol::PopulateNeighborSet(const olsr::MessageHeader& msg,
                                     const olsr::MessageHeader::Hello& hello)
 {
   olsr::NeighborTuple* nb_tuple = m_state.FindNeighborTuple(msg.GetOriginatorAddress());
   if (nb_tuple != nullptr)
   {
     nb_tuple->willingness = hello.willingness;
   }
 }
 
 void
 RoutingProtocol::PopulateTwoHopNeighborSet(const olsr::MessageHeader& msg,
                                            const olsr::MessageHeader::Hello& hello)
 {
   Time now = Simulator::Now();
 
   for (auto link_tuple = m_state.GetLinks().begin(); link_tuple != m_state.GetLinks().end();
        link_tuple++)
   {
     if (GetMainAddress(link_tuple->neighborIfaceAddr) != msg.GetOriginatorAddress())
     {
       continue;
     }
 
     if (link_tuple->symTime < now)
     {
       continue;
     }
 
     for (auto linkMessage = hello.linkMessages.begin();
          linkMessage != hello.linkMessages.end();
          linkMessage++)
     {
       auto neighborType = NeighborType((linkMessage->linkCode >> 2) & 0x3);
 
       for (auto nb2hop_addr_iter = linkMessage->neighborInterfaceAddresses.begin();
            nb2hop_addr_iter != linkMessage->neighborInterfaceAddresses.end();
            nb2hop_addr_iter++)
       {
         Ipv4Address nb2hop_addr = GetMainAddress(*nb2hop_addr_iter);
         if (neighborType == NeighborType::SYM_NEIGH ||
             neighborType == NeighborType::MPR_NEIGH)
         {
           if (nb2hop_addr == m_mainAddress)
           {
             continue;
           }
 
           olsr::TwoHopNeighborTuple* nb2hop_tuple =
               m_state.FindTwoHopNeighborTuple(msg.GetOriginatorAddress(), nb2hop_addr);
           if (nb2hop_tuple == nullptr)
           {
             olsr::TwoHopNeighborTuple new_nb2hop_tuple;
             new_nb2hop_tuple.neighborMainAddr = msg.GetOriginatorAddress();
             new_nb2hop_tuple.twoHopNeighborAddr = nb2hop_addr;
             new_nb2hop_tuple.expirationTime = now + msg.GetVTime();
             AddTwoHopNeighborTuple(new_nb2hop_tuple);
             m_events.Track(Simulator::Schedule(DELAY(new_nb2hop_tuple.expirationTime),
                                                &RoutingProtocol::Nb2hopTupleTimerExpire,
                                                this,
                                                new_nb2hop_tuple.neighborMainAddr,
                                                new_nb2hop_tuple.twoHopNeighborAddr));
           }
           else
           {
             nb2hop_tuple->expirationTime = now + msg.GetVTime();
           }
         }
         else if (neighborType == NeighborType::NOT_NEIGH)
         {
           m_state.EraseTwoHopNeighborTuples(msg.GetOriginatorAddress(), nb2hop_addr);
         }
       }
     }
   }
 }
 
 void
 RoutingProtocol::PopulateMprSelectorSet(const olsr::MessageHeader& msg,
                                        const olsr::MessageHeader::Hello& hello)
 {
   Time now = Simulator::Now();
   double senderEtx = std::numeric_limits<double>::infinity();
   for (const auto& link : m_state.GetLinks())
   {
     if (GetMainAddress(link.neighborIfaceAddr) == msg.GetOriginatorAddress())
     {
       senderEtx = std::min(senderEtx, GetLinkEtx(link.neighborIfaceAddr));
     }
   }
   if (senderEtx > m_etxThreshold)
   {
     return;
   }
 
   for (auto linkMessage = hello.linkMessages.begin(); linkMessage != hello.linkMessages.end();
        linkMessage++)
   {
     auto neighborType = NeighborType(linkMessage->linkCode >> 2);
     if (neighborType == NeighborType::MPR_NEIGH)
     {
       for (auto nb_iface_addr = linkMessage->neighborInterfaceAddresses.begin();
            nb_iface_addr != linkMessage->neighborInterfaceAddresses.end();
            nb_iface_addr++)
       {
         if (GetMainAddress(*nb_iface_addr) == m_mainAddress)
         {
           olsr::MprSelectorTuple* existing =
               m_state.FindMprSelectorTuple(msg.GetOriginatorAddress());
           if (existing == nullptr)
           {
             olsr::MprSelectorTuple mprsel_tuple;
             mprsel_tuple.mainAddr = msg.GetOriginatorAddress();
             mprsel_tuple.expirationTime = now + msg.GetVTime();
             AddMprSelectorTuple(mprsel_tuple);
             m_events.Track(Simulator::Schedule(DELAY(mprsel_tuple.expirationTime),
                                                &RoutingProtocol::MprSelTupleTimerExpire,
                                                this,
                                                mprsel_tuple.mainAddr));
           }
           else
           {
             existing->expirationTime = now + msg.GetVTime();
           }
         }
       }
     }
   }
 }
 
 void
 RoutingProtocol::NeighborLoss(const olsr::LinkTuple& tuple)
 {
   LinkLayerFeedbackHandler(tuple.neighborIfaceAddr, false);
   NS_LOG_DEBUG(Simulator::Now().As(Time::S) << ": ETX-OLSR Node " << m_mainAddress
                                             << " LinkTuple " << tuple.neighborIfaceAddr
                                             << " -> neighbor loss.");
   LinkTupleUpdated(tuple, olsr::Willingness::DEFAULT);
   m_state.EraseTwoHopNeighborTuples(GetMainAddress(tuple.neighborIfaceAddr));
   m_state.EraseMprSelectorTuples(GetMainAddress(tuple.neighborIfaceAddr));
 
   MprComputation();
   RoutingTableComputation();
 }
 
 void RoutingProtocol::AddDuplicateTuple(const olsr::DuplicateTuple& t) { m_state.InsertDuplicateTuple(t); }
 void RoutingProtocol::RemoveDuplicateTuple(const olsr::DuplicateTuple& t) { m_state.EraseDuplicateTuple(t); }
 
 void
 RoutingProtocol::LinkTupleAdded(const olsr::LinkTuple& tuple, olsr::Willingness willingness)
 {
   olsr::NeighborTuple nb_tuple;
   nb_tuple.neighborMainAddr = GetMainAddress(tuple.neighborIfaceAddr);
   nb_tuple.willingness = willingness;
   nb_tuple.status = (tuple.symTime >= Simulator::Now())
                         ? olsr::NeighborTuple::STATUS_SYM
                         : olsr::NeighborTuple::STATUS_NOT_SYM;
   AddNeighborTuple(nb_tuple);
 }
 
 void
 RoutingProtocol::RemoveLinkTuple(const olsr::LinkTuple& tuple)
 {
   m_state.EraseNeighborTuple(GetMainAddress(tuple.neighborIfaceAddr));
   m_state.EraseLinkTuple(tuple);
 }
 
 void
 RoutingProtocol::LinkTupleUpdated(const olsr::LinkTuple& tuple, olsr::Willingness willingness)
 {
   olsr::NeighborTuple* nb_tuple =
       m_state.FindNeighborTuple(GetMainAddress(tuple.neighborIfaceAddr));
 
   if (nb_tuple == nullptr)
   {
     LinkTupleAdded(tuple, willingness);
     nb_tuple = m_state.FindNeighborTuple(GetMainAddress(tuple.neighborIfaceAddr));
   }
 
   if (nb_tuple != nullptr)
   {
     bool hasSymmetricLink = false;
     const olsr::LinkSet& linkSet = m_state.GetLinks();
     for (auto lt = linkSet.begin(); lt != linkSet.end(); lt++)
     {
       if (GetMainAddress(lt->neighborIfaceAddr) == nb_tuple->neighborMainAddr &&
           lt->symTime >= Simulator::Now())
       {
         hasSymmetricLink = true;
         break;
       }
     }
     nb_tuple->status = hasSymmetricLink ? olsr::NeighborTuple::STATUS_SYM
                                         : olsr::NeighborTuple::STATUS_NOT_SYM;
   }
 }
 
 void RoutingProtocol::AddNeighborTuple(const olsr::NeighborTuple& t) { m_state.InsertNeighborTuple(t); IncrementAnsn(); }
 void RoutingProtocol::RemoveNeighborTuple(const olsr::NeighborTuple& t) { m_state.EraseNeighborTuple(t); IncrementAnsn(); }
 void RoutingProtocol::AddTwoHopNeighborTuple(const olsr::TwoHopNeighborTuple& t) { m_state.InsertTwoHopNeighborTuple(t); }
 void RoutingProtocol::RemoveTwoHopNeighborTuple(const olsr::TwoHopNeighborTuple& t) { m_state.EraseTwoHopNeighborTuple(t); }
 void RoutingProtocol::IncrementAnsn() { m_ansn = (m_ansn + 1) % (OLSR_MAX_SEQ_NUM + 1); }
 void RoutingProtocol::AddMprSelectorTuple(const olsr::MprSelectorTuple& t) { m_state.InsertMprSelectorTuple(t); IncrementAnsn(); }
 void RoutingProtocol::RemoveMprSelectorTuple(const olsr::MprSelectorTuple& t) { m_state.EraseMprSelectorTuple(t); IncrementAnsn(); }
 void RoutingProtocol::AddTopologyTuple(const olsr::TopologyTuple& t) { m_state.InsertTopologyTuple(t); }
 void RoutingProtocol::RemoveTopologyTuple(const olsr::TopologyTuple& t) { m_state.EraseTopologyTuple(t); }
 void RoutingProtocol::AddIfaceAssocTuple(const olsr::IfaceAssocTuple& t) { m_state.InsertIfaceAssocTuple(t); }
 void RoutingProtocol::RemoveIfaceAssocTuple(const olsr::IfaceAssocTuple& t) { m_state.EraseIfaceAssocTuple(t); }
 void RoutingProtocol::AddAssociationTuple(const olsr::AssociationTuple& t) { m_state.InsertAssociationTuple(t); }
 void RoutingProtocol::RemoveAssociationTuple(const olsr::AssociationTuple& t) { m_state.EraseAssociationTuple(t); }
 
 uint16_t
 RoutingProtocol::GetPacketSequenceNumber()
 {
   m_packetSequenceNumber = (m_packetSequenceNumber + 1) % (OLSR_MAX_SEQ_NUM + 1);
   return m_packetSequenceNumber;
 }
 
 uint16_t
 RoutingProtocol::GetMessageSequenceNumber()
 {
   m_messageSequenceNumber = (m_messageSequenceNumber + 1) % (OLSR_MAX_SEQ_NUM + 1);
   return m_messageSequenceNumber;
 }
 
void
RoutingProtocol::HelloTimerExpire()
{
  if (!m_ipv4)
  {
    return;
  }
  SendHello();
  m_helloTimer.Schedule(m_helloInterval);
}
 
 void
 RoutingProtocol::TcTimerExpire()
 {
  if (!m_ipv4)
  {
    return;
  }
   if (!m_state.GetMprSelectors().empty())
   {
     SendTc();
   }
   m_tcTimer.Schedule(m_tcInterval);
 }
 
void
RoutingProtocol::MidTimerExpire()
{
  if (!m_ipv4)
  {
    return;
  }
  SendMid();
  m_midTimer.Schedule(m_midInterval);
}
 
 void
 RoutingProtocol::HnaTimerExpire()
 {
  if (!m_ipv4)
  {
    return;
  }
   if (!m_state.GetAssociations().empty())
   {
     SendHna();
   }
   m_hnaTimer.Schedule(m_hnaInterval);
 }
 
 void
 RoutingProtocol::DupTupleTimerExpire(Ipv4Address address, uint16_t sequenceNumber)
 {
   olsr::DuplicateTuple* tuple = m_state.FindDuplicateTuple(address, sequenceNumber);
   if (tuple == nullptr)
   {
     return;
   }
   if (tuple->expirationTime < Simulator::Now())
   {
     RemoveDuplicateTuple(*tuple);
   }
   else
   {
     m_events.Track(Simulator::Schedule(DELAY(tuple->expirationTime),
                                        &RoutingProtocol::DupTupleTimerExpire,
                                        this,
                                        address,
                                        sequenceNumber));
   }
 }
 
 void
 RoutingProtocol::LinkTupleTimerExpire(Ipv4Address neighborIfaceAddr)
 {
   Time now = Simulator::Now();
   olsr::LinkTuple* tuple = m_state.FindLinkTuple(neighborIfaceAddr);
   if (tuple == nullptr)
   {
     return;
   }
   if (tuple->time < now)
   {
     ClearNeighborTelemetry(neighborIfaceAddr);
     RemoveLinkTuple(*tuple);
   }
   else if (tuple->symTime < now)
   {
     if (m_linkTupleTimerFirstTime)
     {
       m_linkTupleTimerFirstTime = false;
     }
     else
     {
       NeighborLoss(*tuple);
     }
     m_events.Track(Simulator::Schedule(DELAY(tuple->time),
                                        &RoutingProtocol::LinkTupleTimerExpire,
                                        this,
                                        neighborIfaceAddr));
   }
   else
   {
     m_events.Track(
         Simulator::Schedule(DELAY(std::min(tuple->time, tuple->symTime)),
                             &RoutingProtocol::LinkTupleTimerExpire,
                             this,
                             neighborIfaceAddr));
   }
 }
 
 void
 RoutingProtocol::Nb2hopTupleTimerExpire(Ipv4Address neighborMainAddr,
                                         Ipv4Address twoHopNeighborAddr)
 {
   olsr::TwoHopNeighborTuple* tuple =
       m_state.FindTwoHopNeighborTuple(neighborMainAddr, twoHopNeighborAddr);
   if (tuple == nullptr)
   {
     return;
   }
   if (tuple->expirationTime < Simulator::Now())
   {
     RemoveTwoHopNeighborTuple(*tuple);
   }
   else
   {
     m_events.Track(Simulator::Schedule(DELAY(tuple->expirationTime),
                                        &RoutingProtocol::Nb2hopTupleTimerExpire,
                                        this,
                                        neighborMainAddr,
                                        twoHopNeighborAddr));
   }
 }
 
 void
 RoutingProtocol::MprSelTupleTimerExpire(Ipv4Address mainAddr)
 {
   olsr::MprSelectorTuple* tuple = m_state.FindMprSelectorTuple(mainAddr);
   if (tuple == nullptr)
   {
     return;
   }
   if (tuple->expirationTime < Simulator::Now())
   {
     RemoveMprSelectorTuple(*tuple);
   }
   else
   {
     m_events.Track(Simulator::Schedule(DELAY(tuple->expirationTime),
                                        &RoutingProtocol::MprSelTupleTimerExpire,
                                        this,
                                        mainAddr));
   }
 }
 
 void
 RoutingProtocol::TopologyTupleTimerExpire(Ipv4Address destAddr, Ipv4Address lastAddr)
 {
   olsr::TopologyTuple* tuple = m_state.FindTopologyTuple(destAddr, lastAddr);
   if (tuple == nullptr)
   {
     return;
   }
   if (tuple->expirationTime < Simulator::Now())
   {
     RemoveTopologyTuple(*tuple);
   }
   else
   {
     m_events.Track(Simulator::Schedule(DELAY(tuple->expirationTime),
                                        &RoutingProtocol::TopologyTupleTimerExpire,
                                        this,
                                        tuple->destAddr,
                                        tuple->lastAddr));
   }
 }
 
 void
 RoutingProtocol::IfaceAssocTupleTimerExpire(Ipv4Address ifaceAddr)
 {
   olsr::IfaceAssocTuple* tuple = m_state.FindIfaceAssocTuple(ifaceAddr);
   if (tuple == nullptr)
   {
     return;
   }
   if (tuple->time < Simulator::Now())
   {
     RemoveIfaceAssocTuple(*tuple);
   }
   else
   {
     m_events.Track(Simulator::Schedule(DELAY(tuple->time),
                                        &RoutingProtocol::IfaceAssocTupleTimerExpire,
                                        this,
                                        ifaceAddr));
   }
 }
 
 void
 RoutingProtocol::AssociationTupleTimerExpire(Ipv4Address gatewayAddr,
                                             Ipv4Address networkAddr,
                                             Ipv4Mask netmask)
 {
   olsr::AssociationTuple* tuple =
       m_state.FindAssociationTuple(gatewayAddr, networkAddr, netmask);
   if (tuple == nullptr)
   {
     return;
   }
   if (tuple->expirationTime < Simulator::Now())
   {
     RemoveAssociationTuple(*tuple);
   }
   else
   {
     m_events.Track(Simulator::Schedule(DELAY(tuple->expirationTime),
                                        &RoutingProtocol::AssociationTupleTimerExpire,
                                        this,
                                        gatewayAddr,
                                        networkAddr,
                                        netmask));
   }
 }
 
 void
 RoutingProtocol::Clear()
 {
   m_table.clear();
 }
 
 void
 RoutingProtocol::RemoveEntry(const Ipv4Address& dest)
 {
   m_table.erase(dest);
 }
 
 bool
 RoutingProtocol::Lookup(const Ipv4Address& dest, RoutingTableEntry& outEntry) const
 {
   auto it = m_table.find(dest);
   if (it == m_table.end())
   {
     return false;
   }
   outEntry = it->second;
   return true;
 }
 
 bool
 RoutingProtocol::FindSendEntry(const RoutingTableEntry& entry, RoutingTableEntry& outEntry) const
 {
   outEntry = entry;
   while (outEntry.destAddr != outEntry.nextAddr)
   {
     if (!Lookup(outEntry.nextAddr, outEntry))
     {
       return false;
     }
   }
   return true;
 }
 
 bool
 RoutingProtocol::ChooseCandidate(const Ipv4Address& dest, CandidateRoute& out, uint32_t& pathIndex)
 {
   auto it = m_candidateTable.find(dest);
   if (it == m_candidateTable.end())
   {
     return false;
   }
 
   const CandidateSet& cset = it->second;
 
   // Helper: true iff next-hop nh has a live symmetric link.
   auto isAvailable = [this](const Ipv4Address& nh) -> bool {
     if (nh == Ipv4Address())
     {
       return false;
     }
 
     Time now = Simulator::Now();
 
     for (const auto& nb : m_state.GetNeighbors())
     {
       if (nb.status == olsr::NeighborTuple::STATUS_SYM &&
           GetMainAddress(nh) == nb.neighborMainAddr)
       {
         return true;
       }
     }
 
     for (const auto& link : m_state.GetLinks())
     {
       if (GetMainAddress(link.neighborIfaceAddr) == GetMainAddress(nh) &&
           link.symTime >= now)
       {
         return true;
       }
     }
 
     return false;
   };
 
   const double negInf = -std::numeric_limits<double>::infinity();
   double bestScore = negInf;
   int bestK = -1;
 
   uint64_t t = m_mabTotalDecisions + 1;
   double logT = std::log(static_cast<double>(t));
 
   for (int k = 0; k < static_cast<int>(cset.size()); k++)
   {
     const CandidateRoute& cand = cset[k];
     if (!isAvailable(cand.nextHop))
     {
       continue;
     }
 
     auto armIt = m_mabArms.find(cand.nextHop);
 
     double ucbScore;
     if (armIt == m_mabArms.end() || armIt->second.totalCount == 0)
     {
       // N=0 -> +inf
       ucbScore = std::numeric_limits<double>::infinity();
     }
     else
     {
      const MabArm& arm = armIt->second;
       const double exploration =
           (m_mabC > 0.0) ? m_mabC * std::sqrt(logT / static_cast<double>(arm.totalCount)) : 0.0;
       const double trend = ComputeTrendPenalty(arm);
       const double sigma = std::sqrt(std::max(0.0, m_sigma[cand.nextHop]));
      double switchPenalty = 0.0;
      auto prevIt = m_lastChosenNextHop.find(dest);
      if (prevIt != m_lastChosenNextHop.end() && prevIt->second != cand.nextHop)
      {
        switchPenalty = m_switchPenalty;
      }
      ucbScore = arm.meanReward + exploration - m_mabGamma * trend - sigma - switchPenalty;
     }
 
     if (ucbScore > bestScore)
     {
       bestScore = ucbScore;
       bestK = k;
     }
   }
 
   if (bestK >= 0)
   {
     out = cset[bestK];
     pathIndex = static_cast<uint32_t>(bestK);
     m_mabTotalDecisions++;
    m_totalPathSelections++;

    NS_LOG_UNCOND("PATH_SELECT dest=" << dest
                                      << " pathId=" << bestK
                                      << " nextHop=" << out.nextHop
                                      << " ctrlCost=" << out.ctrlCost
                                      << " score=" << bestScore
                                      << " reason=UCB"
                                      << " time=" << Simulator::Now().GetSeconds());
 
     auto prevIt = m_lastChosenNextHop.find(dest);
     if (prevIt != m_lastChosenNextHop.end() && prevIt->second != out.nextHop)
     {
       m_switchEvents.push_back(Simulator::Now());
      m_totalPathSwitches++;
      m_pathSwitchCountByDest[dest]++;
      NS_LOG_UNCOND("MAB_SWITCH dest=" << dest
                                       << " prevNextHop=" << prevIt->second
                                       << " newNextHop=" << out.nextHop
                                       << " pathId=" << bestK
                                       << " time=" << Simulator::Now().GetSeconds());
     }
     m_lastChosenNextHop[dest] = out.nextHop;
     while (!m_switchEvents.empty() &&
            (Simulator::Now() - m_switchEvents.front()).GetSeconds() > 1.0)
     {
       m_switchEvents.pop_front();
     }
     const double switchPerSec = static_cast<double>(m_switchEvents.size());
 
     NS_LOG_DEBUG("ChooseCandidate (UCB): dest=" << dest
                  << " k=" << bestK
                  << " nextHop=" << out.nextHop
                  << " ucbScore=" << bestScore
                  << " switchPerSec=" << switchPerSec
                  << " t=" << m_mabTotalDecisions);
     return true;
   }
 
   NS_LOG_DEBUG("ChooseCandidate: dest=" << dest << " no candidate available");
   return false;
 }
 
 std::set<Ipv4Address>
 RoutingProtocol::BuildPathNodesForNextHop(const Ipv4Address& dest, const Ipv4Address& firstHop) const
 {
   std::set<Ipv4Address> nodes;
   if (firstHop == Ipv4Address())
   {
     return nodes;
   }
   nodes.insert(dest);
   static constexpr int MAX_PATH_TRACE_HOPS = 32;
   Ipv4Address cur = dest;
   for (int limit = 0; limit < MAX_PATH_TRACE_HOPS; limit++)
   {
     if (cur == firstHop)
     {
       break;
     }
 
     const olsr::TopologySet& topology = m_state.GetTopologySet();
     bool advanced = false;
     for (const auto& topo : topology)
     {
       if (topo.destAddr != cur)
       {
         continue;
       }
       RoutingTableEntry le;
       if (!Lookup(topo.lastAddr, le))
       {
         continue;
       }
       if (le.nextAddr == firstHop)
       {
         nodes.insert(topo.lastAddr);
         cur = topo.lastAddr;
         advanced = true;
         break;
       }
     }
     if (!advanced)
     {
       break;
     }
   }
   return nodes;
 }
 
 std::vector<Ipv4Address>
 RoutingProtocol::BuildPathSequenceForNextHop(const Ipv4Address& dest, const Ipv4Address& firstHop) const
 {
   std::vector<Ipv4Address> sequence;
   if (firstHop == Ipv4Address())
   {
     return sequence;
   }
 
   sequence.push_back(firstHop);
   if (dest == firstHop)
   {
     return sequence;
   }
 
   static constexpr int MAX_PATH_TRACE_HOPS = 32;
   Ipv4Address cur = dest;
   std::vector<Ipv4Address> reverseTail;
   reverseTail.push_back(dest);
   for (int limit = 0; limit < MAX_PATH_TRACE_HOPS; ++limit)
   {
     if (cur == firstHop)
     {
       break;
     }
 
     bool advanced = false;
     for (const auto& topo : m_state.GetTopologySet())
     {
       if (topo.destAddr != cur)
       {
         continue;
       }
       RoutingTableEntry le;
       if (!Lookup(topo.lastAddr, le))
       {
         continue;
       }
       if (le.nextAddr == firstHop)
       {
         cur = topo.lastAddr;
         reverseTail.push_back(cur);
         advanced = true;
         break;
       }
     }
     if (!advanced)
     {
       break;
     }
   }
 
   for (auto it = reverseTail.rbegin(); it != reverseTail.rend(); ++it)
   {
     if (*it != firstHop)
     {
       sequence.push_back(*it);
     }
   }
   return sequence;
 }
 
 double
 RoutingProtocol::ComputeTrendPenalty(const MabArm& arm) const
 {
   if (arm.rewardHistory.size() < 3)
   {
     return 0.0;
   }
   const std::size_t n = arm.rewardHistory.size();
   const double trend = std::max(0.0, (arm.rewardHistory[n - 3] - arm.rewardHistory[n - 1]) / 2.0);
   return (trend > 0.1) ? trend : 0.0;
 }
 
 double
 RoutingProtocol::ComputeSinrRisk(const Ipv4Address& nextHop) const
 {
   auto it = m_linkState.find(nextHop);
   if (it == m_linkState.end() || !it->second.hasSinr)
   {
     return 0.0;
   }
   if (it->second.smoothSinrDb < m_sinrThresholdDb)
   {
     return std::numeric_limits<double>::infinity();
   }
   return std::max(0.0, (m_sinrThresholdDb + 20.0 - it->second.smoothSinrDb) / 20.0);
 }
 
 void
 RoutingProtocol::ResetArmForNextHop(const Ipv4Address& nextHop)
 {
   MabArm& arm = m_mabArms[nextHop];
   arm.totalCount = 1;
   arm.meanReward = 0.0;
   arm.smoothedDelayMs = m_delayMaxMs;
   arm.hasSmoothedDelay = false;
   arm.rewardHistory.clear();
   m_sigma[nextHop] = 0.0;
 }
 
 void
 RoutingProtocol::UpdateMabModel(const Ipv4Address& nextHop,
                                 bool isSuccess,
                                 double etxValue,
                                 double rawDelayMs,
                                 double sinrDb)
 {
   MabArm& arm = m_mabArms[nextHop];
 
   if (!arm.hasSmoothedDelay)
   {
     arm.smoothedDelayMs = rawDelayMs;
     arm.hasSmoothedDelay = true;
   }
   else
   {
     arm.smoothedDelayMs =
         m_delayEwmaAlpha * arm.smoothedDelayMs + (1.0 - m_delayEwmaAlpha) * rawDelayMs;
   }
   double sinrDbForReward = std::numeric_limits<double>::quiet_NaN();
   if (std::isfinite(sinrDb))
   {
     auto& link = m_linkState[nextHop];
     if (!link.hasSinr)
     {
       link.smoothSinrDb = sinrDb;
       link.smoothRssiDbm = sinrDb;
       link.hasSinr = true;
     }
     else
     {
       link.smoothSinrDb = m_sinrAlpha * link.smoothSinrDb + (1.0 - m_sinrAlpha) * sinrDb;
     }
     sinrDbForReward = link.smoothSinrDb;
   }
   else
   {
     auto it = m_linkState.find(nextHop);
     if (it != m_linkState.end() && it->second.hasSinr)
     {
       sinrDbForReward = it->second.smoothSinrDb;
     }
   }
 
  const double successTerm = isSuccess ? 1.0 : 0.0;
  const double normDelay =
      std::clamp(1.0 - (arm.smoothedDelayMs / std::max(1.0, m_maxAllowedDelayMs)), 0.0, 1.0);
  const double etxClamped = std::clamp(etxValue, 1.0, std::max(1.1, m_rewardEtxMax));
  const double etxNorm = (etxClamped - 1.0) / std::max(0.1, (m_rewardEtxMax - 1.0));
  const double etxQuality = std::clamp(1.0 - etxNorm, 0.0, 1.0);
  const double k = std::max(0.1, m_sinrRiskSlopeDb);
  const double sinrRisk = std::isfinite(sinrDbForReward)
                              ? 1.0 / (1.0 + std::exp((sinrDbForReward - m_sinrRiskMidDb) / k))
                              : 1.0;
  const double normSinr = std::clamp(1.0 - sinrRisk, 0.0, 1.0);
 
   // Normalize weights to avoid "arbitrary tuning" criticism.
   double a = m_rewardAlpha;
   double b = m_rewardBeta;
   double g = m_rewardGamma;
   double d = m_rewardDeltaEtx;
   const double sum = a + b + g + d;
   if (sum > 0.0)
   {
     a /= sum;
     b /= sum;
     g /= sum;
     d /= sum;
   }
  const double reward = a * successTerm + b * normDelay + g * normSinr + d * etxQuality;
 
   arm.totalCount++;
   const double previousMean = arm.meanReward;
   arm.meanReward = (1.0 - m_rewardEwmaAlpha) * arm.meanReward + m_rewardEwmaAlpha * reward;
   const double error = reward - previousMean;
   const double previousSigma = m_sigma[nextHop];
   m_sigma[nextHop] =
       (1.0 - m_rewardEwmaAlpha) * previousSigma + m_rewardEwmaAlpha * error * error;
   arm.rewardHistory.push_back(reward);
   if (arm.rewardHistory.size() > 5)
   {
     arm.rewardHistory.pop_front();
   }
 }
 
 void
 RoutingProtocol::AddEntry(const Ipv4Address& dest,
                           const Ipv4Address& next,
                           uint32_t interface,
                           uint32_t distance,
                           double etxDist)
 {
   NS_LOG_FUNCTION(this << dest << next << interface << distance << etxDist << m_mainAddress);
   NS_ASSERT(distance > 0);
 
   RoutingTableEntry& entry = m_table[dest];
   entry.destAddr = dest;
   entry.nextAddr = next;
   entry.interface = interface;
   entry.distance = distance;
   entry.etxDistance = etxDist;
 }
 
 void
 RoutingProtocol::AddEntry(const Ipv4Address& dest,
                           const Ipv4Address& next,
                           const Ipv4Address& interfaceAddress,
                           uint32_t distance,
                           double etxDist)
 {
   NS_ASSERT(distance > 0);
   NS_ASSERT(m_ipv4);
 
   for (uint32_t i = 0; i < m_ipv4->GetNInterfaces(); i++)
   {
     for (uint32_t j = 0; j < m_ipv4->GetNAddresses(i); j++)
     {
       if (m_ipv4->GetAddress(i, j).GetLocal() == interfaceAddress)
       {
         AddEntry(dest, next, i, distance, etxDist);
         return;
       }
     }
   }
   NS_ASSERT(false);
 }
 
 Ptr<Ipv4Route>
 RoutingProtocol::RouteOutput(Ptr<Packet> p,
                              const Ipv4Header& header,
                              Ptr<NetDevice> oif,
                              Socket::SocketErrno& sockerr)
 {
   NS_LOG_FUNCTION(this << " " << m_ipv4->GetObject<Node>()->GetId() << " "
                        << header.GetDestination() << " " << oif);
   Ptr<Ipv4Route> rtentry;
   RoutingTableEntry entry1;
   RoutingTableEntry entry2;
   CandidateRoute chosen;
   uint32_t selectedPathIndex = 0;
   bool found = false;
   if (m_enableSmartPath)
   {
     auto ctIt = m_candidateTable.find(header.GetDestination());
     if (ctIt != m_candidateTable.end())
     {
       std::set<Ipv4Address> uniqueNextHops;
       for (const auto& cand : ctIt->second)
       {
         if (cand.nextHop != Ipv4Address())
         {
           uniqueNextHops.insert(cand.nextHop);
         }
       }
 
       if (uniqueNextHops.size() > 1)
       {
         const Ipv4Address smartHop = SmartPathSelect(header.GetDestination());
         if (smartHop != Ipv4Address::GetZero())
         {
           for (uint32_t k = 0; k < ctIt->second.size(); ++k)
           {
             if (ctIt->second[k].nextHop == smartHop)
             {
               chosen = ctIt->second[k];
               selectedPathIndex = k;
               found = true;
               break;
             }
           }
         }
       }
     }
   }
   if (found || ChooseCandidate(header.GetDestination(), chosen, selectedPathIndex))
   {
    const uint64_t flowKey = BuildFlowKey(header.GetSource(), header.GetDestination(), header.GetProtocol());
    auto flowPrevIt = m_lastChosenPathByFlow.find(flowKey);
    if (flowPrevIt == m_lastChosenPathByFlow.end())
    {
      m_lastChosenPathByFlow[flowKey] = selectedPathIndex;
    }
    else if (flowPrevIt->second != selectedPathIndex)
    {
      m_pathSwitchCountByFlow[flowKey]++;
      NS_LOG_UNCOND("PATH_SWITCH flowId=" << flowKey
                                          << " fromPath=" << flowPrevIt->second
                                          << " toPath=" << selectedPathIndex
                                          << " src=" << header.GetSource()
                                          << " dst=" << header.GetDestination()
                                          << " time=" << Simulator::Now().GetSeconds());
      flowPrevIt->second = selectedPathIndex;
    }
    NS_LOG_UNCOND("PATH_SELECT_FLOW flowId=" << flowKey
                                             << " src=" << header.GetSource()
                                             << " dst=" << header.GetDestination()
                                             << " pathId=" << selectedPathIndex
                                             << " nextHop=" << chosen.nextHop
                                             << " time=" << Simulator::Now().GetSeconds());

     uint32_t interfaceIdx = chosen.interface;
     if (interfaceIdx >= m_ipv4->GetNInterfaces())
     {
       if (Lookup(header.GetDestination(), entry1))
       {
         bool foundSendEntry = FindSendEntry(entry1, entry2);
         if (!foundSendEntry)
         {
           NS_FATAL_ERROR("FindSendEntry failure");
         }
         interfaceIdx = entry2.interface;
         chosen.nextHop = entry2.nextAddr;
       }
       else
       {
         sockerr = Socket::ERROR_NOROUTETOHOST;
         return rtentry;
       }
     }
 
     if (oif && m_ipv4->GetInterfaceForDevice(oif) != static_cast<int>(interfaceIdx))
     {
       sockerr = Socket::ERROR_NOROUTETOHOST;
       return rtentry;
     }
 
     rtentry = Create<Ipv4Route>();
     rtentry->SetDestination(header.GetDestination());
 
     NS_ASSERT(m_ipv4);
     uint32_t numOifAddresses = m_ipv4->GetNAddresses(interfaceIdx);
     NS_ASSERT(numOifAddresses > 0);
     Ipv4InterfaceAddress ifAddr;
     if (numOifAddresses == 1)
     {
       ifAddr = m_ipv4->GetAddress(interfaceIdx, 0);
     }
     else
     {
       NS_FATAL_ERROR("XXX Not implemented yet: IP aliasing and ETX-OLSR");
     }
 
     rtentry->SetSource(ifAddr.GetLocal());
     rtentry->SetGateway(chosen.nextHop);
     rtentry->SetOutputDevice(m_ipv4->GetNetDevice(interfaceIdx));
     if (!FastLinkFailureDetection(chosen.nextHop, true))
     {
       sockerr = Socket::ERROR_NOROUTETOHOST;
       return nullptr;
     }
     if (p)
     {
       MyRouteTag routeTag;
       routeTag.SetPathId(selectedPathIndex);
       routeTag.SetTimestamp(Simulator::Now());
       p->ReplacePacketTag(routeTag);
     }
     sockerr = Socket::ERROR_NOTERROR;
     found = true;
   }
   else if (Lookup(header.GetDestination(), entry1))
   {
     bool foundSendEntry = FindSendEntry(entry1, entry2);
     if (!foundSendEntry)
     {
       NS_FATAL_ERROR("FindSendEntry failure");
     }
     uint32_t interfaceIdx = entry2.interface;
     if (oif && m_ipv4->GetInterfaceForDevice(oif) != static_cast<int>(interfaceIdx))
     {
       sockerr = Socket::ERROR_NOROUTETOHOST;
       return rtentry;
     }
     rtentry = Create<Ipv4Route>();
     rtentry->SetDestination(header.GetDestination());
 
     NS_ASSERT(m_ipv4);
     uint32_t numOifAddresses = m_ipv4->GetNAddresses(interfaceIdx);
     NS_ASSERT(numOifAddresses > 0);
     Ipv4InterfaceAddress ifAddr;
     if (numOifAddresses == 1)
     {
       ifAddr = m_ipv4->GetAddress(interfaceIdx, 0);
     }
     else
     {
       NS_FATAL_ERROR("XXX Not implemented yet: IP aliasing and ETX-OLSR");
     }
 
     rtentry->SetSource(ifAddr.GetLocal());
     rtentry->SetGateway(entry2.nextAddr);
     rtentry->SetOutputDevice(m_ipv4->GetNetDevice(interfaceIdx));
     if (!FastLinkFailureDetection(entry2.nextAddr, true))
     {
       sockerr = Socket::ERROR_NOROUTETOHOST;
       return nullptr;
     }
     sockerr = Socket::ERROR_NOTERROR;
     found = true;
   }
   else
   {
     rtentry = m_hnaRoutingTable->RouteOutput(p, header, oif, sockerr);
     if (rtentry)
     {
       found = true;
     }
   }
 
   if (!found)
   {
     sockerr = Socket::ERROR_NOROUTETOHOST;
   }
   return rtentry;
 }
 
 bool
 RoutingProtocol::RouteInput(Ptr<const Packet> p,
                             const Ipv4Header& header,
                             Ptr<const NetDevice> idev,
                             const UnicastForwardCallback& ucb,
                             const MulticastForwardCallback& mcb,
                             const LocalDeliverCallback& lcb,
                             const ErrorCallback& ecb)
 {
   NS_LOG_FUNCTION(this << " " << m_ipv4->GetObject<Node>()->GetId() << " "
                        << header.GetDestination());
 
   Ipv4Address dst = header.GetDestination();
   Ipv4Address origin = header.GetSource();
 
   if (IsMyOwnAddress(origin))
   {
     return true;
   }
 
   NS_ASSERT(m_ipv4->GetInterfaceForDevice(idev) >= 0);
   uint32_t iif = m_ipv4->GetInterfaceForDevice(idev);
   if (m_ipv4->IsDestinationAddress(dst, iif))
   {
     if (!lcb.IsNull())
     {
       lcb(p, header, iif);
       return true;
     }
     else
     {
       return false;
     }
   }
 
   Ptr<Ipv4Route> rtentry;
   RoutingTableEntry entry1;
   RoutingTableEntry entry2;
   double measuredDelayMs = 0.0;
  bool hasPathTag = false;
  uint32_t taggedPathIndex = 0;
   MyRouteTag inTag;
   if (p && p->PeekPacketTag(inTag))
   {
    hasPathTag = true;
    taggedPathIndex = inTag.GetPathId();
     measuredDelayMs = (Simulator::Now() - inTag.GetTimestamp()).GetMilliSeconds();
   }
 
  if (hasPathTag)
  {
    auto ctIt = m_candidateTable.find(dst);
    if (ctIt != m_candidateTable.end() && taggedPathIndex < ctIt->second.size())
    {
      const CandidateRoute& taggedCandidate = ctIt->second[taggedPathIndex];
      if (taggedCandidate.nextHop != Ipv4Address() &&
          taggedCandidate.interface < m_ipv4->GetNInterfaces())
      {
        NS_LOG_DEBUG("RouteInput: follow PathId=" << taggedPathIndex
                                                  << " dest=" << dst
                                                  << " via nextHop=" << taggedCandidate.nextHop);
        rtentry = Create<Ipv4Route>();
        rtentry->SetDestination(dst);

        const uint32_t interfaceIdx = taggedCandidate.interface;
        NS_ASSERT(m_ipv4);
        uint32_t numOifAddresses = m_ipv4->GetNAddresses(interfaceIdx);
        NS_ASSERT(numOifAddresses > 0);
        Ipv4InterfaceAddress ifAddr;
        if (numOifAddresses == 1)
        {
          ifAddr = m_ipv4->GetAddress(interfaceIdx, 0);
        }
        else
        {
          NS_FATAL_ERROR("XXX Not implemented yet: IP aliasing and ETX-OLSR");
        }

        rtentry->SetSource(ifAddr.GetLocal());
        rtentry->SetGateway(taggedCandidate.nextHop);
        rtentry->SetOutputDevice(m_ipv4->GetNetDevice(interfaceIdx));
        if (!FastLinkFailureDetection(taggedCandidate.nextHop, true))
        {
          LinkLayerFeedbackHandler(taggedCandidate.nextHop, false);
          return false;
        }

        Ptr<Packet> forwardedPacket = p->Copy();
        MyRouteTag outTag = inTag;
        if (measuredDelayMs <= 0.0)
        {
          outTag.SetTimestamp(Simulator::Now());
        }
        forwardedPacket->ReplacePacketTag(outTag);
        ucb(rtentry, forwardedPacket, header);
        SendBufferedPackets(dst, rtentry);
        if (measuredDelayMs > 0.0)
        {
          TraceRTTUpdate(taggedCandidate.nextHop, MilliSeconds(measuredDelayMs));
          UpdateMabModel(taggedCandidate.nextHop,
                         true,
                         GetLinkEtx(taggedCandidate.nextHop),
                         measuredDelayMs,
                         std::numeric_limits<double>::quiet_NaN());
        }
        return true;
      }
    }
  }

   if (Lookup(header.GetDestination(), entry1))
   {
     bool foundSendEntry = FindSendEntry(entry1, entry2);
     if (!foundSendEntry)
     {
       NS_FATAL_ERROR("FindSendEntry failure");
     }
 
     uint32_t interfaceIdx = entry2.interface;
     rtentry = Create<Ipv4Route>();
     rtentry->SetDestination(header.GetDestination());
 
     NS_ASSERT(m_ipv4);
     uint32_t numOifAddresses = m_ipv4->GetNAddresses(interfaceIdx);
     NS_ASSERT(numOifAddresses > 0);
     Ipv4InterfaceAddress ifAddr;
     if (numOifAddresses == 1)
     {
       ifAddr = m_ipv4->GetAddress(interfaceIdx, 0);
     }
     else
     {
       NS_FATAL_ERROR("XXX Not implemented yet: IP aliasing and ETX-OLSR");
     }
 
     rtentry->SetSource(ifAddr.GetLocal());
     rtentry->SetGateway(entry2.nextAddr);
     rtentry->SetOutputDevice(m_ipv4->GetNetDevice(interfaceIdx));
     if (!FastLinkFailureDetection(entry2.nextAddr, true))
     {
       LinkLayerFeedbackHandler(entry2.nextAddr, false);
       return false;
     }
     Ptr<Packet> forwardedPacket = p->Copy();
     MyRouteTag outTag;
     outTag = inTag;
     if (measuredDelayMs <= 0.0)
     {
       outTag.SetTimestamp(Simulator::Now());
     }
     forwardedPacket->ReplacePacketTag(outTag);
     ucb(rtentry, forwardedPacket, header);
     SendBufferedPackets(header.GetDestination(), rtentry);
     if (measuredDelayMs > 0.0)
     {
       TraceRTTUpdate(entry2.nextAddr, MilliSeconds(measuredDelayMs));
       UpdateMabModel(entry2.nextAddr,
                      true,
                      GetLinkEtx(entry2.nextAddr),
                      measuredDelayMs,
                      std::numeric_limits<double>::quiet_NaN());
     }
     return true;
   }
   else
   {
     if (m_hnaRoutingTable->RouteInput(p, header, idev, ucb, mcb, lcb, ecb))
     {
       return true;
     }
   }
   if (EnqueueBufferedPacket(p, header, ucb, ecb))
   {
     NS_LOG_DEBUG("ETX-OLSR micro queue: buffered packet for destination " << header.GetDestination()
                                                                            << ", queue-size="
                                                                            << m_packetQueue.size());
     return true;
   }
   return false;
 }
 
 void
 RoutingProtocol::NotifyInterfaceUp(uint32_t interface)
 {
   if (!m_ipv4 || interface >= m_ipv4->GetNInterfaces())
   {
     return;
   }
 
   Ptr<NetDevice> device = m_ipv4->GetNetDevice(interface);
   Ptr<WifiNetDevice> wifiDevice = DynamicCast<WifiNetDevice>(device);
   if (!wifiDevice)
   {
     return;
   }
 
   Ptr<Node> hostNode = ResolveHostNode(this, device);
   if (!hostNode)
   {
     NS_LOG_WARN("NotifyInterfaceUp: cannot resolve Node for WiFi traces (interface " << interface
                                                                                     << ")");
     return;
   }
 
   SetupRttTracer(device, hostNode);
 
   Ptr<WifiPhy> phy = wifiDevice->GetPhy();
   if (!phy || m_wifiPhys.count(interface) > 0)
   {
     return;
   }
 
   m_wifiPhys[interface] = phy;
   std::ostringstream path;
  // IMPORTANT: Config paths use NetDevice indices (DeviceList/*), not Ipv4 interface indices.
  path << "/NodeList/" << hostNode->GetId() << "/DeviceList/" << device->GetIfIndex()
        << "/$ns3::WifiNetDevice/Phy/MonitorSnifferRx";
   Config::ConnectWithoutContext(path.str(),
                                 MakeCallback(&RoutingProtocol::NotifyMonitorSnifferRx, this));
 }
 
 void
 RoutingProtocol::SetupRttTracer(Ptr<NetDevice> device, Ptr<Node> hostNode)
 {
   if (!device || !hostNode)
   {
     return;
   }
 
   NS_LOG_INFO("RTT sensor armed on device " << device->GetIfIndex());
  // IMPORTANT: Config paths use NetDevice indices (DeviceList/*), not Ipv4 interface indices.
  const uint32_t devIndex = device->GetIfIndex();
  std::ostringstream path;
  path << "/NodeList/" << hostNode->GetId() << "/DeviceList/" << devIndex
       << "/$ns3::WifiNetDevice/Mac/MacTxDrop";
  Config::ConnectWithoutContext(path.str(),
                                MakeCallback(&RoutingProtocol::NotifyMacTxDrop, this));
   // ns-3.45 note:
   // Wifi MAC/PHY traces are good for observability but do not expose true per-neighbor RTT directly.
   // We therefore use packet timestamp feedback in RouteInput (TraceRTTUpdate) as the RTT-like signal.
 }
 
 void
 RoutingProtocol::NotifyInterfaceDown(uint32_t interface)
 {
   m_wifiPhys.erase(interface);
 }
 void RoutingProtocol::NotifyAddAddress(uint32_t, Ipv4InterfaceAddress) {}
 void RoutingProtocol::NotifyRemoveAddress(uint32_t, Ipv4InterfaceAddress) {}
 
 void
 RoutingProtocol::NotifyMonitorSnifferRx(Ptr<const Packet> packet,
                                         uint16_t,
                                         WifiTxVector,
                                         MpduInfo,
                                         SignalNoiseDbm signalNoise,
                                         uint16_t)
 {
   WifiMacHeader hdr;
   Ptr<Packet> copy = packet->Copy();
   if (!copy->PeekHeader(hdr))
   {
     return;
   }
 
   const Ipv4Address sender = ResolveIpv4FromMac(hdr.GetAddr2());
   if (sender == Ipv4Address())
   {
     return;
   }
 
   const double sinrDb = signalNoise.signal - signalNoise.noise;
   auto& state = m_linkState[sender];
   if (!state.hasSinr)
   {
     state.smoothSinrDb = sinrDb;
     state.smoothRssiDbm = signalNoise.signal;
     state.hasSinr = true;
     state.prevSinrDb = sinrDb;
     state.hasPrevSinr = true;
     state.smoothSinrDeltaDb = 0.0;
   }
   else
   {
     if (state.hasPrevSinr)
     {
       const double absDelta = std::abs(sinrDb - state.prevSinrDb);
       // Reuse m_sinrAlpha as the smoothing factor for SINR-delta observability.
       state.smoothSinrDeltaDb =
           m_sinrAlpha * state.smoothSinrDeltaDb + (1.0 - m_sinrAlpha) * absDelta;
     }
     state.prevSinrDb = sinrDb;
     state.hasPrevSinr = true;
     state.smoothSinrDb = m_sinrAlpha * state.smoothSinrDb + (1.0 - m_sinrAlpha) * sinrDb;
     state.smoothRssiDbm =
         m_sinrAlpha * state.smoothRssiDbm + (1.0 - m_sinrAlpha) * signalNoise.signal;
   }
 }
 
 void
 RoutingProtocol::TraceRTTUpdate(Ipv4Address neighAddr, Time newRtt)
 {
   auto it = m_neighRealTimeRtt.find(neighAddr);
   const Time oldRtt = (it == m_neighRealTimeRtt.end()) ? Time(0) : it->second;
   const bool firstSample = oldRtt.IsZero();
   const int64_t deltaUs = firstSample ? 0 : (newRtt.GetMicroSeconds() - oldRtt.GetMicroSeconds());
 
   m_neighRealTimeRtt[neighAddr] = newRtt;
   m_lastRttDelta[neighAddr] = deltaUs;
   if (deltaUs > 0)
   {
     m_positiveDeltaStreak[neighAddr]++;
   }
   else
   {
     m_positiveDeltaStreak[neighAddr] = 0;
   }
 
   if (firstSample)
   {
     NS_LOG_DEBUG("RTT sensor init neigh=" << neighAddr << " rtt=" << newRtt.GetMilliSeconds() << "ms");
     return;
   }
 
   if (deltaUs > 5000 && m_positiveDeltaStreak[neighAddr] >= 3)
   {
     NS_LOG_DEBUG("RTT sensor degradation neigh=" << neighAddr
                                                  << " old=" << oldRtt.GetMilliSeconds()
                                                  << "ms new=" << newRtt.GetMilliSeconds()
                                                  << "ms deltaUs=" << deltaUs);
   }
 }
 
 Ipv4Address
 RoutingProtocol::SmartPathSelect(Ipv4Address dest)
 {
   auto it = m_candidateTable.find(dest);
   if (it == m_candidateTable.end())
   {
     NS_LOG_DEBUG("SmartPathSelect: no candidate for " << dest);
     return Ipv4Address::GetZero();
   }
 
   double bestScore = std::numeric_limits<double>::infinity();
   Ipv4Address bestHop = Ipv4Address::GetZero();
   for (const auto& cand : it->second)
   {
     if (cand.nextHop != Ipv4Address())
     {
       const auto rttIt = m_neighRealTimeRtt.find(cand.nextHop);
       const double rttMs = (rttIt == m_neighRealTimeRtt.end()) ? 0.0 : rttIt->second.GetMilliSeconds();
 
       const auto deltaIt = m_lastRttDelta.find(cand.nextHop);
       const double deltaMs = (deltaIt == m_lastRttDelta.end()) ? 0.0 : (deltaIt->second / 1000.0);
 
       const double score = cand.ctrlCost + rttMs + 3.0 * std::max(0.0, deltaMs);
       if (score < bestScore)
       {
         bestScore = score;
         bestHop = cand.nextHop;
       }
     }
   }
 
   if (bestHop != Ipv4Address::GetZero())
   {
     NS_LOG_DEBUG("SmartPathSelect: dest=" << dest << " nextHop=" << bestHop << " score=" << bestScore);
   }
   return bestHop;
 }
 
 void
 RoutingProtocol::NotifyMacTxDrop(Ptr<const Packet> packet)
 {
   if (!packet)
   {
     return;
   }
 
   WifiMacHeader hdr;
   Ptr<Packet> copy = packet->Copy();
   if (!copy->PeekHeader(hdr))
   {
     return;
   }
 
   const Ipv4Address neigh = ResolveIpv4FromMac(hdr.GetAddr1());
   if (neigh == Ipv4Address())
   {
     return;
   }
 
   // A drop spike is treated as immediate degradation evidence.
   m_positiveDeltaStreak[neigh] = std::max(m_positiveDeltaStreak[neigh], 3u);
   m_macTxDropEvents[neigh].push_back(Simulator::Now());
   NS_LOG_DEBUG("MacTxDrop observed for neigh=" << neigh);
 }
 
 void
 RoutingProtocol::ClearNeighborTelemetry(Ipv4Address neighAddr)
 {
   m_neighRealTimeRtt.erase(neighAddr);
   m_lastRttDelta.erase(neighAddr);
   m_positiveDeltaStreak.erase(neighAddr);
   m_macTxDropEvents.erase(neighAddr);
 }
 
 void
 RoutingProtocol::PruneDropEvents(Ipv4Address neighAddr, double windowSec)
 {
   if (windowSec <= 0.0)
   {
     return;
   }
   auto it = m_macTxDropEvents.find(neighAddr);
   if (it == m_macTxDropEvents.end())
   {
     return;
   }
   auto& dq = it->second;
   const Time now = Simulator::Now();
   const Time win = Seconds(windowSec);
   while (!dq.empty() && (now - dq.front()) > win)
   {
     dq.pop_front();
   }
 }
 
 uint8_t
 RoutingProtocol::GetNeighborLabels(Ipv4Address neighAddr) const
 {
   uint8_t labels = LABEL_NONE;
   const Time now = Simulator::Now();
 
   // FAST: RTT below threshold (if available).
   auto rttIt = m_neighRealTimeRtt.find(neighAddr);
   if (rttIt != m_neighRealTimeRtt.end() && rttIt->second.IsStrictlyPositive() &&
       rttIt->second.GetMilliSeconds() <= m_fastRttMs)
   {
     labels |= LABEL_FAST;
   }
 
   // NOISY: low SINR, high SINR fluctuation, or recent MAC drops.
   bool noisy = false;
   auto lsIt = m_linkState.find(neighAddr);
   if (lsIt != m_linkState.end() && lsIt->second.hasSinr)
   {
     if (lsIt->second.smoothSinrDb < m_noisySinrDb)
     {
       noisy = true;
     }
   }
 
   // Recent drops are treated as "Noisy" evidence.
   auto dropIt = m_macTxDropEvents.find(neighAddr);
   bool hasRecentNoisyDrops = false;
   bool hasRecentStableDrops = false;
   if (dropIt != m_macTxDropEvents.end() && !dropIt->second.empty())
   {
     const Time lastDrop = dropIt->second.back();
     if (m_noisyDropWindowSec > 0.0 && (now - lastDrop) <= Seconds(m_noisyDropWindowSec))
     {
       hasRecentNoisyDrops = true;
     }
     if (m_stableWindowSec > 0.0 && (now - lastDrop) <= Seconds(m_stableWindowSec))
     {
       hasRecentStableDrops = true;
     }
   }
   if (hasRecentNoisyDrops)
   {
     noisy = true;
   }
 
   if (noisy)
   {
     labels |= LABEL_NOISY;
   }
 
   // STABLE: no drops recently + low SINR fluctuation (and not noisy).
   bool stable = false;
   if (!noisy)
   {
     double sinrDelta = 0.0;
     bool hasSinr = false;
     if (lsIt != m_linkState.end() && lsIt->second.hasSinr)
     {
       hasSinr = true;
       sinrDelta = lsIt->second.smoothSinrDeltaDb;
     }
     if (!hasRecentStableDrops && hasSinr && sinrDelta <= m_stableSinrDeltaDb)
     {
       stable = true;
     }
   }
   if (stable)
   {
     labels |= LABEL_STABLE;
   }
 
   return labels;
 }
 
 Ipv4Address
 RoutingProtocol::ResolveIpv4FromMac(const Address& mac) const
 {
   for (auto nodeIt = NodeList::Begin(); nodeIt != NodeList::End(); ++nodeIt)
   {
     Ptr<Node> node = *nodeIt;
     for (uint32_t i = 0; i < node->GetNDevices(); ++i)
     {
       Ptr<NetDevice> dev = node->GetDevice(i);
       if (dev->GetAddress() != mac)
       {
         continue;
       }
 
       Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
       if (!ipv4)
       {
         return Ipv4Address();
       }
 
       int32_t ifIndex = ipv4->GetInterfaceForDevice(dev);
       if (ifIndex < 0 || ipv4->GetNAddresses(static_cast<uint32_t>(ifIndex)) == 0)
       {
         return Ipv4Address();
       }
       return ipv4->GetAddress(static_cast<uint32_t>(ifIndex), 0).GetLocal();
     }
   }
   return Ipv4Address();
 }
 
 std::vector<RoutingTableEntry>
 RoutingProtocol::GetRoutingTableEntries() const
 {
   std::vector<RoutingTableEntry> retval;
   for (auto iter = m_table.begin(); iter != m_table.end(); iter++)
   {
     retval.push_back(iter->second);
   }
   return retval;
 }
 
 olsr::MprSet
 RoutingProtocol::GetMprSet() const
 {
   return m_state.GetMprSet();
 }
 
 const olsr::MprSelectorSet&
 RoutingProtocol::GetMprSelectors() const
 {
   return m_state.GetMprSelectors();
 }
 
 const olsr::NeighborSet&
 RoutingProtocol::GetNeighbors() const
 {
   return m_state.GetNeighbors();
 }
 
 const olsr::TwoHopNeighborSet&
 RoutingProtocol::GetTwoHopNeighbors() const
 {
   return m_state.GetTwoHopNeighbors();
 }
 
 const olsr::TopologySet&
 RoutingProtocol::GetTopologySet() const
 {
   return m_state.GetTopologySet();
 }
 
 const olsr::OlsrState&
 RoutingProtocol::GetOlsrState() const
 {
   return m_state;
 }
 
 int64_t
 RoutingProtocol::AssignStreams(int64_t stream)
 {
   m_uniformRandomVariable->SetStream(stream);
   return 1;
 }
 
 bool
 RoutingProtocol::IsMyOwnAddress(const Ipv4Address& a) const
 {
   for (auto j = m_sendSockets.begin(); j != m_sendSockets.end(); ++j)
   {
     if (a == j->second.GetLocal())
     {
       return true;
     }
   }
   return false;
 }
 
 void
 RoutingProtocol::Dump()
 {
 #ifdef NS3_LOG_ENABLE
   NS_LOG_DEBUG("ETX-OLSR Dump for node " << m_mainAddress);
   NS_LOG_DEBUG(" Neighbor set");
   for (auto iter = m_state.GetNeighbors().begin(); iter != m_state.GetNeighbors().end(); iter++)
   {
     NS_LOG_DEBUG("  " << *iter);
   }
   NS_LOG_DEBUG(" Routing table");
   for (auto iter = m_table.begin(); iter != m_table.end(); iter++)
   {
     NS_LOG_DEBUG("  dest=" << iter->first << " next=" << iter->second.nextAddr
                            << " hops=" << iter->second.distance
                            << " etx=" << iter->second.etxDistance);
   }
 #endif
 }
 
 Ptr<const Ipv4StaticRouting>
 RoutingProtocol::GetRoutingTableAssociation() const
 {
   return m_hnaRoutingTable;
 }
 
 void
 RoutingProtocol::CollectControlPlaneSignals(double& meanSigmaSqrt,
                                             double& meanDelayMs,
                                             double& pathSwitchRatePerSec,
                                             uint32_t& trackedArms) const
 {
   meanSigmaSqrt = 0.0;
   meanDelayMs = 0.0;
   pathSwitchRatePerSec = 0.0;
   trackedArms = 0;
 
   double sumSigmaSqrt = 0.0;
   double sumDelay = 0.0;
   for (const auto& kv : m_mabArms)
   {
     const MabArm& arm = kv.second;
     if (arm.totalCount == 0)
     {
       continue;
     }
     trackedArms++;
     auto sigIt = m_sigma.find(kv.first);
     const double var = (sigIt != m_sigma.end()) ? sigIt->second : 0.0;
     sumSigmaSqrt += std::sqrt(std::max(0.0, var));
     if (arm.hasSmoothedDelay)
     {
       sumDelay += arm.smoothedDelayMs;
     }
   }
   if (trackedArms > 0)
   {
     meanSigmaSqrt = sumSigmaSqrt / static_cast<double>(trackedArms);
     meanDelayMs = sumDelay / static_cast<double>(trackedArms);
   }
 
   const Time now = Simulator::Now();
   uint32_t switchesInWindow = 0;
   for (const Time& t : m_switchEvents)
   {
     if ((now - t).GetSeconds() <= 1.0)
     {
       switchesInWindow++;
     }
   }
   pathSwitchRatePerSec = static_cast<double>(switchesInWindow);
 }
 
 void
 RoutingProtocol::SetAdaptiveHyperparameters(double mabC, double switchPenalty)
 {
   constexpr double kMabCMin = 0.05;
   constexpr double kMabCMax = 10.0;
   constexpr double kSwitchPenMin = 0.0;
   constexpr double kSwitchPenMax = 10.0;
   m_mabC = std::min(kMabCMax, std::max(kMabCMin, mabC));
   m_switchPenalty = std::min(kSwitchPenMax, std::max(kSwitchPenMin, switchPenalty));
 }
 
 double
 RoutingProtocol::GetMabC() const
 {
   return m_mabC;
 }
 
 double
 RoutingProtocol::GetSwitchPenalty() const
 {
   return m_switchPenalty;
 }
 
 } // namespace etxolsr
 } // namespace ns3