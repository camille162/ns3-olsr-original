/*
 * ETX-OLSR: OLSR routing with ETX-based link metric.
 *
 * Based on the ns-3 OLSR module (src/olsr).
 * Copyright (c) 2004 Francisco J. Ros
 * Copyright (c) 2007 INESC Porto
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#define NS_LOG_APPEND_CONTEXT                                    \
  if (GetObject<Node>())                                         \
  {                                                              \
    std::clog << "[node " << GetObject<Node>()->GetId() << "] "; \
  }

#include "etx-olsr-routing-protocol.h"

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
#include "ns3/simulator.h"
#include "ns3/socket-factory.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>

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
                        TimeValue(Seconds(2)),
                        MakeTimeAccessor(&RoutingProtocol::m_helloInterval),
                        MakeTimeChecker())
          .AddAttribute("TcInterval",
                        "TC messages emission interval.",
                        TimeValue(Seconds(5)),
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
      m_beta(0.3),
      m_lambda(1.0)
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
RoutingProtocol::RecvOlsr(Ptr<Socket> socket)
{
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
  Ptr<NetDevice> dev = node->GetDevice(incomingIf);
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
}

int
RoutingProtocol::Degree(const olsr::NeighborTuple& tuple)
{
  int degree = 0;
  for (auto nb2hop_tuple = m_state.GetTwoHopNeighbors().begin();
       nb2hop_tuple != m_state.GetTwoHopNeighbors().end();
       nb2hop_tuple++)
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

      int r = 0;
      for (auto nb2hop = N2.begin(); nb2hop != N2.end(); nb2hop++)
      {
        if (nb2hop->neighborMainAddr == nb_tuple->neighborMainAddr)
        {
          r++;
        }
      }

      int d = Degree(*nb_tuple);

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

      double newEtx = lastAddrEntry.etxDistance + m_initialEtx;
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

  // ---------------- Phase-1 candidate table (K=2) ----------------
  m_candidateTable.clear();
  const olsr::TopologySet& topology = m_state.GetTopologySet();

  for (const auto& kv : m_table)
  {
    const Ipv4Address dest = kv.first;
    const RoutingTableEntry& primaryEntry = kv.second;

    CandidateSet cset;
    cset[0].nextHop = primaryEntry.nextAddr;
    cset[0].interface = primaryEntry.interface;
    cset[0].ctrlCost = primaryEntry.etxDistance;

    Ipv4Address backupNext;
    uint32_t backupIf = 0;
    double backupCost = std::numeric_limits<double>::infinity();

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

      Ipv4Address candNext = lastEntry.nextAddr;
      if (candNext == cset[0].nextHop)
      {
        continue;
      }

      double candCost = lastEntry.etxDistance + m_initialEtx;
      if (candCost < backupCost)
      {
        backupCost = candCost;
        backupNext = candNext;
        backupIf = lastEntry.interface;
      }
    }

    if (backupNext != Ipv4Address())
    {
      cset[1].nextHop = backupNext;
      cset[1].interface = backupIf;
      cset[1].ctrlCost = backupCost;
    }
    else
    {
      cset[1] = cset[0];
    }

    m_candidateTable[dest] = cset;

    NS_LOG_DEBUG("CandidateTable: dest=" << dest
                  << " primary=" << cset[0].nextHop
                  << " backup=" << cset[1].nextHop);
  }

  // ---- Risk-aware: update per-next-hop variance σ² (Control-Plane timescale) ----
  // For every candidate next-hop that appears in the table, compare the current
  // path-cost estimate (V̂) against the previously stored value.  The squared
  // prediction error drives an EWMA variance estimate:
  //   e  = ctrlCost_now - ctrlCost_prev
  //   σ² ← (1-β)·σ² + β·e²
  for (const auto& kv : m_candidateTable)
  {
    const CandidateSet& cset = kv.second;
    for (std::size_t k = 0; k < cset.size(); k++)
    {
      const CandidateRoute& cand = cset[k];
      if (cand.nextHop == Ipv4Address() || std::isinf(cand.ctrlCost))
      {
        continue;
      }

      auto prevIt = m_prevCandCost.find(cand.nextHop);
      // On the first observation for a next-hop, prevCost == ctrlCost, so error = 0
      // and σ² remains at its initial value of 0.  Variance warms up from the second
      // routing-table computation onwards (cold-start by design).
      double prevCost = (prevIt != m_prevCandCost.end()) ? prevIt->second : cand.ctrlCost;

      double error = cand.ctrlCost - prevCost;
      double prevSigma = 0.0;
      auto sigIt = m_sigma.find(cand.nextHop);
      if (sigIt != m_sigma.end())
      {
        prevSigma = sigIt->second;
      }
      double newSigma = (1.0 - m_beta) * prevSigma + m_beta * error * error;
      m_sigma[cand.nextHop] = newSigma;
      m_prevCandCost[cand.nextHop] = cand.ctrlCost;

      NS_LOG_DEBUG("Sigma update: nextHop=" << cand.nextHop
                   << " cost=" << cand.ctrlCost
                   << " error=" << error
                   << " sigma=" << std::sqrt(newSigma));
    }
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
  if (!m_queuedMessagesTimer.IsRunning())
  {
    m_queuedMessagesTimer.SetDelay(delay);
    m_queuedMessagesTimer.Schedule();
  }
}

void
RoutingProtocol::SendPacket(Ptr<Packet> packet, const olsr::MessageList& containedMessages)
{
  NS_LOG_DEBUG("ETX-OLSR node " << m_mainAddress << " sending a OLSR packet");

  olsr::PacketHeader header;
  header.SetPacketLength(header.GetSerializedSize() + packet->GetSize());
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
  Ptr<Packet> packet = Create<Packet>();
  int numMessages = 0;

  NS_LOG_DEBUG("ETX-OLSR node " << m_mainAddress << ": SendQueuedMessages");

  olsr::MessageList msglist;
  for (auto message = m_queuedMessages.begin(); message != m_queuedMessages.end(); message++)
  {
    Ptr<Packet> p = Create<Packet>();
    p->AddHeader(*message);
    packet->AddAtEnd(p);
    msglist.push_back(*message);
    if (++numMessages == OLSR_MAX_MSGS)
    {
      SendPacket(packet, msglist);
      msglist.clear();
      numMessages = 0;
      packet = Create<Packet>();
    }
  }

  if (packet->GetSize())
  {
    SendPacket(packet, msglist);
  }

  m_queuedMessages.clear();
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

void RoutingProtocol::HelloTimerExpire() { SendHello(); m_helloTimer.Schedule(m_helloInterval); }

void
RoutingProtocol::TcTimerExpire()
{
  if (!m_state.GetMprSelectors().empty())
  {
    SendTc();
  }
  m_tcTimer.Schedule(m_tcInterval);
}

void RoutingProtocol::MidTimerExpire() { SendMid(); m_midTimer.Schedule(m_midInterval); }

void
RoutingProtocol::HnaTimerExpire()
{
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
RoutingProtocol::ChooseCandidate(const Ipv4Address& dest, CandidateRoute& out) const
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

  // Risk-aware scoring (Data-Plane timescale):
  //   score[a] = V̂[a] + λ · σ[a]
  // where V̂[a] = ctrlCost and σ[a] = sqrt(m_sigma[a]).
  // Among all available candidates, select the one with the minimum score.
  const double inf = std::numeric_limits<double>::infinity();
  double bestScore = inf;
  int bestK = -1;

  for (int k = 0; k < static_cast<int>(cset.size()); k++)
  {
    const CandidateRoute& cand = cset[k];
    if (!isAvailable(cand.nextHop))
    {
      continue;
    }

    double sigma = 0.0;
    auto sigIt = m_sigma.find(cand.nextHop);
    if (sigIt != m_sigma.end())
    {
      sigma = std::sqrt(sigIt->second);
    }

    double score = cand.ctrlCost + m_lambda * sigma;
    if (score < bestScore)
    {
      bestScore = score;
      bestK = k;
    }
  }

  if (bestK >= 0)
  {
    out = cset[bestK];
    NS_LOG_DEBUG("ChooseCandidate: dest=" << dest
                 << " k=" << bestK
                 << " nextHop=" << out.nextHop
                 << " score=" << bestScore);
    return true;
  }

  NS_LOG_DEBUG("ChooseCandidate: dest=" << dest << " no candidate available");
  return false;
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
  bool found = false;

  if (ChooseCandidate(header.GetDestination(), chosen))
  {
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
  CandidateRoute chosen;

  if (ChooseCandidate(header.GetDestination(), chosen))
  {
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
        return false;
      }
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
    ucb(rtentry, p, header);
    return true;
  }
  else if (Lookup(header.GetDestination(), entry1))
  {
    bool foundSendEntry = FindSendEntry(entry1, entry2);
    if (!foundSendEntry)
    {
      NS_FATAL_ERROR("FindSendEntry failure");
    }

    rtentry = Create<Ipv4Route>();
    rtentry->SetDestination(header.GetDestination());

    uint32_t interfaceIdx = entry2.interface;
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

    ucb(rtentry, p, header);
    return true;
  }
  else
  {
    if (m_hnaRoutingTable->RouteInput(p, header, idev, ucb, mcb, lcb, ecb))
    {
      return true;
    }
  }
  return false;
}

void RoutingProtocol::NotifyInterfaceUp(uint32_t) {}
void RoutingProtocol::NotifyInterfaceDown(uint32_t) {}
void RoutingProtocol::NotifyAddAddress(uint32_t, Ipv4InterfaceAddress) {}
void RoutingProtocol::NotifyRemoveAddress(uint32_t, Ipv4InterfaceAddress) {}

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

} // namespace etxolsr
} // namespace ns3
