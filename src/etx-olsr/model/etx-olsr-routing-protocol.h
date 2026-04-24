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

#include <map>
#include <vector>

namespace ns3
{
namespace etxolsr
{

/**
 * @ingroup etx-olsr
 *
 * Routing table entry for ETX-OLSR.  Extends the standard OLSR entry with
 * a cumulative ETX path cost (etxDistance).  The hop-count distance is kept
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

/**
 * @ingroup etx-olsr
 *
 * @brief ETX-OLSR routing protocol for IPv4.
 *
 * This protocol reuses the standard OLSR control-plane (RFC 3626) messages
 * and state machine, but selects routes based on a cumulative ETX path cost
 * rather than hop count.
 *
 * ETX estimation:
 * - An EWMA of the per-link packet reception ratio (PRR) is maintained for
 *   each neighbor from which HELLO messages are observed.
 * - The EWMA is updated on every received HELLO with a sample of 1.  Missed
 *   HELLOs (estimated from the sender's hTime field) contribute samples of 0.
 * - ETX = 1 / PRR, clamped so PRR >= 0.01.
 *
 * Route selection:
 * - Routes are built using a Bellman-Ford scan over the topology table with
 *   cumulative ETX as the cost.
 * - Lower ETX wins; ties are broken by hop count.
 * - Links whose ETX is not directly observable use the InitialEtx attribute
 *   as the per-hop cost estimate.
 */
class RoutingProtocol : public Ipv4RoutingProtocol
{
  public:
    static const uint16_t OLSR_PORT_NUMBER; //!< OLSR port (698)

    /**
     * @brief Get the type ID.
     * @return The object TypeId.
     */
    static TypeId GetTypeId();

    RoutingProtocol();
    ~RoutingProtocol() override;

    /**
     * @brief Set the main interface address.
     * @param interface IPv4 interface index.
     */
    void SetMainInterface(uint32_t interface);

    /** Dump internal state to NS_LOG_DEBUG. */
    void Dump();

    /**
     * @return Copy of routing table entries.
     */
    std::vector<RoutingTableEntry> GetRoutingTableEntries() const;

    /**
     * Gets the MPR set.
     * @return MPR set.
     */
    olsr::MprSet GetMprSet() const;

    /** @return MPR selectors. */
    const olsr::MprSelectorSet& GetMprSelectors() const;

    /** @return 1-hop neighbor set. */
    const olsr::NeighborSet& GetNeighbors() const;

    /** @return 2-hop neighbor set. */
    const olsr::TwoHopNeighborSet& GetTwoHopNeighbors() const;

    /** @return Topology set. */
    const olsr::TopologySet& GetTopologySet() const;

    /** @return Internal OLSR state object. */
    const olsr::OlsrState& GetOlsrState() const;

    /**
     * Assign fixed random stream numbers.
     * @param stream First stream index.
     * @return Number of streams assigned.
     */
    int64_t AssignStreams(int64_t stream);

    /** TracedCallback signature for packet Tx/Rx events. */
    typedef void (*PacketTxRxTracedCallback)(const olsr::PacketHeader& header,
                                             const olsr::MessageList& messages);

    /** TracedCallback signature for routing table changes. */
    typedef void (*TableChangeTracedCallback)(uint32_t size);

  private:
    std::set<uint32_t> m_interfaceExclusions;
    Ptr<Ipv4StaticRouting> m_routingTableAssociation;

  public:
    /**
     * @return Set of excluded interfaces.
     */
    std::set<uint32_t> GetInterfaceExclusions() const
    {
        return m_interfaceExclusions;
    }

    /**
     * @param exceptions Set of interface indices to exclude from OLSR.
     */
    void SetInterfaceExclusions(std::set<uint32_t> exceptions);

    /**
     * Add a local HNA association.
     * @param networkAddr Network address.
     * @param netmask     Network mask.
     */
    void AddHostNetworkAssociation(Ipv4Address networkAddr, Ipv4Mask netmask);

    /**
     * Remove a local HNA association.
     * @param networkAddr Network address.
     * @param netmask     Network mask.
     */
    void RemoveHostNetworkAssociation(Ipv4Address networkAddr, Ipv4Mask netmask);

    /**
     * Associate a static routing table for HNA advertisement.
     * @param routingTable The static routing table.
     */
    void SetRoutingTableAssociation(Ptr<Ipv4StaticRouting> routingTable);

    /** @return The HNA routing table. */
    Ptr<const Ipv4StaticRouting> GetRoutingTableAssociation() const;

  protected:
    void DoInitialize() override;
    void DoDispose() override;

  private:
    // ---- Routing table ----
    std::map<Ipv4Address, RoutingTableEntry> m_table; //!< Routing table (keyed by dest).

    Ptr<Ipv4StaticRouting> m_hnaRoutingTable; //!< HNA routing table.

    EventGarbageCollector m_events; //!< Event garbage collector.

    uint16_t m_packetSequenceNumber;  //!< Packet sequence number counter.
    uint16_t m_messageSequenceNumber; //!< Message sequence number counter.
    uint16_t m_ansn;                  //!< Advertised Neighbor Sequence Number.

    // ---- OLSR timers / intervals ----
    Time m_helloInterval;
    Time m_tcInterval;
    Time m_midInterval;
    Time m_hnaInterval;
    olsr::Willingness m_willingness;

    olsr::OlsrState m_state; //!< OLSR internal state.
    Ptr<Ipv4> m_ipv4;        //!< IPv4 object.

    // ---- ETX state ----
    std::map<Ipv4Address, EtxInfo> m_etxMap; //!< Per-neighbor ETX estimates.
    double m_etxAlpha;   //!< EWMA smoothing factor (0..1). Attribute "EtxAlpha".
    double m_initialEtx; //!< Default ETX for unknown links. Attribute "InitialEtx".

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

    /**
     * Add (or overwrite) a routing table entry.
     * @param dest       Destination address.
     * @param next       Next-hop address.
     * @param interface  Outgoing interface index.
     * @param distance   Hop-count distance.
     * @param etxDist    Cumulative ETX path cost.
     */
    void AddEntry(const Ipv4Address& dest,
                  const Ipv4Address& next,
                  uint32_t interface,
                  uint32_t distance,
                  double etxDist);

    /**
     * Add (or overwrite) a routing table entry, resolving interface from address.
     */
    void AddEntry(const Ipv4Address& dest,
                  const Ipv4Address& next,
                  const Ipv4Address& interfaceAddress,
                  uint32_t distance,
                  double etxDist);

    bool Lookup(const Ipv4Address& dest, RoutingTableEntry& outEntry) const;
    bool FindSendEntry(const RoutingTableEntry& entry, RoutingTableEntry& outEntry) const;

    // ---- ETX helpers ----

    /**
     * Update the EWMA PRR estimate for a neighbor from which a HELLO was received.
     * @param neighborIfaceAddr The sender's interface address.
     * @param helloInterval     The sender's advertised hello interval.
     */
    void UpdateNeighborEtx(const Ipv4Address& neighborIfaceAddr, Time helloInterval);

    /**
     * Return the ETX estimate for a direct link to neighborIfaceAddr.
     * Returns m_initialEtx if unknown.
     */
    double GetLinkEtx(const Ipv4Address& neighborIfaceAddr) const;

    // ---- Ipv4RoutingProtocol interface ----
  public:
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

    Ipv4Address m_mainAddress; //!< Main address of this node.
};

} // namespace etxolsr
} // namespace ns3

#endif /* ETX_OLSR_ROUTING_PROTOCOL_H */
