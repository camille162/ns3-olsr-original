/*
 * ETX-OLSR helper for ns-3.
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "etx-olsr-helper.h"

#include "etx-olsr-routing-protocol.h"

#include "ns3/assert.h"
#include "ns3/ipv4-list-routing.h"
#include "ns3/ipv4.h"
#include "ns3/log.h"
#include "ns3/node.h"
#include "ns3/pointer.h"
#include "ns3/simulator.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("EtxOlsrHelper");

EtxOlsrHelper::EtxOlsrHelper()
{
    m_agentFactory.SetTypeId("ns3::etxolsr::RoutingProtocol");
}

EtxOlsrHelper*
EtxOlsrHelper::Copy() const
{
    return new EtxOlsrHelper(*this);
}

Ptr<Ipv4RoutingProtocol>
EtxOlsrHelper::Create(Ptr<Node> node) const
{
    Ptr<etxolsr::RoutingProtocol> agent = m_agentFactory.Create<etxolsr::RoutingProtocol>();
    // Do not call SetIpv4 here: InternetStackHelper::Install binds Ipv4 via
    // Ipv4L3Protocol::SetRoutingProtocol -> Ipv4ListRouting::SetIpv4 -> SetIpv4 on each entry.
    // Calling SetIpv4 twice causes Ipv4StaticRouting (HNA table) to assert (!m_ipv4 && ipv4).
    NS_ASSERT_MSG(node, "EtxOlsrHelper::Create: null node");
    node->AggregateObject(agent);
    return agent;
}

void
EtxOlsrHelper::SetAttribute(std::string name, const AttributeValue& value)
{
    m_agentFactory.Set(name, value);
}

int64_t
EtxOlsrHelper::AssignStreams(NodeContainer c, int64_t stream)
{
    int64_t currentStream = stream;
    for (auto i = c.Begin(); i != c.End(); ++i)
    {
        Ptr<Ipv4> ipv4 = (*i)->GetObject<Ipv4>();
        NS_ASSERT_MSG(ipv4, "EtxOlsrHelper::AssignStreams: no Ipv4 on node");
        Ptr<Ipv4RoutingProtocol> rp = ipv4->GetRoutingProtocol();
        Ptr<etxolsr::RoutingProtocol> etxOlsr = DynamicCast<etxolsr::RoutingProtocol>(rp);
        if (etxOlsr)
        {
            currentStream += etxOlsr->AssignStreams(currentStream);
            continue;
        }
        Ptr<Ipv4ListRouting> lrp = DynamicCast<Ipv4ListRouting>(rp);
        if (lrp)
        {
            for (uint32_t j = 0; j < lrp->GetNRoutingProtocols(); ++j)
            {
                int16_t priority = 0;
                Ptr<Ipv4RoutingProtocol> sub = lrp->GetRoutingProtocol(j, priority);
                Ptr<etxolsr::RoutingProtocol> listEtxOlsr = DynamicCast<etxolsr::RoutingProtocol>(sub);
                if (listEtxOlsr)
                {
                    currentStream += listEtxOlsr->AssignStreams(currentStream);
                    break;
                }
            }
        }
    }
    return currentStream - stream;
}

} // namespace ns3
