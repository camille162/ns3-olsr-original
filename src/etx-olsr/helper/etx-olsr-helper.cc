/*
 * ETX-OLSR helper for ns-3.
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "etx-olsr-helper.h"

#include "ns3/etx-olsr-routing-protocol.h"
#include "ns3/ipv4-list-routing.h"
#include "ns3/ipv4.h"
#include "ns3/node.h"

namespace ns3
{

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
    Ptr<etxolsr::RoutingProtocol> agent =
        m_agentFactory.Create<etxolsr::RoutingProtocol>();
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
        Ptr<Node> node = *i;
        Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
        NS_ASSERT_MSG(ipv4, "EtxOlsrHelper::AssignStreams: no Ipv4 on node");
        Ptr<Ipv4RoutingProtocol> proto = ipv4->GetRoutingProtocol();
        Ptr<etxolsr::RoutingProtocol> etxOlsr =
            DynamicCast<etxolsr::RoutingProtocol>(proto);
        if (etxOlsr)
        {
            currentStream += etxOlsr->AssignStreams(currentStream);
            continue;
        }
        // Check if it's wrapped in a list routing
        Ptr<Ipv4ListRouting> list = DynamicCast<Ipv4ListRouting>(proto);
        if (list)
        {
            int16_t priority;
            for (uint32_t j = 0; j < list->GetNRoutingProtocols(); j++)
            {
                Ptr<Ipv4RoutingProtocol> listProto = list->GetRoutingProtocol(j, priority);
                Ptr<etxolsr::RoutingProtocol> listEtxOlsr =
                    DynamicCast<etxolsr::RoutingProtocol>(listProto);
                if (listEtxOlsr)
                {
                    currentStream += listEtxOlsr->AssignStreams(currentStream);
                    break;
                }
            }
        }
    }
    return (currentStream - stream);
}

} // namespace ns3
