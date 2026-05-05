/*
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "etx-olsr-hopcount-helper.h"

#include "etx-olsr-hopcount-routing-protocol.h"

#include "ns3/assert.h"
#include "ns3/ipv4-list-routing.h"
#include "ns3/ipv4.h"
#include "ns3/log.h"
#include "ns3/node.h"
#include "ns3/pointer.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("EtxOlsrHopcountHelper");

EtxOlsrHopcountHelper::EtxOlsrHopcountHelper()
{
    m_agentFactory.SetTypeId("ns3::olsr::RoutingProtocol");
}

EtxOlsrHopcountHelper*
EtxOlsrHopcountHelper::Copy() const
{
    return new EtxOlsrHopcountHelper(*this);
}

Ptr<Ipv4RoutingProtocol>
EtxOlsrHopcountHelper::Create(Ptr<Node> node) const
{
    Ptr<olsr::RoutingProtocol> agent = m_agentFactory.Create<olsr::RoutingProtocol>();
    NS_ASSERT_MSG(node, "EtxOlsrHopcountHelper::Create: null node");
    node->AggregateObject(agent);
    return agent;
}

void
EtxOlsrHopcountHelper::SetAttribute(std::string name, const AttributeValue& value)
{
    m_agentFactory.Set(name, value);
}

int64_t
EtxOlsrHopcountHelper::AssignStreams(NodeContainer c, int64_t stream)
{
    int64_t currentStream = stream;
    for (auto i = c.Begin(); i != c.End(); ++i)
    {
        Ptr<Ipv4> ipv4 = (*i)->GetObject<Ipv4>();
        NS_ASSERT_MSG(ipv4, "EtxOlsrHopcountHelper::AssignStreams: no Ipv4 on node");
        Ptr<Ipv4RoutingProtocol> rp = ipv4->GetRoutingProtocol();
        Ptr<olsr::RoutingProtocol> olsrRp = DynamicCast<olsr::RoutingProtocol>(rp);
        if (olsrRp)
        {
            currentStream += olsrRp->AssignStreams(currentStream);
            continue;
        }
        Ptr<Ipv4ListRouting> lrp = DynamicCast<Ipv4ListRouting>(rp);
        if (lrp)
        {
            for (uint32_t j = 0; j < lrp->GetNRoutingProtocols(); ++j)
            {
                int16_t priority = 0;
                Ptr<Ipv4RoutingProtocol> sub = lrp->GetRoutingProtocol(j, priority);
                Ptr<olsr::RoutingProtocol> listOlsr = DynamicCast<olsr::RoutingProtocol>(sub);
                if (listOlsr)
                {
                    currentStream += listOlsr->AssignStreams(currentStream);
                    break;
                }
            }
        }
    }
    return currentStream - stream;
}

} // namespace ns3
