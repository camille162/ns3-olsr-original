/*
 * Hop-count OLSR (RFC 3626 baseline) helper — same protocol object as upstream
 * ns3::olsr::RoutingProtocol; filenames use etx-olsr prefix for this contrib layout.
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef ETX_OLSR_HOPCOUNT_HELPER_H
#define ETX_OLSR_HOPCOUNT_HELPER_H

#include "ns3/ipv4-routing-helper.h"
#include "ns3/node-container.h"
#include "ns3/object-factory.h"

namespace ns3
{

/**
 * @ingroup etx-olsr
 * @brief Install vanilla hop-count OLSR (ns3::olsr::RoutingProtocol).
 */
class EtxOlsrHopcountHelper : public Ipv4RoutingHelper
{
  public:
    EtxOlsrHopcountHelper();
    EtxOlsrHopcountHelper* Copy() const override;
    Ptr<Ipv4RoutingProtocol> Create(Ptr<Node> node) const override;
    void SetAttribute(std::string name, const AttributeValue& value);
    int64_t AssignStreams(NodeContainer c, int64_t stream);

  private:
    ObjectFactory m_agentFactory;
};

} // namespace ns3

#endif /* ETX_OLSR_HOPCOUNT_HELPER_H */
