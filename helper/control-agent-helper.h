/*
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef ETX_OLSR_CONTROL_AGENT_HELPER_H
#define ETX_OLSR_CONTROL_AGENT_HELPER_H

#include <string>

#include "ns3/application-container.h"
#include "ns3/attribute.h"
#include "ns3/node-container.h"
#include "ns3/nstime.h"
#include "ns3/object-factory.h"
#include "ns3/ptr.h"

namespace ns3
{

/**
 * @ingroup etx-olsr
 * Install ns3::etxolsr::ControlAgent on nodes (one per node, distributed).
 */
class ControlAgentHelper
{
  public:
    ControlAgentHelper();

    void SetAttribute(std::string name, const AttributeValue& value);

    void SetStartTime(Time start);
    void SetStopTime(Time stop);

    ApplicationContainer Install(Ptr<Node> node) const;
    ApplicationContainer Install(const NodeContainer& c) const;

  private:
    ApplicationContainer DoInstall(Ptr<Node> node) const;

    ObjectFactory m_factory;
    Time m_start;
    Time m_stop;
};

} // namespace ns3

#endif /* ETX_OLSR_CONTROL_AGENT_HELPER_H */
