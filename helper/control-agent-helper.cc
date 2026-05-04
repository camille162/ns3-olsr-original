/*
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "control-agent-helper.h"

#include "control-agent.h"

#include "ns3/log.h"
#include "ns3/node.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ControlAgentHelper");

ControlAgentHelper::ControlAgentHelper()
{
    m_factory.SetTypeId(etxolsr::ControlAgent::GetTypeId());
    m_start = Seconds(0.0);
    m_stop = Seconds(360000.0);
}

void
ControlAgentHelper::SetAttribute(std::string name, const AttributeValue& value)
{
    m_factory.Set(name, value);
}

void
ControlAgentHelper::SetStartTime(Time start)
{
    m_start = start;
}

void
ControlAgentHelper::SetStopTime(Time stop)
{
    m_stop = stop;
}

ApplicationContainer
ControlAgentHelper::Install(Ptr<Node> node) const
{
    return DoInstall(node);
}

ApplicationContainer
ControlAgentHelper::Install(const NodeContainer& c) const
{
    ApplicationContainer apps;
    for (auto i = c.Begin(); i != c.End(); ++i)
    {
        apps.Add(DoInstall(*i));
    }
    return apps;
}

ApplicationContainer
ControlAgentHelper::DoInstall(Ptr<Node> node) const
{
    Ptr<etxolsr::ControlAgent> app = m_factory.Create<etxolsr::ControlAgent>();
    node->AddApplication(app);
    app->SetStartTime(m_start);
    app->SetStopTime(m_stop);
    return ApplicationContainer(app);
}

} // namespace ns3
