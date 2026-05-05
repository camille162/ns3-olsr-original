/*
 * ETX-OLSR helper for ns-3.
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

 #ifndef ETX_OLSR_HELPER_H
 #define ETX_OLSR_HELPER_H
 
 #include "ns3/ipv4-routing-helper.h"
 #include "ns3/node-container.h"
 #include "ns3/object-factory.h"
 
 namespace ns3
 {
 
 /**
  * @ingroup etx-olsr
  *
  * @brief Helper class for configuring and installing ETX-OLSR routing.
  *
  * Usage example:
  * @code
  *   EtxOlsrHelper etxOlsr;
  *   etxOlsr.SetAttribute("EtxAlpha", DoubleValue(0.8));
  *
  *   Ipv4ListRoutingHelper list;
  *   list.Add(etxOlsr, 10);
  *
  *   InternetStackHelper internet;
  *   internet.SetRoutingHelper(list);
  *   internet.Install(nodes);
  * @endcode
  */
 class EtxOlsrHelper : public Ipv4RoutingHelper
 {
   public:
     EtxOlsrHelper();
 
     /**
      * @brief Copy constructor.
      * @return A cloned EtxOlsrHelper.
      */
     EtxOlsrHelper* Copy() const override;
 
     /**
      * @brief Create and return an etxolsr::RoutingProtocol instance.
      * @param node The node on which the routing protocol will run.
      * @return The created routing protocol.
      */
     Ptr<Ipv4RoutingProtocol> Create(Ptr<Node> node) const override;
 
     /**
      * @brief Set an attribute on the underlying etxolsr::RoutingProtocol.
      * @param name  Attribute name.
      * @param value Attribute value.
      */
     void SetAttribute(std::string name, const AttributeValue& value);
 
     /**
      * @brief Assign fixed random stream numbers to random variables used by
      *        the ETX-OLSR protocol instances on a set of nodes.
      * @param c      The NodeContainer.
      * @param stream First stream index to assign.
      * @return Number of streams assigned.
      */
     int64_t AssignStreams(NodeContainer c, int64_t stream);
 
   private:
     ObjectFactory m_agentFactory; //!< Factory for etxolsr::RoutingProtocol objects.
 };
 
 } // namespace ns3
 
 #endif /* ETX_OLSR_HELPER_H */
 