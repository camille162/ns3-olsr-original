#include <iostream>
#include <ns3/core-module.h>
#include <ns3/network-module.h>
#include <ns3/internet-module.h>
#include <ns3/olsr-helper.h>
#include <ns3/yans-wifi-channel.h>
#include <ns3/wifi-module.h>
#include <ns3/flow-monitor-module.h>
#include <ns3/gauss-markov-mobility-model.h>
#include <ns3/udp-socket-factory.h>
#include <ns3/onoff-application.h>
#include <ns3/log.h>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("AntiDroneSimulation");

void EnableJammer(Ptr<Node> jammerNode, double startTime, double stopTime)
{
    Ptr<OnOffApplication> app = CreateObject<OnOffApplication>();
    app->SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
    app->SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
    app->SetAttribute ("DataRate", StringValue ("1000bps"));
    app->SetAttribute ("Remote", AddressValue (InetSocketAddress (InetSocketAddress::GetBroadcast (), 9)));
    jammerNode->AddApplication (app);
    app->SetStartTime(Seconds(startTime));
    app->SetStopTime(Seconds(stopTime));
}

int main (int argc, char *argv[])
{
    CommandLine cmd;
    double stopTime = 100.0;
    bool enableJammer = false;
    cmd.AddValue ("stopTime", "Simulation stop time", stopTime);
    cmd.AddValue ("enableJammer", "Enable jammer (true/false)", enableJammer);
    cmd.Parse (argc, argv);

    NodeContainer nodes;
    nodes.Create (51); // 50 nodes + 1 jammer

    YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
    YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
    phy.SetChannel (channel.Create ());

    WifiHelper wifi;
    wifi.SetStandard (WIFI_STANDARD_802_11g);
    wifi.SetRemoteStationManager ("ns3::ConstantRateManager");

    WifiMacHelper mac;
    mac.SetType ("ns3::AdhocWifiMac");
    NetDeviceContainer devices = wifi.Install (phy.Create (), mac.Create (nodes));

    InternetStackHelper internet;
    internet.Install (nodes);

    OlsrHelper olsr;
    Ipv4ListRoutingHelper list;
    list.Add(olsr, 10);
    Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::GaussMarkovMobilityModel");
    mobility.Install(nodes.Get(0, 49)); // normal nodes
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(nodes.Get(50)); // jammer node

    if (enableJammer)
    {
        EnableJammer(nodes.Get(50), 20.0, 80.0);
        NS_LOG_INFO("Jammer enabled");
    }

    // Create a UDP flow from node 10 to node 0
    uint16_t port = 9;
    UdpSocketFactoryImpl socket;
    Ptr<Socket> sinkSocket = Socket::CreateSocket(nodes.Get(0), UdpSocketFactory::GetTypeId());
    sinkSocket->Bind(InetSocketAddress (Ipv4Address::GetAny(), port));
    sinkSocket->SetRecvCallback(MakeCallback(&ReceivePacket));

    Ptr<Socket> sourceSocket = Socket::CreateSocket(nodes.Get(10), UdpSocketFactory::GetTypeId());
    sourceSocket->Connect(InetSocketAddress(sinkAddress, port));
    sourceSocket->SetStartTime(Seconds(12));
    sourceSocket->SetStopTime(Seconds(stopTime - 1));

    // Flow Monitor
    FlowMonitorHelper flowMonitor;
    Ptr<FlowMonitor> monitor = flowMonitor.InstallAll();

    Simulator::Stop(Seconds(stopTime));
    Simulator::Run();

    monitor->SerializeToXmlFile("flow-monitor.xml", true, true);
    Simulator::Destroy();
    return 0;
}
