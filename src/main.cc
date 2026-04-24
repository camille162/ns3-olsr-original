#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/applications-module.h"
#include "ns3/etx-olsr-helper.h" // 引入etx-olsr模块的Helper头文件

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("MainSimulation");

int main(int argc, char *argv[])
{
  // 参数配置
  uint32_t numNodes = 50;   // 设置正常节点数为50
  uint32_t numJammers = 5;  // 干扰节点数
  double simTime = 60.0;    // 仿真时长为60秒
  double areaSize = 500.0;  // 仿真区域大小为500x500米

  CommandLine cmd;
  cmd.AddValue("numNodes", "Number of normal nodes", numNodes);
  cmd.AddValue("numJammers", "Number of jamming nodes", numJammers);
  cmd.AddValue("simTime", "Simulation time in seconds", simTime);
  cmd.Parse(argc, argv);

  // 创建仿真节点
  NodeContainer normalNodes;
  normalNodes.Create(numNodes);
  NodeContainer jammingNodes;
  jammingNodes.Create(numJammers);

  // 配置节点的移动模型
  MobilityHelper mobility;
  mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                "MinX", DoubleValue(0.0),
                                "MinY", DoubleValue(0.0),
                                "DeltaX", DoubleValue(20.0),
                                "DeltaY", DoubleValue(20.0),
                                "GridWidth", UintegerValue(10),
                                "LayoutType", StringValue("RowFirst"));
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(normalNodes);
  mobility.Install(jammingNodes);

  // 配置WiFi物理层与通道
  WifiHelper wifi;
  wifi.SetStandard(WIFI_PHY_STANDARD_80211a);

  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
  wifiPhy.Set("RxGain", DoubleValue(-10));
  wifiPhy.Set("TxGain", DoubleValue(5));  // 模拟干扰
  wifiPhy.Set("ChannelNumber", UintegerValue(6));

  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
  wifiPhy.SetChannel(wifiChannel.Create());

  // MAC层和SSID
  WifiMacHelper wifiMac;
  Ssid ssid = Ssid("ETX-OLSR-Network");
  wifiMac.SetType("ns3::AdhocWifiMac");

  // 安装网络
  NetDeviceContainer normalDevices = wifi.Install(wifiPhy, wifiMac, normalNodes);
  NetDeviceContainer jammerDevices = wifi.Install(wifiPhy, wifiMac, jammingNodes);

  // 配置协议栈与ETX-OLSR
  InternetStackHelper internet;
  EtxOlsrHelper etxOlsr; // 使用自定义的EtxOlsrHelper
  internet.SetRoutingHelper(etxOlsr); // 替换默认的路由协议
  internet.Install(normalNodes);
  internet.Install(jammingNodes);

  // 为普通节点创建IP地址
  Ipv4AddressHelper ipv4;
  ipv4.SetBase("10.0.0.0", "255.255.255.0");
  Ipv4InterfaceContainer normalInterfaces = ipv4.Assign(normalDevices);
  Ipv4InterfaceContainer jammerInterfaces = ipv4.Assign(jammerDevices);

  // 应用层：普通流量
  uint16_t port = 4000;
  UdpServerHelper udpServer(port);
  ApplicationContainer serverApps = udpServer.Install(normalNodes.Get(0));
  serverApps.Start(Seconds(1.0));
  serverApps.Stop(Seconds(simTime));

  UdpClientHelper udpClient(normalInterfaces.GetAddress(0), port);
  udpClient.SetAttribute("MaxPackets", UintegerValue(1000));
  udpClient.SetAttribute("Interval", TimeValue(Seconds(0.1)));
  udpClient.SetAttribute("PacketSize", UintegerValue(512));

  ApplicationContainer clientApps = udpClient.Install(normalNodes.Get(numNodes - 1));
  clientApps.Start(Seconds(2.0));
  clientApps.Stop(Seconds(simTime));

  // 应用层：干扰流量
  OnOffHelper jammer("ns3::UdpSocketFactory", Address(Ipv4Address::GetBroadcast()));
  jammer.SetConstantRate(DataRate("2Mbps"));
  ApplicationContainer jammerApps = jammer.Install(jammingNodes);
  jammerApps.Start(Seconds(3.0));
  jammerApps.Stop(Seconds(simTime));

  // 启动仿真
  Simulator::Stop(Seconds(simTime));
  Simulator::Run();
  Simulator::Destroy();

  return 0;
}
