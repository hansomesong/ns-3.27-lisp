/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

/*
 * RUN Command:
 *
 * RUN DEBUG Command:
 *
 * Network topology:
 */

// TODO Add helper header
#include <iostream>
#include <fstream>
#include <string>
#include <cassert>

#include "ns3/system-path.h"
#include "ns3/netanim-module.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/map-resolver-helper.h"
#include "ns3/map-server-helper.h"
#include "ns3/lisp-etr-itr-app-helper.h"
#include "ns3/wifi-module.h"
#include "ns3/wifi-mac.h"
#include "ns3/virtual-net-device.h"
#include "ns3/lisp-header.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/dhcp-helper.h"
#include "ns3/virtual-net-device.h"
#include "ns3/callback.h"

#include "ns3/internet-trace-helper.h"

#include "ns3/gnuplot.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/flow-monitor-helper.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("LispMobilityBetweenNetwork");
static uint64_t lastTotalRx = 0;
static void
SetPosition (Ptr<Node> node, Vector position)
{
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
  mobility->SetPosition (position);
}

static Vector
GetPosition (Ptr<Node> node)
{
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
  return mobility->GetPosition ();
}

static void
AdvancePosition (Ptr<Node> node)
{
  Vector pos = GetPosition (node);
  pos.x += 5.0;
  pos.y -= 5.0;
  if (pos.x >= 240.0 or pos.y >= 240 or pos.y <=0)
    {
      return;
    }
  SetPosition (node, pos);
  std::cout << "x=" << pos.x << ", y=" << pos.y << std::endl;
  Simulator::Schedule (Seconds (1.0), &AdvancePosition, node);
}

void
PrintLocations (NodeContainer nodes, std::string header)
{
  std::cout << header << std::endl;
  for (NodeContainer::Iterator iNode = nodes.Begin (); iNode != nodes.End ();
      ++iNode)
    {
      Ptr<Node> object = *iNode;
      Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
      NS_ASSERT(position != 0);
      Vector pos = position->GetPosition ();
      std::cout << "(" << pos.x << ", " << pos.y << ", " << pos.z << ")"
	  << std::endl;
    }
  std::cout << std::endl;
}

void
PrintAddresses (Ipv4InterfaceContainer container, std::string header)
{
  std::cout << header << std::endl;
  uint32_t nNodes = container.GetN ();
  for (uint32_t i = 0; i < nNodes; ++i)
    {
      std::cout << container.GetAddress (i, 0) << std::endl;
    }
  std::cout << std::endl;
}

void
InstallMapResolverApplication (Ptr<Node> node, Ipv4Address msAddress,
			       Time start, Time end)
{
  // Initializing Map Resolver Ddt
  MapResolverDdtHelper mapResDdtHelp;
  mapResDdtHelp.SetMapServerAddress (static_cast<Address> (msAddress));
  ApplicationContainer mrDdtApps = mapResDdtHelp.Install (node);
  mrDdtApps.Start (start);
  mrDdtApps.Stop (end);
}

void
InstallEchoApplication (Ptr<Node> echoServerNode, Ptr<Node> echoClientNode,
			Ipv4Address echoServerIpAddr, uint16_t port, Time start,
			Time end)
{
  // Make node 4 as Echo server, listening port 9
  UdpEchoServerHelper echoServer (port);
  ApplicationContainer serverApps = echoServer.Install (echoServerNode);
  serverApps.Start (start);
  serverApps.Stop (end);
  // Get @ip for net device of node and use it to initialize echo client
  //UdpEchoClientHelper echoClient(i3i4.GetAddress(1), 9);
  UdpEchoClientHelper echoClient (echoServerIpAddr, port);
  echoClient.SetAttribute ("MaxPackets", UintegerValue (2000));
  echoClient.SetAttribute ("Interval", TimeValue (Seconds (1)));
  echoClient.SetAttribute ("PacketSize", UintegerValue (100));
  // Install echo client app at node 0
  ApplicationContainer clientApps = echoClient.Install (echoClientNode);
  clientApps.Start (start);
  clientApps.Stop (end);
}

void
InstallTcpBulkSendApplication (Ptr<Node> echoServerNode, Ptr<Node> echoClientNode,
			Ipv4Address echoServerIpAddr, uint16_t port, Time start,
			Time end)
{
  // Make node 4 as bulk send receiver, listening port 9
  // Create a socket pointing to server address
  BulkSendHelper source ("ns3::TcpSocketFactory",
                           InetSocketAddress (echoServerIpAddr, port));
  // Set the amount of data to send in bytes.  Zero is unlimited.
  int maxBytes =0;
  source.SetAttribute ("MaxBytes", UintegerValue (maxBytes));
  ApplicationContainer sourceApps = source.Install (echoClientNode);
  sourceApps.Start (start);
  sourceApps.Stop (end);
  //
  // Create a PacketSinkApplication and install it on node 1
  //
	PacketSinkHelper sink ("ns3::TcpSocketFactory",
												 InetSocketAddress (Ipv4Address::GetAny (), port));
	ApplicationContainer sinkApps = sink.Install (echoServerNode);
  sinkApps.Start (start);
  sinkApps.Stop (end);
}

void
InstallOnOffApplication (Ptr<Node> dstNode, Ptr<Node> srcNode,
			 Ipv4Address dstIpAddr, uint16_t port, Time start,
			 Time end)
{
  // Create the OnOff applications to send udp datagrams of size
  // 210 bytes at a rate 448 kb/s
  NS_LOG_INFO("Create OnOff Applications.");
  OnOffHelper onoff ("ns3::UdpSocketFactory",
		     Address (InetSocketAddress (dstIpAddr, port))); // to CN
  onoff.SetConstantRate (DataRate ("448kb/s"));
  ApplicationContainer apps = onoff.Install (srcNode); // from MN
  apps.Start (start);
  apps.Stop (end);
  // Create a packet sink to receive these packets at CN node
  PacketSinkHelper sink (
      "ns3::UdpSocketFactory",
      Address (InetSocketAddress (Ipv4Address::GetAny (), port)));
  apps = sink.Install (dstNode);
  apps.Start (start);
  apps.Stop (end);
}

void
InstallMapServerApplication (Ptr<Node> node, Time start, Time end)
{
  // initializing Map Server ddt helper
  MapServerDdtHelper mapServerDdtHelp;
  ApplicationContainer mapServDdtApps = mapServerDdtHelp.Install (node);
  mapServDdtApps.Start (start);
  mapServDdtApps.Stop (end);
}

void
InstallDhcpServerApplication (Ptr<Node> node, Ipv4Address prefix,
			      Ipv4Mask netmask, Ipv4Address dhcpServerAddr,
			      Ipv4Address min_addr, Ipv4Address max_addr,
			      Ipv4Address gateway, Time start, Time end)
{
  DhcpServerHelper dhcp_server (prefix, netmask, dhcpServerAddr, min_addr,
				max_addr, gateway);
  ApplicationContainer ap_dhcp_server = dhcp_server.Install (node);
  ap_dhcp_server.Start (start);
  ap_dhcp_server.Stop (end);

}

void
InstallDhcpClientApplication (Ptr<Node> node, uint32_t device, Time start,
			      Time end)
{
  // Do not know... what it means device...
  DhcpClientHelper dhcp_client (device);
  ApplicationContainer ap_dhcp_client = dhcp_client.Install (node);
  ap_dhcp_client.Start (start);
  ap_dhcp_client.Stop (end);
}

void
InstallLispRouter (NodeContainer nodes)
{
  /*
   * change normal routers as a xTRs. to do this:
   * 1) LISP protocol stack should be installed (by lispHelper)
   * 2) xTR applications, in charge of lisp control plane, should be installed in these routers,
   * 		by lispEtrItrAppHelper.
   *
   * Take a NodeContainer object as input parameter.
   * Take map resolver @ip as input parameter
   */
  // finally install lisp
	std::string curExeBinPath = SystemPath::FindSelfDirectory();
	std::list<std::string> curExeBinPathList = SystemPath::Split(curExeBinPath);
	curExeBinPathList.remove("build");
	curExeBinPath = SystemPath::Join(curExeBinPathList.begin(), curExeBinPathList.end());
  NS_LOG_WARN("Current executable binary path:"<<curExeBinPath);
  std::list<std::string> rlocFilePath =  {"lisp", "mobility_between_network", "rlocs.txt"};
  std::list<std::string> rlocXmlPath =  {"lisp", "mobility_between_network", "rloc_config_xml.txt"};

  std::string rlocFP = SystemPath::Append(
		  curExeBinPath,
			SystemPath::Join(
					rlocFilePath.begin(),
					rlocFilePath.end()
			)
);
  NS_LOG_WARN("RLOC list file path:"<<rlocFP);
  NS_LOG_INFO("Install Lisp");
  LispHelper lispHelper;
  lispHelper.BuildRlocsSet(rlocFP);
	lispHelper.Install (nodes);
	std::string rlocXmlFP = SystemPath::Append(
			curExeBinPath,
			SystemPath::Join(
					rlocXmlPath.begin(),
					rlocXmlPath.end()
			)
	);
	lispHelper.BuildMapTables2 (rlocXmlFP);
  lispHelper.InstallMapTables (nodes);
  NS_LOG_WARN("Example: Lisp is successfully aggregated");
}

void
InstallXtrApplication(NodeContainer nodes, Ipv4Address mrAddress,
		       Ipv4Address msAddress, Time start, Time end){
  LispEtrItrAppHelper lispAppHelper;
  Ptr<Locator> mrLocator = Create<Locator> (mrAddress);
  lispAppHelper.AddMapResolverRlocs (mrLocator);
  lispAppHelper.AddMapServerAddress (static_cast<Address> (msAddress));
  ApplicationContainer mapResClientApps = lispAppHelper.Install (nodes);
  mapResClientApps.Start (start);
  mapResClientApps.Stop (end);
}

void
PopulateStaticRoutingTable (NodeContainer c)
{
  // No need to set default route for MN (in WiFi networks) in presence of DHCP server

  // Populate routing table for xTR1
  Ipv4StaticRoutingHelper ipv4SrHelper;
  Ptr<Ipv4> ipv4 = c.Get (1)->GetObject<Ipv4> ();
  Ptr<Ipv4StaticRouting> ipv4Stat = ipv4SrHelper.GetStaticRouting (ipv4);
  ipv4Stat->SetDefaultRoute (Ipv4Address ("10.1.2.2"), 1);

  // For xTR2 (node 2)
  ipv4 = c.Get (2)->GetObject<Ipv4> ();
  ipv4Stat = ipv4SrHelper.GetStaticRouting (ipv4);
  ipv4Stat->SetDefaultRoute (Ipv4Address ("10.1.3.2"), 1);

  // For xTR3 (node 3)
  ipv4 = c.Get (3)->GetObject<Ipv4> ();
  ipv4Stat = ipv4SrHelper.GetStaticRouting (ipv4);
  ipv4Stat->SetDefaultRoute (Ipv4Address ("10.1.5.1"), 1);

  // For CN (node 4)
  ipv4 = c.Get (4)->GetObject<Ipv4> ();
  ipv4Stat = ipv4SrHelper.GetStaticRouting (ipv4);
  ipv4Stat->SetDefaultRoute (Ipv4Address ("10.1.3.1"), 1);

  // For intermediate router node 5
  ipv4 = c.Get (5)->GetObject<Ipv4> ();
  ipv4Stat = ipv4SrHelper.GetStaticRouting (ipv4);
  //ipv4Stat->AddNetworkRouteTo(Ipv4Address ("10.1.1.0"), Ipv4Mask ("255.255.255.0"), Ipv4Address ("10.1.2.1"), 1, 0);
  ipv4Stat->AddNetworkRouteTo (Ipv4Address ("10.1.6.0"),
			       Ipv4Mask ("255.255.255.0"),
			       Ipv4Address ("10.1.4.2"), 3, 0);
  //ipv4Stat->AddNetworkRouteTo(Ipv4Address ("10.3.3.0"), Ipv4Mask ("255.255.255.0"), Ipv4Address ("10.1.5.2"), 4, 0);

  // For MR,(node 6)
  ipv4 = c.Get (6)->GetObject<Ipv4> ();
  ipv4Stat = ipv4SrHelper.GetStaticRouting (ipv4);
  ipv4Stat->SetDefaultRoute (Ipv4Address ("10.1.4.1"), 1);

  // For MS (node 7)
  ipv4 = c.Get (7)->GetObject<Ipv4> ();
  ipv4Stat = ipv4SrHelper.GetStaticRouting (ipv4);
  ipv4Stat->SetDefaultRoute (Ipv4Address ("10.1.6.1"), 1);
}

void
PopulateStaticRoutingTable2 (NodeContainer c)
{
  // Static Routing Table without DHCP
  Ipv4StaticRoutingHelper ipv4SrHelper;
  Ptr<Ipv4> ipv4 = c.Get (0)->GetObject<Ipv4> ();
  Ptr<Ipv4StaticRouting> ipv4Stat = ipv4SrHelper.GetStaticRouting (ipv4);

//	// IP route for DHCP request
//	ipv4Stat->AddHostRouteTo(Ipv4Address("255.255.255.255"), 1);
  //TODO: Should automatically to find the interface index for TUN device.
  ipv4Stat->AddNetworkRouteTo(Ipv4Address("0.0.0.0"), Ipv4Mask("/1"), 2);
  ipv4Stat->AddNetworkRouteTo(Ipv4Address("128.0.0.0"), Ipv4Mask("/1"), 2);

  // Populate routing table for xTR1
  ipv4 = c.Get (1)->GetObject<Ipv4> ();
  ipv4Stat = ipv4SrHelper.GetStaticRouting (ipv4);
  ipv4Stat->SetDefaultRoute (Ipv4Address ("10.1.2.2"), 1);

  // For xTR2 (node 2)
  ipv4 = c.Get (2)->GetObject<Ipv4> ();
  ipv4Stat = ipv4SrHelper.GetStaticRouting (ipv4);
  ipv4Stat->SetDefaultRoute (Ipv4Address ("10.1.3.2"), 1);

  // For xTR3 (node 3)
  ipv4 = c.Get (3)->GetObject<Ipv4> ();
  ipv4Stat = ipv4SrHelper.GetStaticRouting (ipv4);
  ipv4Stat->SetDefaultRoute (Ipv4Address ("10.1.5.1"), 1);

  // For CN (node 4)
  ipv4 = c.Get (4)->GetObject<Ipv4> ();
  ipv4Stat = ipv4SrHelper.GetStaticRouting (ipv4);
  ipv4Stat->SetDefaultRoute (Ipv4Address ("10.1.3.1"), 1);

  // For intermediate router node 5
  ipv4 = c.Get (5)->GetObject<Ipv4> ();
  ipv4Stat = ipv4SrHelper.GetStaticRouting (ipv4);
  ipv4Stat->AddNetworkRouteTo (Ipv4Address ("10.1.1.0"),
			       Ipv4Mask ("255.255.255.0"),
			       Ipv4Address ("10.1.2.1"), 1, 0);
  ipv4Stat->AddNetworkRouteTo (Ipv4Address ("10.1.6.0"),
			       Ipv4Mask ("255.255.255.0"),
			       Ipv4Address ("10.1.4.2"), 3, 0);
  ipv4Stat->AddNetworkRouteTo (Ipv4Address ("10.1.7.0"),
			       Ipv4Mask ("255.255.255.0"),
			       Ipv4Address ("10.1.3.1"), 2, 0);
  //ipv4Stat->AddNetworkRouteTo(Ipv4Address ("10.3.3.0"), Ipv4Mask ("255.255.255.0"), Ipv4Address ("10.1.5.2"), 4, 0);

  // For MR,(node 6)
  ipv4 = c.Get (6)->GetObject<Ipv4> ();
  ipv4Stat = ipv4SrHelper.GetStaticRouting (ipv4);
  ipv4Stat->SetDefaultRoute (Ipv4Address ("10.1.4.1"), 1);

  // For MS (node 7)
  ipv4 = c.Get (7)->GetObject<Ipv4> ();
  ipv4Stat = ipv4SrHelper.GetStaticRouting (ipv4);
  ipv4Stat->SetDefaultRoute (Ipv4Address ("10.1.6.1"), 1);
}

void
CreateAnimFile (NodeContainer c, std::string animFile)
{
  // Create the animation object and configure for specified output
  AnimationInterface anim (animFile);
  // node positions
  // It seems that what the Iterator gets is the pointer...
  /*for (NodeContainer::Iterator iNode = c.Begin(); iNode != c.End();
   ++iNode) {
   Ptr<Node> object = *iNode;
   Ptr<MobilityModel> position = object->GetObject<MobilityModel>();
   NS_ASSERT(position != 0);
   Vector pos = position->GetPosition();
   anim.SetConstantPosition(object, pos.x, pos.y, pos.z);
   }
   */
  anim.SetConstantPosition (c.Get (0), 0, 110);
  anim.SetConstantPosition (c.Get (1), 0, 120);
  anim.SetConstantPosition (c.Get (2), 120, 0);
  anim.SetConstantPosition (c.Get (3), 180, 180);
  anim.SetConstantPosition (c.Get (4), 240, 240);
  anim.SetConstantPosition (c.Get (5), 120, 120);
  anim.SetConstantPosition (c.Get (6), 180, 60);
  anim.SetConstantPosition (c.Get (7), 240, 0);
  anim.EnablePacketMetadata (true); // Optional
  anim.EnableIpv4L3ProtocolCounters (Seconds (0), Seconds (10)); // Optional
}

// Copy from: https://groups.google.com/forum/#!topic/ns-3-users/-BnPRmJwcGs
void
ThroughputMonitor (FlowMonitorHelper *fmhelper, Ptr<FlowMonitor> flowMon, Gnuplot2dDataset DataSet)
//ThroughputMonitor (FlowMonitorHelper *fmhelper, Ptr<FlowMonitor> flowMon)

{
	double localThrou=0;
	flowMon->CheckForLostPackets ();
	std::map<FlowId, FlowMonitor::FlowStats> flowStats = flowMon->GetFlowStats ();
	Ptr<Ipv4FlowClassifier> classing = DynamicCast<Ipv4FlowClassifier> (
			fmhelper->GetClassifier ());
	std::cout <<std::endl;
	std::cout <<std::endl;
	std::cout <<std::endl;
	std::cout << "Call ThroughputmMonitor method at "<<Simulator::Now().GetSeconds() << " second."<<std::endl;
	;
	for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator stats =
			flowStats.begin (); stats != flowStats.end (); ++stats)
		{
			Ipv4FlowClassifier::FiveTuple fiveTuple = classing->FindFlow (
					stats->first);
			if (
//			/**
//			 * The traffic flow is defined as the set of all packets that have the same file tuples.
//			 * We use the ACK traffic flow from remote CN to LISP-MN to analyze the instantaneous throughput.
//			 * The reason are two-fold:
//			 * 1) Originally, flow monitor only counts with the outer IP header. To count the inner IP header
//			 * needs some work.
//			 * 2) Before and after handover, the outer IP header has different source IP address
//			 */
//
////				Comment the following two lines. Maybe useful in the future.
			(fiveTuple.sourceAddress == "10.3.3.2" && fiveTuple.destinationAddress == "172.16.0.1")
//					or
//			(fiveTuple.sourceAddress == "10.1.7.1"
//					&& fiveTuple.destinationAddress == "10.1.5.2")
//					or
//			(fiveTuple.sourceAddress == "176.16.0.1"
//					&& fiveTuple.destinationAddress == "10.3.3.2")
			)
				{

					std::cout << "Flow ID			: " << stats->first << " ; "
							<< fiveTuple.sourceAddress << " -----> "
							<< fiveTuple.destinationAddress << std::endl;
					std::cout << "Tx Bytes		: " << stats->second.txBytes << "\n";
					std::cout << "Rx Bytes		: " << stats->second.rxBytes << "\n";
					std::cout << "Duration		: "
							<< stats->second.timeLastRxPacket.GetSeconds ()
									- stats->second.timeFirstTxPacket.GetSeconds () << std::endl;
					std::cout << "Last Received Packet	: "
							<< stats->second.timeLastRxPacket.GetSeconds () << " Seconds"
							<< std::endl;
//          localThrou=(stats->second.rxBytes * 8.0 / (stats->second.timeLastRxPacket.GetSeconds()-stats->second.timeFirstTxPacket.GetSeconds())/1024/1024);
          localThrou=((stats->second.rxBytes - lastTotalRx) * 8.0 /0.2/1024/1024);
          std::cout << "Throughput		: "
							<< localThrou << " Mbps" << std::endl;
					std::cout
							<< "---------------------------------------------------------------------------"
							<< std::endl;
					lastTotalRx = stats->second.rxBytes;
				}
		}
	flowMon->SerializeToXmlFile ("lisp-mobility.flowmon", true, true);
  DataSet.Add((double)Simulator::Now().GetSeconds(),(double) localThrou);
	Simulator::Schedule (Seconds (0.2), &ThroughputMonitor, fmhelper, flowMon, DataSet);
}

int
main (int argc, char *argv[])
{
	/**
	 * ./waf --run "lisp_mobility_between_subnet --dhcp-collect=0.7 --wifi-beacon-interval=0.2"
	 * ./waf --run "lisp_mobility_between_subnet --dhcp-collect=0.7 --wifi-beacon-interval=0.2 --map-search-time=0.4"
	 *
	 */
  LogComponentEnable ("LispMobilityBetweenNetwork", LOG_LEVEL_ALL);
  CommandLine cmd;
  bool g_verbose = true;
  float dhcp_collect = 1.0;
  float wifiBeaconInterval = 0.2;
  float mappingSearchTime = 0.4; //unit second
  cmd.AddValue ("verbose", "Print trace information if true", g_verbose);
  cmd.AddValue ("dhcp-collect", "Time for which offer collection starts", dhcp_collect);
  cmd.AddValue ("wifi-beacon-interval", "Time for which offer collection starts", wifiBeaconInterval);
  cmd.AddValue ("map-search-time", "Time consumed for EID-RLOC mapping search in map server", mappingSearchTime);

  cmd.Parse (argc, argv);

  std::string animFile = "lisp-mobility-between-subnet.xml"; // Name of file for animation output
  Packet::EnablePrinting ();

  // enable rts cts all the time.
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold",
		      StringValue ("2200"));
  // disable fragmentation
  Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold",
		      StringValue ("2200"));
  Config::SetDefault ("ns3::Ipv4GlobalRouting::RandomEcmpRouting",
		      BooleanValue (true));
  Config::SetDefault ("ns3::DhcpClient::Collect", TimeValue (Seconds (dhcp_collect)));
  Config::SetDefault ("ns3::MapServerDdt::SearchTime", TimeValue (Seconds (mappingSearchTime)));

//  Config::SetDefault ("ns3::ApWifiMac::BeaconInterval", TimeValue (Seconds (wifiBeaconInterval)));

  NS_LOG_INFO("Create nodes.");
  NS_LOG_INFO("Default DHCP Collect time is:"<<Seconds (dhcp_collect));
//  NS_LOG_INFO()
  NodeContainer c;
  c.Create (8);

  // Scenario: one STA in network 2, managed by 2 APs in network 1,
  // wants to communicate with one PC in another network 2 managed by one P2P link
  // LISP Mapping system have one MR and one MS
  // Except link between stats and aps, other links are p2p-based
  NodeContainer mn = NodeContainer (c.Get (0));
  NodeContainer xTR1 = NodeContainer (c.Get (1));
  NodeContainer xTR2 = NodeContainer (c.Get (2));
  NodeContainer d1d5 = NodeContainer (c.Get (1), c.Get (5));
  NodeContainer d2d5 = NodeContainer (c.Get (2), c.Get (5));
  NodeContainer d5d6 = NodeContainer (c.Get (5), c.Get (6));
  NodeContainer d5d3 = NodeContainer (c.Get (5), c.Get (3));
  NodeContainer d3d4 = NodeContainer (c.Get (3), c.Get (4));
  NodeContainer d6d7 = NodeContainer (c.Get (6), c.Get (7));

  // create channels without any IP addressing info
  NS_LOG_INFO("Create p2p channels");
  PointToPointHelper p2p;
  p2p.SetDeviceAttribute ("DataRate", StringValue ("5Mbps"));
  p2p.SetChannelAttribute ("Delay", StringValue ("2ms"));

  NetDeviceContainer netd1d5 = p2p.Install (d1d5);
  NetDeviceContainer netd2d5 = p2p.Install (d2d5);
  NetDeviceContainer netd5d6 = p2p.Install (d5d6);
  NetDeviceContainer netd5d3 = p2p.Install (d5d3);
  NetDeviceContainer netd3d4 = p2p.Install (d3d4);
  NetDeviceContainer netd6d7 = p2p.Install (d6d7);

  // make all nodes be routers
  InternetStackHelper internet;
  internet.Install (c);
  // We add IP addresses
  Ipv4AddressHelper ipv4;
  NS_LOG_INFO("Assign IP Addresses.");

  ipv4.SetBase ("10.1.2.0", "255.255.255.0");
  Ipv4InterfaceContainer i1i5 = ipv4.Assign (netd1d5);
  // Set link metrics to make sure that xTR1 is the best choice for prefix 10.1.1.0/24
  i1i5.SetMetric (0, 1);
  i1i5.SetMetric (1, 1);

  ipv4.SetBase ("10.1.3.0", "255.255.255.0");
  Ipv4InterfaceContainer i2i5 = ipv4.Assign (netd2d5);
  i2i5.SetMetric (0, 3);
  i2i5.SetMetric (1, 3);

  ipv4.SetBase ("10.1.4.0", "255.255.255.0");
  Ipv4InterfaceContainer i5i6 = ipv4.Assign (netd5d6);

  ipv4.SetBase ("10.1.5.0", "255.255.255.0");
  Ipv4InterfaceContainer i5i3 = ipv4.Assign (netd5d3);

  ipv4.SetBase ("10.1.6.0", "255.255.255.0");
  Ipv4InterfaceContainer i6i7 = ipv4.Assign (netd6d7);

  ipv4.SetBase ("10.3.3.0", "255.255.255.0");
  Ipv4InterfaceContainer i3i4 = ipv4.Assign (netd3d4);

  // configure wifi PHY layer and channel helper
  //TODO maybe should add path-loss and rss level? Otherwise how STA change the attached AP?
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  // If one want to use another propagation loss model different with the default one.
  // Don't use syntax
//  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper();

//  wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
//  wifiChannel.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange", DoubleValue(90));

  wifiPhy.SetChannel (wifiChannel.Create ());

  WifiHelper wifi = WifiHelper();
  // set rate control algo.
  wifi.SetRemoteStationManager ("ns3::ArfWifiManager");
  // use no-Qos MAC
  NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
  // Ad-hoc wifi
//  wifiMac.SetType ("ns3::AdhocWifiMac");
  // wifi mac setup for mn(i.e. STA in wifi's terminology).
  Ssid ssid = Ssid("xTR1");
  wifiMac.SetType(
		  "ns3::StaWifiMac",
		  "Ssid", SsidValue(ssid), // need to tell STA the target SSID
		  "ActiveProbing", BooleanValue(true) //make sure that STA does not perform active probing
  );

  //Once all station specific parameters are configured, create wifi devices of STA
  NetDeviceContainer mnDevs;
  mnDevs = wifi.Install (wifiPhy, wifiMac, mn);
  // wifi mac setup for the involved xTRs (i.e. access point in wifi's terminology).
  // change the defaut attributes of NqosWifiMacHelper to reflect the requirements of the xTR (i.e. AP)
  wifiMac.SetType(
		  "ns3::ApWifiMac",		// Non-Qos Access Point Type
		  "Ssid", SsidValue(ssid), "BeaconGeneration", BooleanValue(true),
		  "BeaconInterval", TimeValue(Seconds(wifiBeaconInterval)),
		  "EnableBeaconJitter", BooleanValue(true)
  );
  // Mofidy above atribute to change Beacon Interval. Now set as 0.6, namely 600 ms.
  // Compile and execute
  NetDeviceContainer xTRDev1, xTRDev2;
  xTRDev1 = wifi.Install (wifiPhy, wifiMac, xTR1);
  xTRDev2 = wifi.Install (wifiPhy, wifiMac, xTR2);

  Ptr<Ipv4> ipv4MN = c.Get (0)->GetObject<Ipv4> ();
  uint32_t ifIndex1 = ipv4MN->AddInterface (mnDevs.Get (0));
  ipv4MN->AddAddress (
      ifIndex1,
      Ipv4InterfaceAddress (Ipv4Address ("0.0.0.0"), Ipv4Mask ("/0")));
  ipv4MN->SetForwarding (ifIndex1, true);
  ipv4MN->SetUp (ifIndex1);
  /**
   * Set TUN/TAP device for LISP-MN node
   * A question: for tap, we need to care about Ethernet header or not?
   */
  Address lispMnEidAddr = Ipv4Address ("172.16.0.1");
  Ptr<NetDevice> m_n0Tap = CreateObject<TunNetDevice> (mnDevs.Get (0));
  c.Get (0)->AddDevice (m_n0Tap);
  Ptr<Ipv4> ipv4Tun = c.Get (0)->GetObject<Ipv4> ();
  uint32_t ifIndexTap = ipv4Tun->AddInterface (m_n0Tap);
  ipv4Tun->AddAddress (
      ifIndexTap,
      Ipv4InterfaceAddress (Ipv4Address::ConvertFrom(lispMnEidAddr),
			    Ipv4Mask ("255.255.255.255")));
  ipv4Tun->SetForwarding (ifIndexTap, true);
  ipv4Tun->SetUp (ifIndexTap);

  /*
   * Assign a unique address: 10.1.1.254 for wifi net device on xTR1
   * code snippet fromm dhcp-example.cc
   */
  Ptr<Ipv4> ipv4xTR1 = c.Get (1)->GetObject<Ipv4> ();
  uint32_t ifIndex = ipv4xTR1->AddInterface (xTRDev1.Get (0));
  ipv4xTR1->AddAddress (
      ifIndex,
      Ipv4InterfaceAddress (Ipv4Address ("10.1.1.254"), Ipv4Mask ("/24")));
  ipv4xTR1->SetForwarding (ifIndex, true);
  ipv4xTR1->SetMetric (ifIndex, 1);
  ipv4xTR1->SetUp (ifIndex);
  Ipv4InterfaceContainer ap1Interface;
  ap1Interface.Add (ipv4xTR1, ifIndex);

  /*
   * Assign a unique address: 10.1.7.254 for wifi net device on xTR2
   * code snippet fromm dhcp-example.cc
   */
  Ptr<Ipv4> ipv4xTR2 = c.Get (2)->GetObject<Ipv4> ();
  uint32_t ifIndex2 = ipv4xTR2->AddInterface (xTRDev2.Get (0));
//	ipv4xTR1->AddAddress(ifIndex,
//			Ipv4InterfaceAddress(Ipv4Address("10.1.1.254"), Ipv4Mask("/0"))); // need to remove this workaround
  ipv4xTR2->AddAddress (
      ifIndex,
      Ipv4InterfaceAddress (Ipv4Address ("10.1.7.254"), Ipv4Mask ("/24")));
  ipv4xTR2->SetForwarding (ifIndex, true);
  ipv4xTR2->SetMetric (ifIndex, 1);
  ipv4xTR2->SetUp (ifIndex);
  Ipv4InterfaceContainer ap2Interface;
  ap2Interface.Add (ipv4xTR2, ifIndex2);

//  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
//  //Ipv4InterfaceContainer ap1Interface = ipv4.Assign(xTRDev1);
//  Ipv4InterfaceContainer ap2Interface = ipv4.Assign (xTRDev2);

  // Create router node before assigning @ip for xTR2 to avoid multiple path problem.
  NS_LOG_INFO("Populating routing table...");
  // For such a simple topology, we use static route instead of global routing.
  //Ipv4GlobalRoutingHelper::PopulateRoutingTables();
  PopulateStaticRoutingTable2 (c);

  Ipv4GlobalRoutingHelper helper;
  NS_LOG_INFO("Print routing table...");
  Ptr<OutputStreamWrapper> stream1 = Create<OutputStreamWrapper> (
      "Table2", std::ios::out);
  helper.PrintRoutingTableAllAt (Seconds (5.0), stream1);
  // Add mobility model.
  // MN (i.e. STA) is mobile, wandering around inside a bouding box.
  // Other nodes are stationary.
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc =
      CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0.0, 110.0, 0.0)); // position MN
//  positionAlloc->Add (Vector (110, 0.0, 0.0)); // position MN
  positionAlloc->Add (Vector (0.0, 120.0, 0.0)); // position xTR1
  positionAlloc->Add (Vector (120.0, 0.0, 0.0)); // position xTR2
  positionAlloc->Add (Vector (180, 180.0, 0.0)); // position xTR3
  positionAlloc->Add (Vector (240.0, 240.0, 0.0)); // position CN
  positionAlloc->Add (Vector (120.0, 120.0, 0.0)); // position intermiediate router
  positionAlloc->Add (Vector (180.0, 60.0, 0.0)); // position MR
  positionAlloc->Add (Vector (240.0, 0.0, 0.0)); //position MS
  mobility.SetPositionAllocator (positionAlloc);
  mobility.Install (c);
  PrintLocations (c, "Location of all nodes");

  Simulator::Schedule(Seconds(10), &AdvancePosition, mn.Get(0));

  Time END_T = Seconds (45);
  Time ECO_END_T = Seconds (45.5);
  Time START_T = Seconds (5.0);

  /*
   * Now use DHCP server to allocate @Ip for MN
   * Due to DHCP program implementation constraint, we have to first assign a 0.0.0.0/0 for DHCP client
   */
  NS_LOG_INFO("Start DHCP server...");
  InstallDhcpServerApplication (c.Get (1), Ipv4Address ("10.1.1.0"),
				Ipv4Mask ("/24"), ap1Interface.GetAddress (0),
				Ipv4Address ("10.1.1.0"),
				Ipv4Address ("10.1.1.252"),
				ap1Interface.GetAddress (0), Seconds (0.0),
				END_T);
  InstallDhcpServerApplication (c.Get (2), Ipv4Address ("10.1.7.0"),
				Ipv4Mask ("/24"), ap2Interface.GetAddress (0),
				Ipv4Address ("10.1.7.0"),
				Ipv4Address ("10.1.7.252"),
				ap2Interface.GetAddress (0), Seconds (0.0),
				END_T);
  //Install DHCP client app at MN (i.e. node 0), has to be runned at the beginning...
  //ATTENTIO!!!: if use DHCP mode, you have to used the second static routing conf...!!!
  NS_LOG_INFO("Start DHCP client at MN...");
  InstallDhcpClientApplication (c.Get (0), 1, Seconds (0.0), END_T);

//	InstallOnOffApplication(c.Get(4), c.Get(0), i3i4.GetAddress(1), 9, START_T, END_T); // Discard port (RFC 863)
//  InstallEchoApplication (c.Get (4), c.Get (0), i3i4.GetAddress (1), 9, START_T,
//			  ECO_END_T); // Discard port (RFC 863)

  InstallTcpBulkSendApplication (c.Get (4), c.Get (0), i3i4.GetAddress (1), 9, START_T,
			  ECO_END_T); // Discard port (RFC 863)

  // Make xTR1&2&3 as lisp-supported routers
  Ipv4Address msAddr = i6i7.GetAddress (1);
  Ipv4Address mrAddr = i5i6.GetAddress (1);
  NodeContainer lispRouters = NodeContainer (c.Get (1), c.Get(2), c.Get (3));
  lispRouters.Add(c.Get(0));
  lispRouters.Add(c.Get(6));
  lispRouters.Add(c.Get(7));

  NodeContainer xTRNodes = NodeContainer(c.Get(0), c.Get (1), c.Get (2), c.Get (3));
  // MR/MS are also lisp-speaking devices => install lisp but no xTR app. Instead mr/ms app.
  InstallLispRouter (lispRouters);
  // Install xTR apps on xTR nodes.
  InstallXtrApplication(xTRNodes, mrAddr, msAddr, Seconds (0.0), END_T);
  // Stop xTR1 at 20s.
//  InstallXtrApplication(xTRNodes2, mrAddr, msAddr, Seconds (0.0), Seconds(10.0));
  // Install Map Resolver at node 6
  InstallMapResolverApplication (c.Get (6), msAddr, Seconds (0.0), END_T);
  // Install lisp Mapping server at node 7
  InstallMapServerApplication (c.Get (7), Seconds (0.0), END_T);


  AsciiTraceHelper ascii;
  p2p.EnableAsciiAll (
      ascii.CreateFileStream ("lisp-mobility-between-subnet.tr"));
  wifiPhy.EnableAscii(ascii.CreateFileStream ("lisp-mn.tr"), 0, 1);

  wifiPhy.EnablePcapAll ("lisp-mobility-between-subnet-wifi", true);
//  wifiPhy.EnablePcap("lisp-mobility-between-subnet-wifi2", m_n0Tap);
  p2p.EnablePcapAll ("lisp-mobility-between-subnet");
  MobilityHelper::EnableAsciiAll (
      ascii.CreateFileStream ("lisp-mobility-between-subnet.mob"));


  Names::Add("lisp-mn-ipv4", c.Get(0)->GetObject<Ipv4>());

  internet.EnableAsciiIpv4("ipv4-trace-lisp-mn.tr", "lisp-mn-ipv4", 1);

  //CreateAnimFile(c, animFile);
  AnimationInterface anim (animFile);
  anim.SetConstantPosition (c.Get (0), 0, 110);
  anim.SetConstantPosition (c.Get (1), 0, 120);
  anim.SetConstantPosition (c.Get (2), 120, 0);
  anim.SetConstantPosition (c.Get (3), 180, 180);
  anim.SetConstantPosition (c.Get (4), 240, 240);
  anim.SetConstantPosition (c.Get (5), 120, 120);
  anim.SetConstantPosition (c.Get (6), 180, 60);
  anim.SetConstantPosition (c.Get (7), 240, 0);
  anim.SetMobilityPollInterval (Seconds (0.25));
  anim.EnablePacketMetadata (true); // Optional
  anim.EnableIpv4L3ProtocolCounters (Seconds (0), END_T); // Optional
  anim.EnableIpv4RouteTracking ("lisp-mobility-double-encap-routing-table-case2.xml",
				Seconds (0), END_T, Seconds (0.25));

	// Gnuplot parameter
	std::string fileNameWithNoExtension = "lisp-mobility-double-encap-throughput-vs-time";
	std::string graphicsFileName = fileNameWithNoExtension + ".png";
	std::string plotFileName = fileNameWithNoExtension + ".plt";
	std::string plotTitle = "Instantaneous Throughput";
	std::string dataTitle = "lisp-tcp-throughput";

	// Instantiate the plot and set its title.
	Gnuplot gnuplot (graphicsFileName);
	gnuplot.SetTitle (plotTitle);

	// Make the graphics file, which the plot file will be when it
	// is used with Gnuplot, be a PNG file.
	gnuplot.SetTerminal ("png");

	// Set the labels for each axis.
	gnuplot.SetLegend ("Time/Seconds", "Throughput/Mbps");

	Gnuplot2dDataset dataset;
	dataset.SetTitle (dataTitle);
	dataset.SetStyle (Gnuplot2dDataset::POINTS);

	//------Throughput Monitor--------------------------------------------------------------------------------------
	FlowMonitorHelper fmHelper;
	Ptr<FlowMonitor> monitor = fmHelper.InstallAll ();
//	NodeContainer flowmon_nodes;
//	flowmon_nodes.Add (mn);
//	flowmon_nodes.Add (c.Get (4));
  Ptr<FlowMonitor> allMon = fmHelper.InstallAll();
	//*
//	allMon->SetAttribute ("DelayBinWidth", DoubleValue (0.001));
//	allMon->SetAttribute ("JitterBinWidth", DoubleValue (0.001));
//	allMon->SetAttribute ("PacketSizeBinWidth", DoubleValue (20));
	//*/
//  ThroughputMonitor(&fmHelper, allMon, dataset);
  ThroughputMonitor(&fmHelper, allMon, dataset);



	NS_LOG_INFO("Now do the real Simulation.");
	// Set stop time before run simulation.
	Simulator::Stop (END_T);
	Simulator::Run ();

	//Gnuplot ...continued
	gnuplot.AddDataset (dataset);
	// Open the plot file.
	std::ofstream plotFile (plotFileName.c_str ());
	// Write the plot file.
	gnuplot.GenerateOutput (plotFile);
	// Close the plot file.
	plotFile.close ();

  NS_LOG_INFO("Simulation Done.");
  Simulator::Destroy ();

  return 0;
}
