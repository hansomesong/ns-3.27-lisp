/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

/*
 * RUN Command:
 *
 * RUN DEBUG Command:
 *
 * Network topology:
 */

// TODO Add helper header
#include "lisp_mobility_between_subnet_p2p_single_encap.h"

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
#include "ns3/flow-monitor-helper.h"
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

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("LispMobilityBetweenNetwork");

bool
Fuck::TapVirtualSend (Ptr<Packet> packet, const Address& source,
		const Address& dest, uint16_t protocolNumber)
{
//    NS_LOG_DEBUG("Transmitted packet: " << *packet);
	m_netDve->Send (packet, dest, protocolNumber);
	return true;
}

Fuck::Fuck (Ptr<VirtualNetDevice> n0Tap, Ptr<NetDevice> netDve) :
		m_n0Tap (n0Tap), m_netDve (netDve)
{
	m_n0Tap->SetSendCallback (MakeCallback (&Fuck::TapVirtualSend, this));
}


Simulation3::Simulation3(Address addr)
{
  lispMnEidAddr = Ipv4Address ("172.16.0.1");
  rlocsFile = "rlocs_single_encap.txt";
  mapServerInitDbFile = "rloc_config_xml_single_encap.txt";
  wifiPcapFilePrefix = "lisp-mobility-between-subnet-p2p";

}

void Simulation3::SetPosition (Ptr<Node> node, Vector position)
{
	Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
	mobility->SetPosition (position);
}

Vector Simulation3::GetPosition (Ptr<Node> node)
{
	Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
	return mobility->GetPosition ();
}

void Simulation3::AdvancePosition (Ptr<Node> node)
{
	Vector pos = GetPosition (node);
	pos.x += 5.0;
	pos.y -= 5.0;
	if (pos.x >= 240.0 or pos.y >= 240 or pos.y <= 0)
		{
			return;
		}
	SetPosition (node, pos);

	std::cout << "x=" << pos.x << ", y=" << pos.y << std::endl;

	Simulator::Schedule (Seconds (1.0), &AdvancePosition, node);
}






void
Simulation3::ChangeDefautGateway (Ptr<Node> node, Ipv4Address gateway, uint32_t interface)
{
  // set defaut route for MN (in WiFi networks)
  Ipv4StaticRoutingHelper ipv4SrHelper;
  Ptr<Ipv4> ipv4 = node->GetObject<Ipv4> ();
  Ptr<Ipv4StaticRouting> ipv4Stat = ipv4SrHelper.GetStaticRouting (ipv4);
  ipv4Stat->RemoveRoute (2);
  ipv4Stat->SetDefaultRoute (gateway, interface);
  Ptr<OutputStreamWrapper> stream1 = Create<OutputStreamWrapper> (
      "Static Routing Table for MN", std::ios::out);
  ipv4Stat->PrintRoutingTable (stream1);
}


void Simulation3::PrintLocations (NodeContainer nodes, std::string header)
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


void Simulation3::InstallMapResolverApplication (Ptr<Node> node, Ipv4Address msAddress,
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
Simulation3::InstallEchoApplication (Ptr<Node> echoServerNode, Ptr<Node> echoClientNode,
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
  echoClient.SetAttribute ("MaxPackets", UintegerValue (10000));
  echoClient.SetAttribute ("Interval", TimeValue (Seconds (1)));
  echoClient.SetAttribute ("PacketSize", UintegerValue (1024));
  // Install echo client app at node 0
  ApplicationContainer clientApps = echoClient.Install (echoClientNode);
  clientApps.Start (start);
  clientApps.Stop (end);
}

void
Simulation3::InstallMapServerApplication (Ptr<Node> node, Time start, Time end)
{
  // initializing Map Server ddt helper
  MapServerDdtHelper mapServerDdtHelp;
  ApplicationContainer mapServDdtApps = mapServerDdtHelp.Install (node);
  mapServDdtApps.Start (start);
  mapServDdtApps.Stop (end);
}

void
Simulation3::InstallDhcpServerApplication (Ptr<Node> node, Ipv4Address prefix,
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
Simulation3::InstallDhcpClientApplication (Ptr<Node> node, uint32_t device, Time start,
			      Time end)
{
  // Do not know... what it means device...
  DhcpClientHelper dhcp_client (device);
  ApplicationContainer ap_dhcp_client = dhcp_client.Install (node);
  ap_dhcp_client.Start (start);
  ap_dhcp_client.Stop (end);
}


void
Simulation3::InstallLispRouter (NodeContainer nodes)
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
  std::list<std::string> rlocFilePath =  {"lisp", "mobility_between_network", rlocsFile};
  std::list<std::string> rlocXmlPath =  {"lisp", "mobility_between_network", mapServerInitDbFile};

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
	NS_LOG_INFO("Install Lisp step 2...");
	lispHelper.BuildMapTables2 (rlocXmlFP);
	NS_LOG_INFO("Install Lisp step 3...");
  lispHelper.InstallMapTables (nodes);
  NS_LOG_WARN("Example: Lisp is successfully aggregated");
}

void
Simulation3::InstallXtrApplication(NodeContainer nodes, Ipv4Address mrAddress,
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
Simulation3::PopulateStaticRoutingTable2 (NodeContainer c, uint32_t tunDevIndex)
{
  // Static Routing Table without DHCP
  Ipv4StaticRoutingHelper ipv4SrHelper;
  Ptr<Ipv4> ipv4 = c.Get (0)->GetObject<Ipv4> ();
  Ptr<Ipv4StaticRouting> ipv4Stat = ipv4SrHelper.GetStaticRouting (ipv4);

//	// IP route for DHCP request
//	ipv4Stat->AddHostRouteTo(Ipv4Address("255.255.255.255"), 1);
  //TODO: Should automatically to find the interface index for TUN device.
  ipv4Stat->AddNetworkRouteTo(Ipv4Address("0.0.0.0"), Ipv4Mask("/1"), tunDevIndex);
  ipv4Stat->AddNetworkRouteTo(Ipv4Address("128.0.0.0"), Ipv4Mask("/1"), tunDevIndex);
//  ipv4Stat->AddNetworkRouteTo(Ipv4Address("0.0.0.0"), Ipv4Mask("/1"), 4);
//  ipv4Stat->AddNetworkRouteTo(Ipv4Address("128.0.0.0"), Ipv4Mask("/1"), 4);

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

  // For MR/MS,(node 6)
  ipv4 = c.Get (6)->GetObject<Ipv4> ();
  ipv4Stat = ipv4SrHelper.GetStaticRouting (ipv4);
  ipv4Stat->SetDefaultRoute (Ipv4Address ("10.1.4.1"), 1);
}

void
Simulation3::CreateAnimFile (NodeContainer c, std::string animFile)
{
  // Create the animation object and configure for specified output
  AnimationInterface anim (animFile);
  anim.SetConstantPosition (c.Get (0), 0, 110);
  anim.SetConstantPosition (c.Get (1), 0, 120);
  anim.SetConstantPosition (c.Get (2), 120, 0);
  anim.SetConstantPosition (c.Get (3), 180, 180);
  anim.SetConstantPosition (c.Get (4), 240, 240);
  anim.SetConstantPosition (c.Get (5), 120, 120);
  anim.SetConstantPosition (c.Get (6), 180, 60);
  anim.EnablePacketMetadata (true); // Optional
  anim.EnableIpv4L3ProtocolCounters (Seconds (0), Seconds (10)); // Optional
}

void HandOver(Ptr<Node> node, Ptr<VirtualNetDevice> tun, Ptr<NetDevice> dev)
{
  Ptr<Ipv4> ipv4MN = node->GetObject<Ipv4> ();
  ipv4MN->SetDown (1);
  ipv4MN->SetDown (3);
  ipv4MN->SetUp (2);
  ipv4MN->SetUp (4);

  Ipv4StaticRoutingHelper ipv4SrHelper;
  Ptr<Ipv4> ipv4 = node->GetObject<Ipv4> ();
  Ptr<Ipv4StaticRouting> ipv4Stat = ipv4SrHelper.GetStaticRouting (ipv4);

//	// IP route for DHCP request
//	ipv4Stat->AddHostRouteTo(Ipv4Address("255.255.255.255"), 1);
  //TODO: Should automatically to find the interface index for TUN device.

  ipv4Stat->AddNetworkRouteTo(Ipv4Address("0.0.0.0"), Ipv4Mask("/1"), 4);
  ipv4Stat->AddNetworkRouteTo(Ipv4Address("128.0.0.0"), Ipv4Mask("/1"), 4);
}

int
main (int argc, char *argv[])
{
	/**
	 * ./waf --run "lisp_mobility_between_subnet --dhcp-collect=0.7 --wifi-beacon-interval=0.2"
	 * ./waf --run "lisp_mobility_between_subnet --dhcp-collect=0.7 --wifi-beacon-interval=0.2 --map-search-time=0.4"
	 *
	 */
	Address lispMnEidAddr = Ipv4Address ("172.16.0.1");
	Ptr<Simulation3> sim = CreateObject<Simulation3>(lispMnEidAddr);

  LogComponentEnable ("LispMobilityBetweenNetwork", LOG_LEVEL_ALL);
  CommandLine cmd;
  float dhcp_collect = 1.0;
  float mappingSearchTime = 0.0; //unit second
  cmd.AddValue ("dhcp-collect", "Time for which offer collection starts", dhcp_collect);
  cmd.AddValue ("map-search-time", "Time consumed for EID-RLOC mapping search in map server", mappingSearchTime);

  cmd.Parse (argc, argv);

  std::string animFile = "lisp-mobility-between-subnet.xml"; // Name of file for animation output
  Packet::EnablePrinting ();

  // enable rts cts all the time.
  Config::SetDefault ("ns3::Ipv4GlobalRouting::RandomEcmpRouting",
		      BooleanValue (true));
  Config::SetDefault ("ns3::DhcpClient::Collect", TimeValue (Seconds (dhcp_collect)));
  Config::SetDefault ("ns3::MapServerDdt::SearchTime", TimeValue (Seconds (mappingSearchTime)));

//  Config::SetDefault ("ns3::ApWifiMac::BeaconInterval", TimeValue (Seconds (wifiBeaconInterval)));

  NS_LOG_INFO("Create nodes.");
  NS_LOG_INFO("Default DHCP Collect time is:"<<Seconds (dhcp_collect));

  NodeContainer c;
  c.Create (7);

  // Scenario: one STA in network 2, managed by 2 APs in network 1,
  // wants to communicate with one PC in another network 2 managed by one P2P link
  // LISP Mapping system have one MR and one MS
  // Except link between stats and aps, other links are p2p-based
  NodeContainer mnxTR1 = NodeContainer (c.Get (0), c.Get (1));
  NodeContainer mnxTR2 = NodeContainer (c.Get (0), c.Get (2));

  NodeContainer lispRouters = NodeContainer (c.Get(0), c.Get (1), c.Get(2), c.Get (3), c.Get(6));
  NodeContainer xTRNodes = NodeContainer(c.Get(0), c.Get (1), c.Get (2), c.Get (3));
  NodeContainer xTR1 = NodeContainer (c.Get (1));
  NodeContainer xTR2 = NodeContainer (c.Get (2));
  NodeContainer d1d5 = NodeContainer (c.Get (1), c.Get (5));
  NodeContainer d2d5 = NodeContainer (c.Get (2), c.Get (5));
  NodeContainer d5d6 = NodeContainer (c.Get (5), c.Get (6));
  NodeContainer d5d3 = NodeContainer (c.Get (5), c.Get (3));
  NodeContainer d3d4 = NodeContainer (c.Get (3), c.Get (4));

  // create channels without any IP addressing info
  NS_LOG_INFO("Create p2p channels");
  PointToPointHelper p2p;//if not pointer type, no need to use key word "new"
  p2p.SetDeviceAttribute ("DataRate", StringValue ("10Mbps"));
  p2p.SetChannelAttribute ("Delay", StringValue ("2ms"));

  NetDeviceContainer netd1d5 = p2p.Install (d1d5);
  NetDeviceContainer netd2d5 = p2p.Install (d2d5);
  NetDeviceContainer netd5d6 = p2p.Install (d5d6);
  NetDeviceContainer netd5d3 = p2p.Install (d5d3);
  NetDeviceContainer netd3d4 = p2p.Install (d3d4);

  // make all nodes be routers
  InternetStackHelper internet;
  internet.Install (c);
  // We add IP addresses
  Ipv4AddressHelper ipv4;
  NS_LOG_INFO("Assign IP Addresses.");

  ipv4.SetBase ("10.1.2.0", "255.255.255.0");
  Ipv4InterfaceContainer i1i5 = ipv4.Assign (netd1d5);

  ipv4.SetBase ("10.1.3.0", "255.255.255.0");
  Ipv4InterfaceContainer i2i5 = ipv4.Assign (netd2d5);

  ipv4.SetBase ("10.1.4.0", "255.255.255.0");
  Ipv4InterfaceContainer i5i6 = ipv4.Assign (netd5d6);

  ipv4.SetBase ("10.1.5.0", "255.255.255.0");
  Ipv4InterfaceContainer i5i3 = ipv4.Assign (netd5d3);

  ipv4.SetBase ("10.3.3.0", "255.255.255.0");
  Ipv4InterfaceContainer i3i4 = ipv4.Assign (netd3d4);

  NetDeviceContainer mnxTR1Devs = p2p.Install (mnxTR1);
  NetDeviceContainer mnxTR2Devs = p2p.Install (mnxTR2);

  Ptr<Ipv4> ipv4MN = c.Get (0)->GetObject<Ipv4> ();
  uint32_t ifIndex1 = ipv4MN->AddInterface (mnxTR1Devs.Get (0));
  ipv4MN->AddAddress (
      ifIndex1,
      Ipv4InterfaceAddress (Ipv4Address ("0.0.0.0"), Ipv4Mask ("/0")));
  ipv4MN->SetForwarding (ifIndex1, true);
  ipv4MN->SetUp (ifIndex1);

  ipv4MN = c.Get (0)->GetObject<Ipv4> ();
  ifIndex1 = ipv4MN->AddInterface (mnxTR2Devs.Get (0));
  ipv4MN->AddAddress (
      ifIndex1,
      Ipv4InterfaceAddress (Ipv4Address ("0.0.0.0"), Ipv4Mask ("/0")));
  ipv4MN->SetForwarding (ifIndex1, true);
//  ipv4MN->SetMetric(ifIndex1, 4);
  ipv4MN->SetUp (ifIndex1);

  /**
   * Set TUN/TAP device for LISP-MN node
   * A question: for tap, we need to care about Ethernet header or not?
   */
//  Address lispMnEidAddr = Ipv4Address ("172.16.0.1");
  Ptr<VirtualNetDevice> m_n0Tap = CreateObject<VirtualNetDevice> ();
  m_n0Tap->SetAddress (Mac48Address ("11:00:01:02:03:01"));
  c.Get (0)->AddDevice (m_n0Tap);
  Ptr<Ipv4> ipv4Tun = c.Get (0)->GetObject<Ipv4> ();
  uint32_t ifIndexTap = ipv4Tun->AddInterface (m_n0Tap);
  ipv4Tun->AddAddress (
      ifIndexTap,
      Ipv4InterfaceAddress (Ipv4Address::ConvertFrom(lispMnEidAddr),
			    Ipv4Mask ("255.255.255.255")));
  ipv4Tun->SetForwarding (ifIndexTap, true);
  ipv4Tun->SetUp (ifIndexTap);

  Ptr<VirtualNetDevice> m_n0Tap2 = CreateObject<VirtualNetDevice> ();
  m_n0Tap2->SetAddress (Mac48Address ("11:00:01:02:03:02"));
  c.Get (0)->AddDevice (m_n0Tap2);
  Ptr<Ipv4> ipv4Tun2 = c.Get (0)->GetObject<Ipv4> ();
  uint32_t ifIndexTap2 = ipv4Tun2->AddInterface (m_n0Tap2);
  ipv4Tun2->AddAddress (
      ifIndexTap2,
      Ipv4InterfaceAddress (Ipv4Address::ConvertFrom(lispMnEidAddr),
			    Ipv4Mask ("255.255.255.255")));
  ipv4Tun2->SetForwarding (ifIndexTap2, true);
  ipv4Tun2->SetUp (ifIndexTap2);
  /**
   * It is obligatory to set TransmitCallBack for virtual-net-device.
   * Otherwise when transmitting packet, virtual-net-device does not know what to do,
   * because it has not a physical NIC.
   */
  Fuck fuck (m_n0Tap, mnxTR1Devs.Get (0));
  Fuck fuck2 (m_n0Tap2, mnxTR2Devs.Get (0));

  /*
   * Assign a unique address: 10.1.1.254 for wifi net device on xTR1
   * code snippet fromm dhcp-example.cc
   */
  Ptr<Ipv4> ipv4xTR1 = c.Get (1)->GetObject<Ipv4> ();
  uint32_t ifIndex = ipv4xTR1->AddInterface (mnxTR1Devs.Get (1));
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
  uint32_t ifIndex2 = ipv4xTR2->AddInterface (mnxTR2Devs.Get (1));
  ipv4xTR2->AddAddress (
      ifIndex,
      Ipv4InterfaceAddress (Ipv4Address ("10.1.7.254"), Ipv4Mask ("/24")));
  ipv4xTR2->SetForwarding (ifIndex, true);
  ipv4xTR2->SetMetric (ifIndex, 1);
  ipv4xTR2->SetUp (ifIndex);
  Ipv4InterfaceContainer ap2Interface;
  ap2Interface.Add (ipv4xTR2, ifIndex2);

  // Crate router node before assigning @ip for xTR2 to avoid multiple path problem.
  NS_LOG_INFO("Populating routing table...");
  // For such a simple topology, we use static route instead of global routing.
  //Ipv4GlobalRoutingHelper::PopulateRoutingTables();
  sim->PopulateStaticRoutingTable2 (c, ifIndexTap);

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
  positionAlloc->Add (Vector (0.0, 120.0, 0.0)); // position xTR1
  positionAlloc->Add (Vector (120.0, 0.0, 0.0)); // position xTR2
  positionAlloc->Add (Vector (180, 180.0, 0.0)); // position xTR3
  positionAlloc->Add (Vector (240.0, 240.0, 0.0)); // position CN
  positionAlloc->Add (Vector (120.0, 120.0, 0.0)); // position intermiediate router
  positionAlloc->Add (Vector (180.0, 60.0, 0.0)); // position MR/MS
  mobility.SetPositionAllocator (positionAlloc);
  mobility.Install (c);

  Time END_T = Seconds (20.0);
  Time ECO_END_T = Seconds (45.5);
  Time START_T = Seconds (5.0);
  Time HandTime = Seconds(15.5);

  Simulator::Schedule(HandTime, &HandOver, c.Get(0), m_n0Tap, mnxTR2Devs.Get (0));
  /*
   * Now use DHCP server to allocate @Ip for MN
   * Due to DHCP program implementation constraint, we have to first assign a 0.0.0.0/0 for DHCP client
   */
  NS_LOG_INFO("Start DHCP server...");
  sim->InstallDhcpServerApplication (c.Get (1), Ipv4Address ("10.1.1.0"),
				Ipv4Mask ("/24"), ap1Interface.GetAddress (0),
				Ipv4Address ("10.1.1.0"),
				Ipv4Address ("10.1.1.252"),
				ap1Interface.GetAddress (0), Seconds (0.0),
				END_T);
  sim->InstallDhcpServerApplication (c.Get (2), Ipv4Address ("10.1.7.0"),
				Ipv4Mask ("/24"), ap2Interface.GetAddress (0),
				Ipv4Address ("10.1.7.0"),
				Ipv4Address ("10.1.7.252"),
				ap2Interface.GetAddress (0), Seconds (0.0),
				END_T);
  //Install DHCP client app at MN (i.e. node 0), has to be runned at the beginning...
  //ATTENTIO!!!: if use DHCP mode, you have to used the second static routing conf...!!!
  NS_LOG_INFO("Start DHCP client at MN...");
  sim->InstallDhcpClientApplication (c.Get (0), 1, Seconds (0.0), END_T);
  sim->InstallDhcpClientApplication (c.Get (0), 2, HandTime, END_T);

  sim->InstallEchoApplication (c.Get (4), c.Get (0), i3i4.GetAddress (1), 9, START_T,
			  ECO_END_T); // Discard port (RFC 863)

  // Make xTR1&2&3 as lisp-supported routers
  Ipv4Address mrAddr = i5i6.GetAddress (1);
  Ipv4Address msAddr = i5i6.GetAddress (1);

  // MR/MS are also lisp-speaking devices => install lisp but no xTR app. Instead mr/ms app.
  sim->InstallLispRouter (lispRouters);
  // Install xTR apps on xTR nodes.
  sim->InstallXtrApplication(xTRNodes, mrAddr, msAddr, Seconds (0.0), END_T);
  // Install lisp Mapping server at node 6. No map resolver in this simulation.
  sim->InstallMapServerApplication (c.Get (6), Seconds (0.0), END_T);

  AsciiTraceHelper ascii;
  p2p.EnableAsciiAll (
      ascii.CreateFileStream ("lisp-mobility-between-subnet.tr"));

//  wifiPhy.EnablePcap("lisp-mobility-between-subnet-wifi2", m_n0Tap);
  p2p.EnablePcapAll ("lisp-mobility-between-subnet-no-wifi");
  MobilityHelper::EnableAsciiAll (
      ascii.CreateFileStream ("lisp-mobility-between-subnet-no-wifi.mob"));

  // Flow Monitor
  FlowMonitorHelper flowmonHelper;
  if (true)
    {
      flowmonHelper.InstallAll ();
      flowmonHelper.SerializeToXmlFile("test.flowmon", false, false);
    }
  //CreateAnimFile(c, animFile);
  AnimationInterface anim (animFile);
  anim.SetConstantPosition (c.Get (0), 0, 110);
  anim.SetConstantPosition (c.Get (1), 0, 120);
  anim.SetConstantPosition (c.Get (2), 120, 0);
  anim.SetConstantPosition (c.Get (3), 180, 180);
  anim.SetConstantPosition (c.Get (4), 240, 240);
  anim.SetConstantPosition (c.Get (5), 120, 120);
  anim.SetConstantPosition (c.Get (6), 180, 60);
  anim.SetMobilityPollInterval (Seconds (0.25));
  anim.EnablePacketMetadata (true); // Optional
  anim.EnableIpv4L3ProtocolCounters (Seconds (0), END_T); // Optional
  anim.EnableIpv4RouteTracking ("lisp-mobility-routing-table-case2",
				Seconds (0), END_T);

  NS_LOG_INFO("Run Simulation.");
  // Set stop time before run simulation.
  Simulator::Stop (END_T);
  Simulator::Run ();
  NS_LOG_INFO("Simulation Done.");
  Simulator::Destroy ();

  return 0;
}
