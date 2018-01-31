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
#include "ns3/athstats-helper.h"

//#include "ns3/core-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("LispMobilityWithinSubNet");

static bool g_verbose = true;

void DevTxTrace(std::string context, Ptr<const Packet> p) {
	if (g_verbose) {
		std::cout << " TX p: " << *p << std::endl;
	}
}
void DevRxTrace(std::string context, Ptr<const Packet> p) {
	if (g_verbose) {
		std::cout << " RX p: " << *p << std::endl;
	}
}
void PhyRxOkTrace(std::string context, Ptr<const Packet> packet, double snr,
		WifiMode mode, enum WifiPreamble preamble) {
	if (g_verbose) {
		std::cout << "PHYRXOK mode=" << mode << " snr=" << snr << " " << *packet
				<< std::endl;
	}
}
void PhyRxErrorTrace(std::string context, Ptr<const Packet> packet,
		double snr) {
	if (g_verbose) {
		std::cout << "PHYRXERROR snr=" << snr << " " << *packet << std::endl;
	}
}
void PhyTxTrace(std::string context, Ptr<const Packet> packet, WifiMode mode,
		WifiPreamble preamble, uint8_t txPower) {
	if (g_verbose) {
		std::cout << "PHYTX mode=" << mode << " " << *packet << std::endl;
	}
}
void PhyStateTrace(std::string context, Time start, Time duration,
		enum WifiPhy::State state) {
	if (g_verbose) {
		std::cout << " state=" << state << " start=" << start << " duration="
				<< duration << std::endl;
	}
}

static void SetPosition(Ptr<Node> node, Vector position) {
	Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
	mobility->SetPosition(position);
}

static Vector GetPosition(Ptr<Node> node) {
	Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
	return mobility->GetPosition();
}

static void AdvancePosition(Ptr<Node> node) {
	Vector pos = GetPosition(node);
	pos.x += 5.0;
	if (pos.x >= 210.0) {
		return;
	}
	SetPosition(node, pos);

	if (g_verbose) {
		std::cout << "x=" << pos.x << std::endl;
	}
	Simulator::Schedule(Seconds(1.0), &AdvancePosition, node);
}

void ChangeDefautGateway(Ptr<Node> node, Ipv4Address gateway, uint32_t interface){
	// set defaut route for MN (in WiFi networks)
	Ipv4StaticRoutingHelper ipv4SrHelper;
	Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
	Ptr<Ipv4StaticRouting> ipv4Stat = ipv4SrHelper.GetStaticRouting(ipv4);
	ipv4Stat->RemoveRoute(2);
	ipv4Stat->SetDefaultRoute(gateway, interface);
	Ptr<OutputStreamWrapper> stream1 = Create<OutputStreamWrapper> ("Static Routing Table for MN", std::ios::out);
	ipv4Stat->PrintRoutingTable(stream1);
}

void AdjustStaticRoutingTable(NodeContainer c){
	// set defaut route for MN (in WiFi networks)
	Ipv4StaticRoutingHelper ipv4SrHelper;
	Ptr<Ipv4> ipv4 = c.Get(0)->GetObject<Ipv4>();
	Ptr<Ipv4StaticRouting> ipv4Stat = ipv4SrHelper.GetStaticRouting(ipv4);
	// Do not forget to remove previously set default gateway
	ipv4Stat->RemoveRoute(2);
	ipv4Stat->SetDefaultRoute(Ipv4Address ("10.1.1.3"), 1);

	ipv4 = c.Get(5)->GetObject<Ipv4>();
	ipv4Stat = ipv4SrHelper.GetStaticRouting(ipv4);
	// For node 5, to go to 10.1.1.0/24. It should pass through xTR2, from interface 2, next hop is 10.1.3.1
	ipv4Stat->AddNetworkRouteTo(Ipv4Address ("10.1.1.0"), Ipv4Mask ("255.255.255.0"), Ipv4Address ("10.1.3.1"), 2, 0);


	Ptr<OutputStreamWrapper> stream1 = Create<OutputStreamWrapper> ("Static Routing Table for MN", std::ios::out);
	ipv4Stat->PrintRoutingTable(stream1);
}

void PrintLocations(NodeContainer nodes, std::string header) {
	std::cout << header << std::endl;
	for (NodeContainer::Iterator iNode = nodes.Begin(); iNode != nodes.End();
			++iNode) {
		Ptr<Node> object = *iNode;
		Ptr<MobilityModel> position = object->GetObject<MobilityModel>();
		NS_ASSERT(position != 0);
		Vector pos = position->GetPosition();
		std::cout << "(" << pos.x << ", " << pos.y << ", " << pos.z << ")"
				<< std::endl;
	}
	std::cout << std::endl;
}

void PrintAddresses(Ipv4InterfaceContainer container, std::string header) {
	std::cout << header << std::endl;
	uint32_t nNodes = container.GetN();
	for (uint32_t i = 0; i < nNodes; ++i) {
		std::cout << container.GetAddress(i, 0) << std::endl;
	}
	std::cout << std::endl;
}

void InstallMapResolverApplication(Ptr<Node> node, Ipv4Address msAddress, Time start, Time end){
	// Initializing Map Resolver Ddt
	MapResolverDdtHelper mapResDdtHelp;
	mapResDdtHelp.SetMapServerAddress(static_cast<Address>(msAddress));
	ApplicationContainer mrDdtApps = mapResDdtHelp.Install(node);
	mrDdtApps.Start(start);
	mrDdtApps.Stop(end);
}

void InstallEchoApplication(Ptr<Node> echoServerNode, Ptr<Node> echoClientNode,
		Ipv4Address echoServerIpAddr, uint16_t port, Time start, Time end)
{
	// Make node 4 as Echo server, listening port 9
	UdpEchoServerHelper echoServer(port);
	ApplicationContainer serverApps = echoServer.Install(echoServerNode);
	serverApps.Start(start);
	serverApps.Stop(end);
	// Get @ip for net device of node and use it to initialize echo client
	//UdpEchoClientHelper echoClient(i3i4.GetAddress(1), 9);
	UdpEchoClientHelper echoClient(echoServerIpAddr, port);
	echoClient.SetAttribute("MaxPackets", UintegerValue(100));
	echoClient.SetAttribute("Interval", TimeValue(Seconds(1)));
	echoClient.SetAttribute("PacketSize", UintegerValue(512));
	// Install echo client app at node 0
	ApplicationContainer clientApps = echoClient.Install(echoClientNode);
	clientApps.Start(start);
	clientApps.Stop(end);
}

void InstallOnOffApplication(Ptr<Node> dstNode, Ptr<Node> srcNode,
		Ipv4Address dstIpAddr, uint16_t port, Time start, Time end)
{
	// Create the OnOff applications to send udp datagrams of size
	// 210 bytes at a rate 448 kb/s
	NS_LOG_INFO("Create OnOff Applications.");
	OnOffHelper onoff("ns3::UdpSocketFactory",
			Address(InetSocketAddress(dstIpAddr, port))); // to CN
	onoff.SetConstantRate(DataRate("448kb/s"));
	ApplicationContainer apps = onoff.Install(srcNode);// from MN
	apps.Start(start);
	apps.Stop(end);
	// Create a packet sink to receive these packets at CN node
	PacketSinkHelper sink("ns3::UdpSocketFactory",
			Address(InetSocketAddress(Ipv4Address::GetAny(), port)));
	apps = sink.Install(dstNode);
	apps.Start(start);
	apps.Stop(end);
}

void InstallMapServerApplication(Ptr<Node> node, Time start, Time end){
	// initializing Map Server ddt helper
	MapServerDdtHelper mapServerDdtHelp;
	ApplicationContainer mapServDdtApps = mapServerDdtHelp.Install(node);
	mapServDdtApps.Start(start);
	mapServDdtApps.Stop(end);
}

void EnablePhyMacTraces(){
	//TODO I don't know what the following instructions want to do...
	Config::Connect("/NodeList/*/DeviceList/*/Mac/MacTx",
			MakeCallback(&DevTxTrace));
	Config::Connect("/NodeList/*/DeviceList/*/Mac/MacRx",
			MakeCallback(&DevRxTrace));
	Config::Connect("/NodeList/*/DeviceList/*/Phy/State/RxOk",
			MakeCallback(&PhyRxOkTrace));
	Config::Connect("/NodeList/*/DeviceList/*/Phy/State/RxError",
			MakeCallback(&PhyRxErrorTrace));
	Config::Connect("/NodeList/*/DeviceList/*/Phy/State/Tx",
			MakeCallback(&PhyTxTrace));
	Config::Connect("/NodeList/*/DeviceList/*/Phy/State/State",
			MakeCallback(&PhyStateTrace));
}

void InstallLispRouter(NodeContainer nodes, Ipv4Address mrAddress, Ipv4Address msAddress, Time start, Time end){
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
	NS_LOG_INFO("Install Lisp");
	LispHelper lispHelper;
	lispHelper.BuildRlocsSet(
			"/home/qsong/Documents/ns-3.24/src/internet/examples/lisp/mobility_within_network/rlocs.txt");
	lispHelper.Install(nodes);
	lispHelper.BuildMapTables2(
			"/home/qsong/Documents/ns-3.24/src/internet/examples/lisp/mobility_within_network/rloc_config_xml.txt");
	lispHelper.InstallMapTables(nodes);
	NS_LOG_WARN("Example: Lisp is successfully aggregated");

	LispEtrItrAppHelper lispAppHelper;
	Ptr<Locator> mrLocator = Create<Locator>(mrAddress);
	lispAppHelper.AddMapResolverRlocs(mrLocator);
	lispAppHelper.AddMapServerAddress(static_cast<Address>(msAddress));
	ApplicationContainer mapResClientApps = lispAppHelper.Install(nodes);
	mapResClientApps.Start(start);
	mapResClientApps.Stop(end);
}

void PopulateStaticRoutingTable(NodeContainer c){
	// set defaut route for MN (in WiFi networks)
	Ipv4StaticRoutingHelper ipv4SrHelper;
	Ptr<Ipv4> ipv4 = c.Get(0)->GetObject<Ipv4>();
	Ptr<Ipv4StaticRouting> ipv4Stat = ipv4SrHelper.GetStaticRouting(ipv4);
	// Take the address of xTR1 as defaut gateway.
	ipv4Stat->SetDefaultRoute(Ipv4Address("10.1.1.2"), 1);
	//ipv4Stat->AddNetworkRouteTo(Ipv4Address ("10.3.3.0"), Ipv4Mask ("255.255.255.0"), Ipv4Address ("10.1.1.2"), 1, 0);

	// Populate routing table for xTR1
	ipv4 = c.Get(1)->GetObject<Ipv4>();
	ipv4Stat = ipv4SrHelper.GetStaticRouting(ipv4);
	ipv4Stat->SetDefaultRoute(Ipv4Address("10.1.2.2"), 1);

	// For xTR2 (node 2)
	ipv4 = c.Get(2)->GetObject<Ipv4>();
	ipv4Stat = ipv4SrHelper.GetStaticRouting(ipv4);
	ipv4Stat->SetDefaultRoute(Ipv4Address("10.1.3.2"), 1);

	// For xTR3 (node 3)
	ipv4 = c.Get(3)->GetObject<Ipv4>();
	ipv4Stat = ipv4SrHelper.GetStaticRouting(ipv4);
	ipv4Stat->SetDefaultRoute(Ipv4Address("10.1.5.1"), 1);

	// For CN (node 4)
	ipv4 = c.Get(4)->GetObject<Ipv4>();
	ipv4Stat = ipv4SrHelper.GetStaticRouting(ipv4);
	ipv4Stat->SetDefaultRoute(Ipv4Address("10.1.3.1"), 1);

	// For intermediate router node 5
	ipv4 = c.Get(5)->GetObject<Ipv4>();
	ipv4Stat = ipv4SrHelper.GetStaticRouting(ipv4);
	//ipv4Stat->AddNetworkRouteTo(Ipv4Address ("10.1.1.0"), Ipv4Mask ("255.255.255.0"), Ipv4Address ("10.1.2.1"), 1, 0);
	ipv4Stat->AddNetworkRouteTo(Ipv4Address ("10.1.6.0"), Ipv4Mask ("255.255.255.0"), Ipv4Address ("10.1.4.2"), 3, 0);
	//ipv4Stat->AddNetworkRouteTo(Ipv4Address ("10.3.3.0"), Ipv4Mask ("255.255.255.0"), Ipv4Address ("10.1.5.2"), 4, 0);

	// For MR,(node 6)
	ipv4 = c.Get(6)->GetObject<Ipv4>();
	ipv4Stat = ipv4SrHelper.GetStaticRouting(ipv4);
	ipv4Stat->SetDefaultRoute(Ipv4Address("10.1.4.1"), 1);

	// For MS (node 7)
	ipv4 = c.Get(7)->GetObject<Ipv4>();
	ipv4Stat = ipv4SrHelper.GetStaticRouting(ipv4);
	ipv4Stat->SetDefaultRoute(Ipv4Address("10.1.6.1"), 1);
}

void CreateAnimFile(NodeContainer c, std::string animFile){
	// Create the animation object and configure for specified output
	AnimationInterface anim(animFile);
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
	anim.SetConstantPosition(c.Get(0), 0, 110);
	anim.SetConstantPosition(c.Get(1), 0, 120);
	anim.SetConstantPosition(c.Get(2), 120, 0);
	anim.SetConstantPosition(c.Get(3), 180, 180);
	anim.SetConstantPosition(c.Get(4), 240, 240);
	anim.SetConstantPosition(c.Get(5), 120, 120);
	anim.SetConstantPosition(c.Get(6), 180, 60);
	anim.SetConstantPosition(c.Get(7), 240, 0);
	anim.EnablePacketMetadata(true); // Optional
	anim.EnableIpv4L3ProtocolCounters(Seconds(0), Seconds(10)); // Optional
}

int main(int argc, char *argv[]) {
	LogComponentEnable("LispMobilityWithinSubNet", LOG_LEVEL_ALL);
	CommandLine cmd;
	cmd.AddValue("verbose", "Print trace information if true", g_verbose);
	cmd.Parse(argc, argv);
	g_verbose = true;
    if (g_verbose)
    {	//TODO: Add Log Component for lisp-related class.
        LogComponentEnable("UdpEchoClientApplication", LOG_LEVEL_INFO);
        LogComponentEnable("UdpEchoServerApplication", LOG_LEVEL_INFO);
        LogComponentEnable("OnOffApplication", LOG_LEVEL_INFO);
    }

	std::string animFile = "lisp-mobility-within-subnet.xml"; // Name of file for animation output
	Packet::EnablePrinting();

	// enable rts cts all the time.
	Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold",
			StringValue("2200"));
	// disable fragmentation
	Config::SetDefault("ns3::WifiRemoteStationManager::FragmentationThreshold",
			StringValue("2200"));
	Config::SetDefault("ns3::Ipv4GlobalRouting::RandomEcmpRouting",
	BooleanValue(true));

	NS_LOG_INFO("Create nodes.");
	NodeContainer c;
	c.Create(8);

	// Scenario: one STA in network 2, managed by 2 APs in network 1,
	// wants to communicate with one PC in another network 2 managed by one P2P link
	// LISP Mapping system have one MR and one MS
	// Except link between stats and aps, other links are p2p-based
	NodeContainer mn = NodeContainer(c.Get(0));
	NodeContainer xTR1 = NodeContainer(c.Get(1));
	NodeContainer xTR2 = NodeContainer(c.Get(2));
	NodeContainer d1d5 = NodeContainer(c.Get(1), c.Get(5));
	NodeContainer d2d5 = NodeContainer(c.Get(2), c.Get(5));
	NodeContainer d5d6 = NodeContainer(c.Get(5), c.Get(6));
	NodeContainer d5d3 = NodeContainer(c.Get(5), c.Get(3));
	NodeContainer d3d4 = NodeContainer(c.Get(3), c.Get(4));
	NodeContainer d6d7 = NodeContainer(c.Get(6), c.Get(7));

	// create channels without any IP addressing info
	NS_LOG_INFO("Create p2p channels");
	PointToPointHelper p2p;
	p2p.SetDeviceAttribute("DataRate", StringValue("5Mbps"));
	p2p.SetChannelAttribute("Delay", StringValue("2ms"));

	NetDeviceContainer netd1d5 = p2p.Install(d1d5);
	NetDeviceContainer netd2d5 = p2p.Install(d2d5);
	NetDeviceContainer netd5d6 = p2p.Install(d5d6);
	NetDeviceContainer netd5d3 = p2p.Install(d5d3);
	NetDeviceContainer netd3d4 = p2p.Install(d3d4);
	NetDeviceContainer netd6d7 = p2p.Install(d6d7);

	// make all nodes be routers
	InternetStackHelper internet;
	internet.Install(c);
	// We add IP addresses
	Ipv4AddressHelper ipv4;
	NS_LOG_INFO("Assign IP Addresses.");

	ipv4.SetBase("10.1.2.0", "255.255.255.0");
	Ipv4InterfaceContainer i1i5 = ipv4.Assign(netd1d5);
	// Set link metrics to make sure that xTR1 is the best choice for prefix 10.1.1.0/24
	i1i5.SetMetric(0,1);
	i1i5.SetMetric(1,1);

	ipv4.SetBase("10.1.3.0", "255.255.255.0");
	Ipv4InterfaceContainer i2i5 = ipv4.Assign(netd2d5);
	i2i5.SetMetric(0,3);
	i2i5.SetMetric(1,3);

	ipv4.SetBase("10.1.4.0", "255.255.255.0");
	Ipv4InterfaceContainer i5i6 = ipv4.Assign(netd5d6);

	ipv4.SetBase("10.1.5.0", "255.255.255.0");
	Ipv4InterfaceContainer i5i3 = ipv4.Assign(netd5d3);

	ipv4.SetBase("10.1.6.0", "255.255.255.0");
	Ipv4InterfaceContainer i6i7 = ipv4.Assign(netd6d7);

	ipv4.SetBase("10.3.3.0", "255.255.255.0");
	Ipv4InterfaceContainer i3i4 = ipv4.Assign(netd3d4);

	// configure wifi PHY layer and channel helper
	//TODO maybe should add path-loss and rss level? Otherwise how STA change the attached AP?
	YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
	YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
//	wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
//	wifiChannel.AddPropagationLoss("ns3::FriisPropagationLossModel");
	wifiPhy.SetChannel(wifiChannel.Create());

	WifiHelper wifi = WifiHelper::Default();
	// set rate control algo.
	wifi.SetRemoteStationManager("ns3::ArfWifiManager");
	// use no-Qos MAC
	NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default();
	// Ad-hoc wifi
	wifiMac.SetType("ns3::AdhocWifiMac");
	// wifi mac setup for mn(i.e. STA in wifi's terminology).
//	Ssid ssid = Ssid("xTR1");
//	wifiMac.SetType(
//			"ns3::StaWifiMac",
//			"Ssid", SsidValue(ssid), // need to tell STA the target SSID
//			"ActiveProbing", BooleanValue(false) //make sure that STA does not perform active probing
//	);

	//Once all station specific parameters are configured, create wifi devices of STA
	NetDeviceContainer mnDevs;
	mnDevs = wifi.Install(wifiPhy, wifiMac, mn);
	// wifi mac setup for the involved xTRs (i.e. access point in wifi's terminology).
	// change the defaut attributes of NqosWifiMacHelper to reflect the requirements of the xTR (i.e. AP)
//	wifiMac.SetType(
//			"ns3::ApWifiMac",		// Non-Qos Access Point Type
//			"Ssid", SsidValue(ssid), "BeaconGeneration", BooleanValue(true),
//			"BeaconInterval", TimeValue(Seconds(2.5)));

	NetDeviceContainer xTRDev1, xTRDev2;
	xTRDev1 = wifi.Install(wifiPhy, wifiMac, xTR1);
	xTRDev2 = wifi.Install(wifiPhy, wifiMac, xTR2);

	ipv4.SetBase("10.1.1.0", "255.255.255.0");
	Ipv4InterfaceContainer staInterface = ipv4.Assign(mnDevs);
	Ipv4InterfaceContainer ap1Interface = ipv4.Assign(xTRDev1);

	// Create router node before assigning @ip for xTR2 to avoid multiple path problem.
	NS_LOG_INFO("Populating routing table...");
	// For such a simple topology, we use static route instead of global routing.
	//Ipv4GlobalRoutingHelper::PopulateRoutingTables();

	Ipv4InterfaceContainer ap2Interface = ipv4.Assign(xTRDev2);

	PopulateStaticRoutingTable(c);

	Ipv4GlobalRoutingHelper helper;
	NS_LOG_INFO("Print routing table...");
	Ptr<OutputStreamWrapper> stream1 = Create<OutputStreamWrapper> ("Table2", std::ios::out);
	helper.PrintRoutingTableAllAt(Seconds(5.0), stream1);
	// Add mobility model.
	// MN (i.e. STA) is mobile, wandering around inside a bouding box.
	// Other nodes are stationary.
	MobilityHelper mobility;
	Ptr<ListPositionAllocator> positionAlloc = CreateObject<
			ListPositionAllocator>();
	positionAlloc->Add(Vector(0.0, 110.0, 0.0)); // position MN
//	positionAlloc->Add(Vector(110, 0.0, 0.0)); // position MN
	positionAlloc->Add(Vector(0.0, 120.0, 0.0)); // position xTR1
	positionAlloc->Add(Vector(120.0, 0.0, 0.0)); // position xTR2
	positionAlloc->Add(Vector(180, 180.0, 0.0)); // position xTR3
	positionAlloc->Add(Vector(240.0, 240.0, 0.0));// position CN
	positionAlloc->Add(Vector(120.0, 120.0, 0.0));// position intermiediate router
	positionAlloc->Add(Vector(180.0, 60.0, 0.0)); // position MR
	positionAlloc->Add(Vector(240.0, 0.0, 0.0)); //position MS
	mobility.SetPositionAllocator(positionAlloc);
	mobility.Install(c);
    PrintLocations(c, "Location of all nodes");
	//Simulator::Schedule(Seconds(2.5), &SetPosition, mn.Get(0), Vector(110, 0.0, 0.0));
	// 10.1.1.3 is at interface with index 2
	//Simulator::Schedule(Seconds(2.5), &AdjustStaticRoutingTable, c);

    Time END_T = Seconds(20.0);
	Time START_T = Seconds(1.0);

//	InstallOnOffApplication(c.Get(4), c.Get(0), i3i4.GetAddress(1), 9, START_T, END_T); // Discard port (RFC 863)
	InstallEchoApplication(c.Get(4), c.Get(0), i3i4.GetAddress(1), 9, START_T, END_T); // Discard port (RFC 863)

	// Make xTR1&2&3 as lisp-supported routers
	Ipv4Address msAddr = i6i7.GetAddress(1);
	Ipv4Address mrAddr = i5i6.GetAddress(1);
	NodeContainer lispRouters = NodeContainer(c.Get(1), c.Get(3));
	InstallLispRouter(lispRouters, mrAddr, msAddr, START_T, END_T);
	// Install Map Resolver at node 6
	InstallMapResolverApplication(c.Get(6), msAddr, START_T, END_T);
	// Install lisp Mapping server at node 7
	InstallMapServerApplication(c.Get(7), START_T, END_T);


//	EnablePhyMacTraces();
	//TODO: The following code make program crash with unknown (at least for me) reason. Comment it.
//	AthstatsHelper athstats;
//	athstats.EnableAthstats("athstats-mn", mn);
//	athstats.EnableAthstats("athstats-ap", xTR1);

	AsciiTraceHelper ascii;
	p2p.EnableAsciiAll(ascii.CreateFileStream("lisp-mobility-within-subnet.tr"));
    wifiPhy.EnablePcapAll ("lisp-mobility-within-subnet-wifi", true);
	p2p.EnablePcapAll("lisp-mobility-within-subnet");
    MobilityHelper::EnableAsciiAll (ascii.CreateFileStream ("lisp-mobility-within-subnet.mob"));

	// Flow Monitor
	FlowMonitorHelper flowmonHelper;
	if (true) {
		flowmonHelper.InstallAll();
	}
    //CreateAnimFile(c, animFile);
	AnimationInterface anim(animFile);
	anim.SetConstantPosition(c.Get(0), 0, 110);
	anim.SetConstantPosition(c.Get(1), 0, 120);
	anim.SetConstantPosition(c.Get(2), 120, 0);
	anim.SetConstantPosition(c.Get(3), 180, 180);
	anim.SetConstantPosition(c.Get(4), 240, 240);
	anim.SetConstantPosition(c.Get(5), 120, 120);
	anim.SetConstantPosition(c.Get(6), 180, 60);
	anim.SetConstantPosition(c.Get(7), 240, 0);
	anim.SetMobilityPollInterval(Seconds(0.25));
	anim.EnablePacketMetadata(true); // Optional
	anim.EnableIpv4L3ProtocolCounters(Seconds(0), END_T); // Optional
	anim.EnableIpv4RouteTracking("lisp-mobility-routing-table", Seconds(0), END_T);

	NS_LOG_INFO("Run Simulation.");
	// Set stop time before run simulation.
	Simulator::Stop(END_T);
	Simulator::Run();
	NS_LOG_INFO("Simulation Done.");
	Simulator::Destroy();

	return 0;
}
