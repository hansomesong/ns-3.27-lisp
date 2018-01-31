/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

/*
 * RUN Command:
 * NS_LOG="LispOverIpv4Impl:LispProtocol:SimpleMapTables:LispHelper=level_all|prefix_node:Ipv4L3Protocol=level_debug|prefix_node|prefix_level" ./waf --run lisp_example_1
 *
 * RUN DEBUG Command:
 * NS_LOG="LispOverIpv4Impl:SimpleMapTables:LispHelper=level_all|prefix_node:Ipv4L3Protocol=level_debug|prefix_node|prefix_level" ./waf --run lisp_example_1 --command-template="gdb --args %s <args>"
 *
 * Network topology
 *  Es1 (0)          MR1 (11)--- MS1 (12)
 *    \               \           \
 *     \               Rt (4)----Rd2 (7)
 *      \             /          |     \
 *      Rs1 (1)------Rx (5)      |      \
 *       |                       |       \
 *      Rs2 (2)-----Ry (6)------Rd1 (8)---Ed2 (9)
 *      /                        \
 *     /                          \
 *    Es2 (3)                     Ed1 (10)
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
#include "ns3/map-resolver-helper.h"
#include "ns3/map-server-helper.h"
#include "ns3/lisp-etr-itr-app-helper.h"
#include "ns3/wifi-module.h"
#include "ns3/wifi-mac.h"
#include "ns3/virtual-net-device.h"
#include "ns3/lisp-header.h"

//#include "ns3/core-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("LispExample");

class Tunnel {
	Ptr<Socket> m_dstSocket;
	Ptr<Socket> m_srcSocket;
	Ipv4Address m_dstAddress;
	Ipv4Address m_srcAddress;
	Ptr<VirtualNetDevice> m_srcTap;
	Ptr<VirtualNetDevice> m_dstTap;

	bool SrcVirtualSend(Ptr<Packet> packet, const Address& source,
			const Address& dest, uint16_t protocolNumber) {
		// packet is IP packet with EID as src/dst IP address
		NS_LOG_DEBUG("Send to " << m_dstAddress << ": " << *packet);
		// Add a LISP header
		LispHeader lispHeader = LispHeader ();
		packet->AddHeader(lispHeader);
		m_srcSocket->SendTo(packet, 0, InetSocketAddress(m_dstAddress, 4341));
		return true;
	}

	bool DstirtualSend(Ptr<Packet> packet, const Address& source,
			const Address& dest, uint16_t protocolNumber) {
		NS_LOG_DEBUG("Send to " << m_srcAddress << ": " << *packet);
		m_dstSocket->SendTo(packet, 0, InetSocketAddress(m_srcAddress, 4341));
		return true;
	}

	void DstSocketRecv(Ptr<Socket> socket) {
		Ptr<Packet> packet = socket->Recv(65535, 0);
		NS_LOG_DEBUG("N3SocketRecv: " << *packet);
		m_dstTap->Receive(packet, 0x0800, m_dstTap->GetAddress(),
				m_dstTap->GetAddress(), NetDevice::PACKET_HOST);
	}

	void SrcSocketRecv(Ptr<Socket> socket) {
		Ptr<Packet> packet = socket->Recv(65535, 0);
		NS_LOG_DEBUG("N0SocketRecv: " << *packet);
		m_srcTap->Receive(packet, 0x0800, m_srcTap->GetAddress(),
				m_srcTap->GetAddress(), NetDevice::PACKET_HOST);
	}

public:

	Tunnel(Ptr<Node> n_Dst, Ptr<Node> n_Src, Ipv4Address n_DstAddr, Ipv4Address n_SrcAddr) :
			m_dstAddress(n_DstAddr), m_srcAddress(n_SrcAddr) {
		m_dstSocket = Socket::CreateSocket(n_Dst,
				TypeId::LookupByName("ns3::UdpSocketFactory"));
		m_dstSocket->Bind(InetSocketAddress(Ipv4Address::GetAny(), 4341));
		m_dstSocket->SetRecvCallback(
				MakeCallback(&Tunnel::DstSocketRecv, this));

		m_srcSocket = Socket::CreateSocket(n_Src,
				TypeId::LookupByName("ns3::UdpSocketFactory"));
		m_srcSocket->Bind(InetSocketAddress(Ipv4Address::GetAny(), 4341));
		m_srcSocket->SetRecvCallback(
				MakeCallback(&Tunnel::SrcSocketRecv, this));

		// n0 tap device
		m_srcTap = CreateObject<VirtualNetDevice>();
		m_srcTap->SetAddress(Mac48Address("11:00:01:02:03:01"));
		m_srcTap->SetSendCallback(MakeCallback(&Tunnel::SrcVirtualSend, this));
		n_Src->AddDevice(m_srcTap);
		Ptr<Ipv4> ipv4 = n_Src->GetObject<Ipv4>();
		uint32_t i = ipv4->AddInterface(m_srcTap);
		ipv4->AddAddress(i,
				Ipv4InterfaceAddress(Ipv4Address("192.168.0.1"),
						Ipv4Mask("255.255.255.0")));
		ipv4->SetUp(i);

		// n3 tap device
		m_dstTap = CreateObject<VirtualNetDevice>();
		m_dstTap->SetAddress(Mac48Address("11:00:01:02:03:04"));
		m_dstTap->SetSendCallback(MakeCallback(&Tunnel::DstirtualSend, this));
		n_Dst->AddDevice(m_dstTap);
		ipv4 = n_Dst->GetObject<Ipv4>();
		i = ipv4->AddInterface(m_dstTap);
		ipv4->AddAddress(i,
				Ipv4InterfaceAddress(Ipv4Address("192.168.0.254"),
						Ipv4Mask("255.255.255.0")));
		ipv4->SetUp(i);

	}
};

int main(int argc, char *argv[]) {
	Time END_T = Seconds(3.0);
	bool verbose = true;
	//std::string phyRate = "HtMcs7"; /* Physical layer bitrate. */

	std::string animFile = "lisp-animation.xml"; // Name of file for animation output

	LogComponentEnable("LispExample", LOG_LEVEL_ALL);
	NS_LOG_INFO("In the simulation... Good start!");
	// Set up some default values for the simulation.  Use the
	Config::SetDefault("ns3::OnOffApplication::PacketSize", UintegerValue(210));
	Config::SetDefault("ns3::OnOffApplication::DataRate",
			StringValue("448kb/s"));

	CommandLine cmd;
	bool enableFlowMonitor = false;
	cmd.AddValue("EnableMonitor", "Enable Flow Monitor", enableFlowMonitor);
	cmd.AddValue("verbose", "Tell application to log if true", verbose);
	cmd.AddValue("animFile", "File Name for Animation Output", animFile);
	cmd.Parse(argc, argv);

//	WifiMacHelper wifiMac;
//	WifiHelper wifiHelper;
//	wifiHelper.SetStandard(WIFI_PHY_STANDARD_80211n_5GHZ);
//
//	/* Set up Legacy Channel */
//	YansWifiChannelHelper wifiChannel;
//	wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
//	wifiChannel.AddPropagationLoss("ns3::FriisPropagationLossModel",
//			"Frequency", DoubleValue(5e9));
//
//	/* Setup Physical Layer */
//	YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
//	wifiPhy.SetChannel(wifiChannel.Create());
//	wifiPhy.Set("TxPowerStart", DoubleValue(10.0));
//	wifiPhy.Set("TxPowerEnd", DoubleValue(10.0));
//	wifiPhy.Set("TxPowerLevels", UintegerValue(1));
//	wifiPhy.Set("TxGain", DoubleValue(0));
//	wifiPhy.Set("RxGain", DoubleValue(0));
//	wifiPhy.Set("RxNoiseFigure", DoubleValue(10));
//	wifiPhy.Set("CcaMode1Threshold", DoubleValue(-79));
//	wifiPhy.Set("EnergyDetectionThreshold", DoubleValue(-79 + 3));
//	wifiPhy.SetErrorRateModel("ns3::YansErrorRateModel");
//	wifiHelper.SetRemoteStationManager("ns3::ConstantRateWifiManager",
//			"DataMode", StringValue(phyRate), "ControlMode",
//			StringValue("HtMcs0"));
//
//	NodeContainer networkNodes;
//	networkNodes.Create(2);
//	Ptr<Node> apWifiNode = networkNodes.Get(0);
//	Ptr<Node> staWifiNode = networkNodes.Get(1);
//
//	/* Configure AP */
//	Ssid ssid = Ssid("network");
//	wifiMac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
//
//	NetDeviceContainer apDevice;
//	apDevice = wifiHelper.Install(wifiPhy, wifiMac, apWifiNode);
//
//	/* Configure STA */
//	wifiMac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));
//
//	NetDeviceContainer staDevices;
//	staDevices = wifiHelper.Install(wifiPhy, wifiMac, staWifiNode);
	NS_LOG_INFO("Create nodes.");
	NodeContainer c;
	c.Create(13);
	NodeContainer es1rs1 = NodeContainer(c.Get(0), c.Get(1));
	NodeContainer rs1rs2 = NodeContainer(c.Get(1), c.Get(2));
	NodeContainer rs2es2 = NodeContainer(c.Get(2), c.Get(3));
	NodeContainer rtrx = NodeContainer(c.Get(4), c.Get(5));
	NodeContainer rs1rx = NodeContainer(c.Get(1), c.Get(5));
	NodeContainer rs2ry = NodeContainer(c.Get(2), c.Get(6));
	NodeContainer rtrd2 = NodeContainer(c.Get(4), c.Get(7));
	NodeContainer ryrd1 = NodeContainer(c.Get(6), c.Get(8));
	NodeContainer rd2ed2 = NodeContainer(c.Get(7), c.Get(9));
	NodeContainer rd1ed2 = NodeContainer(c.Get(8), c.Get(9));
	NodeContainer rd1ed1 = NodeContainer(c.Get(8), c.Get(10));
	NodeContainer rtmr = NodeContainer(c.Get(4), c.Get(11));
	NodeContainer mrms = NodeContainer(c.Get(11), c.Get(12));
	NodeContainer rd2ms = NodeContainer(c.Get(7), c.Get(12));

	// make them all routers
	InternetStackHelper internet;
	internet.Install(c);

	// create channels without any IP addressing info
	NS_LOG_INFO("Create channels");
	PointToPointHelper p2p;
	p2p.SetDeviceAttribute("DataRate", StringValue("5Mbps"));
	p2p.SetChannelAttribute("Delay", StringValue("2ms"));

	NetDeviceContainer d0d1 = p2p.Install(es1rs1);
	NetDeviceContainer d1d2 = p2p.Install(rs1rs2);
	NetDeviceContainer d2d3 = p2p.Install(rs2es2);
	NetDeviceContainer d4d5 = p2p.Install(rtrx);
	NetDeviceContainer d1d5 = p2p.Install(rs1rx);
	NetDeviceContainer d2d6 = p2p.Install(rs2ry);
	NetDeviceContainer d4d7 = p2p.Install(rtrd2);
	NetDeviceContainer d6d8 = p2p.Install(ryrd1);
	NetDeviceContainer d7d9 = p2p.Install(rd2ed2);
	NetDeviceContainer d8d9 = p2p.Install(rd1ed2);
	NetDeviceContainer d8d10 = p2p.Install(rd1ed1);
	NetDeviceContainer d4d11 = p2p.Install(rtmr);
	NetDeviceContainer d11d12 = p2p.Install(mrms);
	NetDeviceContainer d7d12 = p2p.Install(rd2ms);

	// We add IP addresses
	NS_LOG_INFO("Assign IP Addresses.");
	Ipv4AddressHelper ipv4;
	Ipv4AddressHelper ipv4_alt;
	ipv4.SetBase("10.1.1.0", "255.255.255.0");
	ipv4_alt.SetBase("192.168.1.0", "255.255.255.0");
	Ipv4InterfaceContainer i0i1 = ipv4.Assign(d0d1);
	//Ipv4InterfaceContainer i0i1 = ipv4.Assign(d0d1);
	ipv4.SetBase("10.1.2.0", "255.255.255.0");
	Ipv4InterfaceContainer i1i2 = ipv4.Assign(d1d2);

	ipv4.SetBase("10.1.3.0", "255.255.255.0");
	Ipv4InterfaceContainer i2i3 = ipv4.Assign(d2d3);

	ipv4.SetBase("10.1.4.0", "255.255.255.0");
	Ipv4InterfaceContainer i4i5 = ipv4.Assign(d4d5);

	ipv4.SetBase("10.1.5.0", "255.255.255.0");
	Ipv4InterfaceContainer i1i5 = ipv4.Assign(d1d5);

	ipv4.SetBase("10.1.6.0", "255.255.255.0");
	Ipv4InterfaceContainer i2i6 = ipv4.Assign(d2d6);

	ipv4.SetBase("10.1.7.0", "255.255.255.0");
	Ipv4InterfaceContainer i4i7 = ipv4.Assign(d4d7);

	ipv4.SetBase("10.1.8.0", "255.255.255.0");
	Ipv4InterfaceContainer i6i8 = ipv4.Assign(d6d8);

	ipv4.SetBase("10.1.9.0", "255.255.255.0");
	Ipv4InterfaceContainer i7i9 = ipv4.Assign(d7d9);

	ipv4.SetBase("10.1.10.0", "255.255.255.0");
	Ipv4InterfaceContainer i8i9 = ipv4.Assign(d8d9);

	ipv4.SetBase("10.1.11.0", "255.255.255.0");
	Ipv4InterfaceContainer i8i10 = ipv4.Assign(d8d10);

	// MAP RESOLVER
	ipv4.SetBase("10.1.12.0", "255.255.255.0");
	Ipv4InterfaceContainer i4i11 = ipv4.Assign(d4d11);

	// MR and MS
	ipv4.SetBase("10.1.13.0", "255.255.255.0");
	Ipv4InterfaceContainer i11i12 = ipv4.Assign(d11d12);

	// RD2 and MS
	ipv4.SetBase("10.1.14.0", "255.255.255.0");
	Ipv4InterfaceContainer i7i12 = ipv4.Assign(d7d12);

	// Create router node
	Ipv4GlobalRoutingHelper::PopulateRoutingTables();

	// finally install lisp
	NS_LOG_INFO("Install Lisp");
	LispHelper lispHelper;
	lispHelper.BuildRlocsSet(
			"/home/qsong/Documents/ns-3.24/src/internet/examples/lisp/rlocs.txt");
	NodeContainer lispRouters = NodeContainer(c.Get(1), c.Get(2), c.Get(8));
	//NodeContainer lispNodes = NodeContainer (c.Get (1), c.Get (2), c.Get (8), c.Get (11));
	lispHelper.Install(lispRouters);
	lispHelper.BuildMapTables2(
			"/home/qsong/Documents/ns-3.24/src/internet/examples/lisp/lisp_rloc_config_xml2.txt");
	lispHelper.InstallMapTables(lispRouters);

	NS_LOG_WARN("Example: Lisp is successfully aggregated");

	// Initializing Map Resolver Ddt
	MapResolverDdtHelper mapResDdtHelp;
	mapResDdtHelp.SetMapServerAddress(
			static_cast<Address>(Ipv4Address("10.1.13.2")));
	ApplicationContainer mrDdtApps = mapResDdtHelp.Install(c.Get(11));
	mrDdtApps.Start(Seconds(1.0));
	mrDdtApps.Stop(END_T);

	LispEtrItrAppHelper lispAppHelper;
	Ptr<Locator> msLocator = Create<Locator>(i4i11.GetAddress(1));
	lispAppHelper.AddMapResolverRlocs(msLocator);
	lispAppHelper.AddMapServerAddress(
			static_cast<Address>(Ipv4Address("10.1.13.2")));
	ApplicationContainer mapResClientApps = lispAppHelper.Install(lispRouters);
	mapResClientApps.Start(Seconds(1.0));
	mapResClientApps.Stop(END_T);

	// initializing Map Server ddt helper
	MapServerDdtHelper mapServerDdtHelp;
	ApplicationContainer mapServDdtApps = mapServerDdtHelp.Install(c.Get(12));
	mapServDdtApps.Start(Seconds(1.0));
	mapServDdtApps.Stop(END_T);


	// Create IP-Tunnel
	Tunnel tunnel (c.Get(9), c.Get (0), i8i9.GetAddress(1), i0i1.GetAddress (0));



	// Create the OnOff applications to send udp datagrams of size
	// 210 bytes at a rate 448 kb/s
	NS_LOG_INFO("Create Applications.");
	uint16_t port = 9;   // Discard port (RFC 863)
//	OnOffHelper onoff("ns3::UdpSocketFactory",
//			Address(InetSocketAddress(i8i9.GetAddress(1), port))); // to Ed2
	OnOffHelper onoff("ns3::UdpSocketFactory",
			Address(InetSocketAddress(Ipv4Address ("192.168.0.254"), port))); // to Ed2
	onoff.SetConstantRate(DataRate("448kb/s"));
	ApplicationContainer apps = onoff.Install(c.Get(0)); // from Es1
	apps.Start(Seconds(1.0));
	apps.Stop(END_T);

	// Create a packet sink to receive these packets
	PacketSinkHelper sink("ns3::UdpSocketFactory",
			Address(InetSocketAddress(Ipv4Address::GetAny(), port)));
	apps = sink.Install(c.Get(9));
	apps.Start(Seconds(1.0));
	apps.Stop(END_T);

	AsciiTraceHelper ascii;
	p2p.EnableAsciiAll(ascii.CreateFileStream("lisp-routing.tr"));
	p2p.EnablePcapAll("lisp-routing-final");

	// Flow Monitor
	FlowMonitorHelper flowmonHelper;
	if (enableFlowMonitor) {
		flowmonHelper.InstallAll();
	}

	// Create the animation object and configure for specified output
	AnimationInterface anim(animFile);
	// node positions
	anim.SetConstantPosition(c.Get(0), 0, 10);
	anim.SetConstantPosition(c.Get(1), 2, 7);
	anim.SetConstantPosition(c.Get(2), 2, 5);
	anim.SetConstantPosition(c.Get(3), 0, 3);
	anim.SetConstantPosition(c.Get(4), 5, 8);
	anim.SetConstantPosition(c.Get(4), 5, 9);
	anim.SetConstantPosition(c.Get(5), 4, 7);
	anim.SetConstantPosition(c.Get(6), 4, 5);
	anim.SetConstantPosition(c.Get(7), 7, 7);
	anim.SetConstantPosition(c.Get(8), 8, 5);
	anim.SetConstantPosition(c.Get(9), 10, 5);
	anim.SetConstantPosition(c.Get(10), 9, 7);
	anim.SetConstantPosition(c.Get(11), 4, 2);
	anim.SetConstantPosition(c.Get(12), 7, 2);

	anim.EnablePacketMetadata(true); // Optional
	anim.EnableIpv4L3ProtocolCounters(Seconds(0), Seconds(5)); // Optional

	NS_LOG_INFO("Run Simulation.");
	Simulator::Stop(END_T);
	Simulator::Run();
	std::cout << "Animation Trace file created:" << animFile.c_str()
			<< std::endl;
	NS_LOG_INFO("Done.");

	if (enableFlowMonitor) {
		flowmonHelper.SerializeToXmlFile("lisp-routing.flowmon", false, false);
	}

	Simulator::Destroy();
	return 0;
}

