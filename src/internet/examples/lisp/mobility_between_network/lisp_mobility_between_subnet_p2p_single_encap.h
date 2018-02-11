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
#include "ns3/flow-monitor-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/map-resolver-helper.h"
#include "ns3/map-server-helper.h"
#include "ns3/lisp-etr-itr-app-helper.h"
#include "ns3/wifi-module.h"
#include "ns3/wifi-mac.h"
#include "ns3/lisp-header.h"
#include "ns3/config-store-module.h"
#include "ns3/dhcp-helper.h"
#include "ns3/callback.h"



namespace ns3
{
	class Node;
	class Ipv4Address;
	class VirtualNetDevice;
	class MobilityModel;
	class Simulation3: public Object
	{
	public:

		Simulation3(Address lispMnEidAddr);

//		void ConfigureLispMN(Ptr<Node> node, NetDeviceContainer mnDevs);

		static void
		SetPosition (Ptr<Node> node, Vector position);

		static Vector
		GetPosition (Ptr<Node> node);

		static void
		AdvancePosition (Ptr<Node> node);

		void
		ChangeDefautGateway (Ptr<Node> node, Ipv4Address gateway,
													uint32_t interface);

		void
		PrintLocations (NodeContainer nodes, std::string header);

		void
		InstallMapResolverApplication (Ptr<Node> node, Ipv4Address msAddress,
																		Time start, Time end);

		void
		InstallEchoApplication (Ptr<Node> echoServerNode, Ptr<Node> echoClientNode,
														Ipv4Address echoServerIpAddr, uint16_t port,
														Time start, Time end);

		void
		InstallOnOffApplication (Ptr<Node> dstNode, Ptr<Node> srcNode,
															Ipv4Address dstIpAddr, uint16_t port, Time start,
															Time end);

		void
		InstallMapServerApplication (Ptr<Node> node, Time start, Time end);

		void
		InstallDhcpServerApplication (Ptr<Node> node, Ipv4Address prefix,
																	Ipv4Mask netmask, Ipv4Address dhcpServerAddr,
																	Ipv4Address min_addr, Ipv4Address max_addr,
																	Ipv4Address gateway, Time start, Time end);

		void
		InstallDhcpClientApplication (Ptr<Node> node, uint32_t device, Time start,
																	Time end);

		void
		InstallLispRouter (NodeContainer nodes);

		void
		InstallXtrApplication (NodeContainer nodes, Ipv4Address mrAddress,
														Ipv4Address msAddress, Time start, Time end);

		void
		PopulateStaticRoutingTable2 (NodeContainer c, uint32_t index);

		void
		CreateAnimFile (NodeContainer c, std::string animFile);


	private:
		Address lispMnEidAddr;
		std::string mapServerInitDbFile;
		std::string rlocsFile;
		std::string wifiPcapFilePrefix;

	};

	class Fuck
	{

	public:
		Fuck (Ptr<VirtualNetDevice> n0Tap, Ptr<NetDevice> netDve);

		void ChangeCB(Ptr<NetDevice> netDve);

		bool
		TapVirtualSend (Ptr<Packet> packet, const Address& source,
										const Address& dest, uint16_t protocolNumber);

		Ptr<VirtualNetDevice> m_n0Tap;
		Ptr<NetDevice> m_netDve;
	};

}

