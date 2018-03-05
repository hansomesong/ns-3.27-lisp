/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 UPB
 * Copyright (c) 2017 NITK Surathkal
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Radu Lupu <rlupu@elcom.pub.ro>
 *         Ankit Deepak <adadeepak8@gmail.com>
 *         Deepti Rajagopal <deeptir96@gmail.com>
 *
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <list>

#include "ns3/ipv4.h"
#include "ns3/log.h"
#include "ns3/double.h"
#include "ns3/ipv4-address.h"
#include "ns3/nstime.h"
#include "ns3/inet-socket-address.h"
#include "ns3/socket.h"
#include "ns3/simulator.h"
#include "ns3/socket-factory.h"
#include "ns3/packet.h"
#include "ns3/uinteger.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/random-variable-stream.h"
#include "ns3/ipv4-routing-table-entry.h"
#include "ns3/pointer.h"
#include "ns3/string.h"
#include "ns3/ipv4-raw-socket-impl.h"
#include "ns3/ipv4-raw-socket-factory.h"
#include "ns3/udp-l4-protocol.h"
#include "ns3/udp-header.h"
#include "ns3/lisp-mapping-socket.h" //To support LISP with DHCP!
#include "ns3/mapping-socket-msg.h" //To support LISP with DHCP!
#include "ns3/lisp-over-ipv4.h"
//#include "ns3/locators-impl.h"
#include "ns3/simple-map-tables.h"

#include "dhcp-client.h"
#include "dhcp-server.h"
#include "dhcp-header.h"

namespace ns3
{

	NS_LOG_COMPONENT_DEFINE("DhcpClient");
	NS_OBJECT_ENSURE_REGISTERED(DhcpClient);

	TypeId
	DhcpClient::GetTypeId (void)
	{
		static TypeId tid =
				TypeId ("ns3::DhcpClient").SetParent<Application> ().AddConstructor<
						DhcpClient> ().SetGroupName ("Internet-Apps").AddAttribute (
						"NetDevice", "Index of netdevice of the node for DHCP",
						UintegerValue (0), MakeUintegerAccessor (&DhcpClient::m_device),
						MakeUintegerChecker<uint32_t> ()).AddAttribute (
						"RTRS", "Time for retransmission of Discover message",
						TimeValue (Seconds (100)), MakeTimeAccessor (&DhcpClient::m_rtrs),
						MakeTimeChecker ()).AddAttribute (
						"Collect", "Time for which offer collection starts",
						TimeValue (Seconds (0.9)),
						MakeTimeAccessor (&DhcpClient::m_collect), MakeTimeChecker ()).AddAttribute (
						"ReRequest",
						"Time after which request will be resent to next server",
						TimeValue (Seconds (10)),
						MakeTimeAccessor (&DhcpClient::m_nextoffer), MakeTimeChecker ()).AddAttribute (
						"Transactions", "The possible value of transaction numbers ",
						StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1000000.0]"),
						MakePointerAccessor (&DhcpClient::m_ran),
						MakePointerChecker<RandomVariableStream> ()).AddTraceSource (
						"NewLease", "Get a NewLease",
						MakeTraceSourceAccessor (&DhcpClient::m_newLease),
						"ns3::Ipv4Address::TracedCallback").AddTraceSource (
						"Expiry", "A lease expires",
						MakeTraceSourceAccessor (&DhcpClient::m_expiry),
						"ns3::Ipv4Address::TracedCallback");
		return tid;
	}

	DhcpClient::DhcpClient () :
			m_server (Ipv4Address::GetAny ())
	{
		NS_LOG_FUNCTION_NOARGS ();
		// socket can be initialized as 0. How about callback function?
		m_socket = 0;
		m_refreshEvent = EventId ();
		m_requestEvent = EventId ();
		m_discoverEvent = EventId ();
		m_rebindEvent = EventId ();
		m_nextOfferEvent = EventId ();
		m_timeout = EventId ();
		m_trigLisp = false;
		m_remoteAddress = Ipv4Address ("255.255.255.255");
		m_myAddress = Ipv4Address ("0.0.0.0");
		m_gateway = Ipv4Address ("0.0.0.0");

	}

	DhcpClient::~DhcpClient ()
	{
		NS_LOG_FUNCTION_NOARGS ();
	}

	Ipv4Address
	DhcpClient::GetDhcpServer (void)
	{
		return m_server;
	}

	void
	DhcpClient::DoDispose (void)
	{
		NS_LOG_FUNCTION_NOARGS ();
		Application::DoDispose ();
	}

	int64_t
	DhcpClient::AssignStreams (int64_t stream)
	{
		NS_LOG_FUNCTION(this << stream);
		m_ran->SetStream (stream);
		return 1;
	}

	void
	DhcpClient::CreateSocket ()
	{
		NS_LOG_FUNCTION(this);
		NS_LOG_DEBUG("DHCP client tries to create Ipv4RawSocket...");
		if (m_socket != 0)
			{
				NS_LOG_WARN("DHCP client has already a Ipv4RawSocket!");
			}
		if (m_socket == 0)
			{
				TypeId tid = TypeId::LookupByName ("ns3::Ipv4RawSocketFactory");
				m_socket = DynamicCast<Ipv4RawSocketImpl> (
						Socket::CreateSocket (GetNode (), tid));
				m_socket->SetProtocol (UdpL4Protocol::PROT_NUMBER);
				m_socket->SetAllowBroadcast (true);
				m_socket->Bind ();
				m_socket->BindToNetDevice (GetNode ()->GetDevice (m_device));
			}
		// Since in LinkStateChange handler, RecvCallback has been removed...
		// Thus, here in anycase, we should add this callback...
		m_socket->SetRecvCallback (MakeCallback (&DhcpClient::NetHandler, this));
		NS_LOG_DEBUG("DHCP client finishes the socket create process");
	}

	void
	DhcpClient::CloseSocket ()
	{
		m_socket->SetRecvCallback (MakeNullCallback<void, Ptr<Socket> > ());
		m_socket->Close ();
	}

	void
	DhcpClient::StartApplication (void)
	{
		NS_LOG_FUNCTION_NOARGS ();
		NS_LOG_DEBUG(
				"At the start of DCHP client application, Wifi Net device's status: "<<GetNode ()->GetDevice (m_device)->IsLinkUp ());
		InitializeAddress ();
		CreateSocket ();
		/**
		 * Update,02-22-2018,Yue
		 * Associate callback with corresponding LISP methods
		 */
	  Ptr<LispOverIp> lisp = GetNode()->GetObject<LispOverIp> ();
	  if(lisp)
	  	{
	  		NS_LOG_DEBUG("LISP compatible DHCP client. Associate callbacks...");
	  	  SetUpdateRlocCallback(MakeCallback(&LispOverIp::InsertKnownRloc, lisp));
	  	  SetAlloIpCallback(MakeCallback(&LispOverIp::DatabaseUpdateHandler, lisp));
	  	}

		// For example, Wifi Adhoc mode has no link state change(I think so...)
		// That's why we have to send call Boot() here to send DHCP discover...
//    Boot ();
		/**
		 * Append a Callback to the chain. I wonder what the chain refers to.
		 * It seems a container which can hold more than one elements.
		 * It's also logical for a net device to trigger more several callbacks if one event
		 * triggers. Thus, I think, here we should not add a Link change callback function
		 * each time DHCP client is started. Because link state change handler will start DHCP
		 * client which add link chagne callback function. Hence, I think, it is better to
		 * add this hanlder just in DHCP's constructor.
		 */
		GetNode ()->GetDevice (m_device)->AddLinkChangeCallback (
				MakeCallback (&DhcpClient::LinkStateHandler, this));
		NS_LOG_DEBUG(
				"Add a Link Change Callback to net device with index: "<<unsigned(m_device));
		Boot ();
	}

	void
	DhcpClient::InitializeAddress ()
	{
		Ptr<Ipv4> ipv4 = GetNode ()->GetObject<Ipv4> ();
		uint32_t ifIndex = ipv4->GetInterfaceForDevice (
				GetNode ()->GetDevice (m_device));
//    NS_LOG_DEBUG("DHCP client is running on interface: "<<unsigned(ifIndex));
		bool found = false;
		for (uint32_t i = 0; i < ipv4->GetNAddresses (ifIndex); i++)
			{
				if (ipv4->GetAddress (ifIndex, i).GetLocal () == m_myAddress)
					{
						found = true;
					}
			}
		if (!found)
			{
				ipv4->AddAddress (
						ifIndex,
						Ipv4InterfaceAddress (Ipv4Address ("0.0.0.0"), Ipv4Mask ("/0")));
			}
	}

	void
	DhcpClient::StopApplication ()
	{
		NS_LOG_FUNCTION_NOARGS ();
		Simulator::Remove (m_discoverEvent);
		Simulator::Remove (m_requestEvent);
		Simulator::Remove (m_rebindEvent);
		Simulator::Remove (m_refreshEvent);
		Simulator::Remove (m_timeout);
		Simulator::Remove (m_nextOfferEvent);
		Ptr<Ipv4> ipv4 = GetNode ()->GetObject<Ipv4> ();

		int32_t ifIndex = ipv4->GetInterfaceForDevice (
				GetNode ()->GetDevice (m_device));
		NS_ASSERT(ifIndex >= 0);
		for (uint32_t i = 0; i < ipv4->GetNAddresses (ifIndex); i++)
			{
				if (ipv4->GetAddress (ifIndex, i).GetLocal () == m_myAddress)
					{
						ipv4->RemoveAddress (ifIndex, i);
						break;
					}
			}
		Ipv4StaticRoutingHelper ipv4RoutingHelper;
		Ptr<Ipv4StaticRouting> staticRouting = ipv4RoutingHelper.GetStaticRouting (
				ipv4);
		uint32_t i;
		for (i = 0; i < staticRouting->GetNRoutes (); i++)
			{
				if (staticRouting->GetRoute (i).GetGateway () == m_gateway
						&& staticRouting->GetRoute (i).GetInterface () == (uint32_t) ifIndex
						&& staticRouting->GetRoute (i).GetDest ()
								== Ipv4Address ("0.0.0.0"))
					{
						staticRouting->RemoveRoute (i);
						break;
					}
			}
		CloseSocket ();
	}

	void
	DhcpClient::LinkStateHandler (void)
	{
		NS_LOG_FUNCTION_NOARGS ();
		bool linkUp = GetNode ()->GetDevice (m_device)->IsLinkUp ();
		if (linkUp)
			{
				/**
				 * ATTENTION: It seems that AdHoc has no link change... I never find this
				 * handler is triggered until now if using Adhoc wifi mode.
				 */
				NS_LOG_INFO("Link up at " << Simulator::Now ().GetSeconds ());
				InitializeAddress ();
				CreateSocket ();
				Boot ();
			}
		else if (not linkUp)
			{
				//TODO: If Link down, under LISP-MN => Assigned RLOC is lost => Need to update lisp
				//data plan database.
				NS_LOG_INFO("Link down at " << Simulator::Now ().GetSeconds ()); //reinitialization
				Simulator::Remove (m_refreshEvent); //stop refresh timer!!!!
				Simulator::Remove (m_rebindEvent);
				Simulator::Remove (m_timeout);
				m_socket->SetRecvCallback (MakeNullCallback<void, Ptr<Socket> > ()); //stop receiving on this socket !!!

				Ptr<Ipv4> ipv4 = GetNode ()->GetObject<Ipv4> ();
				int32_t ifIndex = ipv4->GetInterfaceForDevice (
						GetNode ()->GetDevice (m_device));
				NS_ASSERT_MSG(ifIndex >= 0,
											"interface index should be >=0, but:"<<ifIndex);
				for (uint32_t i = 0; i < ipv4->GetNAddresses (ifIndex); i++)
					{
						if (ipv4->GetAddress (ifIndex, i).GetLocal () == m_myAddress)
							{
								ipv4->RemoveAddress (ifIndex, i);
								// Still set ipv4 address to 0.0.0.0
								// Since when link down, it is still possible to transmit IP packet,
								// which causes problems if no IP address.
								ipv4->AddAddress (
										ifIndex,
										Ipv4InterfaceAddress (Ipv4Address ("0.0.0.0"),
																					Ipv4Mask ("/0")));
								break;
							}
					}

				Ipv4StaticRoutingHelper ipv4RoutingHelper;
				Ptr<Ipv4StaticRouting> staticRouting =
						ipv4RoutingHelper.GetStaticRouting (ipv4);
				Ipv4RoutingTableEntry routeEntry;
				/**
				 * Two iterations are required. First one is to find the route index to be deleted.
				 * Second one is to delete those routes found in first iteration
				 */
				//TODO: Don't know should remove all zeros...
//	DhcpClient::RemoveAllZerosStaticRoute ();
				for (uint32_t i = 0; i < staticRouting->GetNRoutes (); i++)
					{
						routeEntry = staticRouting->GetRoute (i);
						NS_LOG_DEBUG(
								"Processing Static Route to "<<routeEntry.GetDestNetwork());
						if (routeEntry.GetGateway () == m_gateway)
							{
								NS_LOG_DEBUG(
										"Static Route "<<staticRouting->GetRoute (i)<<" will be deleted!");
								staticRouting->RemoveRoute (i);
								i--;
							}
					}
				//TODO: should be improved. hard code now...
				//Now idea is to delete all routes containing m_gateway
				NS_LOG_INFO("Finish to processing link down related manipulation...");
			}
	}

	void
	DhcpClient::NetHandler (Ptr<Socket> socket)
	{
		NS_LOG_FUNCTION(this << socket);
		Address from;
		Ptr<Packet> packet = m_socket->RecvFrom (2048, 0, from);
		NS_LOG_DEBUG("Received packet:"<<packet);

		Ipv4Header ipHeader;
		UdpHeader udpHeader;
		packet->RemoveHeader (ipHeader);
		packet->RemoveHeader (udpHeader);

		if (udpHeader.GetDestinationPort () != PORT)
			{
				return;
			}

		DhcpHeader header;
		if (packet->RemoveHeader (header) == 0)
			{
				return;
			}
		if (header.GetChaddr () != GetNode ()->GetDevice (m_device)->GetAddress ())
			{
				return;
			}
		if (m_state == WAIT_OFFER && header.GetType () == DhcpHeader::DHCPOFFER)
			{
				OfferHandler (header);
			}
		if (m_state == WAIT_ACK && header.GetType () == DhcpHeader::DHCPACK)
			{
				Simulator::Remove (m_nextOfferEvent);
				AcceptAck (header, from);
			}
		if (m_state == WAIT_ACK && header.GetType () == DhcpHeader::DHCPNACK)
			{
				Simulator::Remove (m_nextOfferEvent);
				Boot ();
			}
	}

	void
	DhcpClient::Boot (void)
	{
		NS_LOG_FUNCTION_NOARGS ();
		DhcpHeader dhcpHeader;
		UdpHeader udpHeader;
		Ptr<Packet> packet;
		packet = Create<Packet> ();

		dhcpHeader.ResetOpt ();
		m_tran = (uint32_t) (m_ran->GetValue ());
		dhcpHeader.SetTran (m_tran);
		dhcpHeader.SetType (DhcpHeader::DHCPDISCOVER);
		dhcpHeader.SetTime ();
		dhcpHeader.SetChaddr (GetNode ()->GetDevice (m_device)->GetAddress ());
		packet->AddHeader (dhcpHeader);

		udpHeader.SetDestinationPort (DhcpServer::PORT);
		udpHeader.SetSourcePort (PORT);
		if (Node::ChecksumEnabled ())
			{
				udpHeader.EnableChecksums ();
				udpHeader.InitializeChecksum (m_myAddress,
																			Ipv4Address ("255.255.255.255"),
																			UdpL4Protocol::PROT_NUMBER);
			}
		packet->AddHeader (udpHeader);

		if ((m_socket->SendTo (packet, 0,
														InetSocketAddress (Ipv4Address ("255.255.255.255"))))
				>= 0)
			{
				NS_LOG_INFO(
						"At time " << Simulator::Now ().GetSeconds () << "s DHCP DISCOVER sent. Content: "<<*packet);
			}
		else
			{
				NS_LOG_INFO("Error while sending DHCP DISCOVER to " << m_remoteAddress);
			}
		//TODO: Is it OK to put m_state assignment here? what if m_socket cannot send a DHCP discovery...
		m_state = WAIT_OFFER;
		m_offered = false;
		m_discoverEvent = Simulator::Schedule (m_rtrs, &DhcpClient::Boot, this);
	}

	void
	DhcpClient::OfferHandler (DhcpHeader header)
	{
		NS_LOG_FUNCTION(this << header);
		m_offerList.push_back (header);
		if (m_offered == false)
			{
				Simulator::Remove (m_discoverEvent);
				m_offered = true;
				NS_LOG_INFO(
						"Reception of DHCP offer from DHCP server. Remove m_discoverEvent event and schedule a new event!");
				Simulator::Schedule (m_collect, &DhcpClient::Select, this);
			}
	}

	void
	DhcpClient::Select (void)
	{
		NS_LOG_FUNCTION_NOARGS ();
		if (m_offerList.empty ())
			{
				return;
			}
		DhcpHeader header = m_offerList.front ();
		m_offerList.pop_front ();
		m_lease = Time (Seconds (header.GetLease ()));
		m_renew = Time (Seconds (header.GetRenew ()));
		m_rebind = Time (Seconds (header.GetRebind ()));
		m_offeredAddress = header.GetYiaddr ();
		m_myMask = Ipv4Mask (header.GetMask ());
		m_server = header.GetDhcps ();
		m_gateway = header.GetRouter ();
		m_offerList.clear ();
		m_offered = false;
		Request ();
		NS_LOG_INFO(
				"At time " << Simulator::Now ().GetSeconds () << "s DHCP client sent DHCP REQUEST to DHCP Server");
	}

	void
	DhcpClient::Request (void)
	{
		NS_LOG_FUNCTION_NOARGS ();
		DhcpHeader dhcpHeader;
		UdpHeader udpHeader;
		Ptr<Packet> packet;
		if (m_state != REFRESH_LEASE)
			{
				packet = Create<Packet> ();

				dhcpHeader.ResetOpt ();
				dhcpHeader.SetType (DhcpHeader::DHCPREQ);
				dhcpHeader.SetTime ();
				dhcpHeader.SetTran (m_tran);
				dhcpHeader.SetReq (m_offeredAddress);
				dhcpHeader.SetChaddr (GetNode ()->GetDevice (m_device)->GetAddress ());
				packet->AddHeader (dhcpHeader);

				udpHeader.SetDestinationPort (DhcpServer::PORT);
				udpHeader.SetSourcePort (PORT);
				if (Node::ChecksumEnabled ())
					{
						udpHeader.EnableChecksums ();
						udpHeader.InitializeChecksum (m_myAddress,
																					Ipv4Address ("255.255.255.255"),
																					UdpL4Protocol::PROT_NUMBER);
					}
				packet->AddHeader (udpHeader);

				m_socket->SendTo (packet, 0,
													InetSocketAddress (Ipv4Address ("255.255.255.255")));
				m_state = WAIT_ACK;
				m_nextOfferEvent = Simulator::Schedule (m_nextoffer,
																								&DhcpClient::Select, this);
			}
		else
			{
				//CreateSocket ();
				uint32_t addr = m_myAddress.Get ();
				packet = Create<Packet> ((uint8_t*) &addr, sizeof(addr));

				dhcpHeader.ResetOpt ();
				m_tran = (uint32_t) (m_ran->GetValue ());
				dhcpHeader.SetTran (m_tran);
				dhcpHeader.SetTime ();
				dhcpHeader.SetType (DhcpHeader::DHCPREQ);
				dhcpHeader.SetReq (m_myAddress);
				m_offeredAddress = m_myAddress;
				dhcpHeader.SetChaddr (GetNode ()->GetDevice (m_device)->GetAddress ());
				packet->AddHeader (dhcpHeader);

				udpHeader.SetDestinationPort (DhcpServer::PORT);
				udpHeader.SetSourcePort (PORT);
				if (Node::ChecksumEnabled ())
					{
						udpHeader.EnableChecksums ();
						udpHeader.InitializeChecksum (m_myAddress, m_remoteAddress,
																					UdpL4Protocol::PROT_NUMBER);
					}
				packet->AddHeader (udpHeader);

				if ((m_socket->SendTo (packet, 0, InetSocketAddress (m_remoteAddress)))
						>= 0)
					{
						NS_LOG_INFO("DHCP REQUEST sent");
					}
				else
					{
						NS_LOG_INFO("Error while sending DHCP REQ to " << m_remoteAddress);
					}
				m_state = WAIT_ACK;
			}
	}

	void
	DhcpClient::AcceptAck (DhcpHeader header, Address from)
	{
		/**
		 * Note that attributes such as m_offeredAddress, m_gateway,etc. have been
		 * configured in DhcpClient::Select.
		 */
		Simulator::Remove (m_rebindEvent);
		Simulator::Remove (m_refreshEvent);
		Simulator::Remove (m_timeout);
		NS_LOG_INFO(
				"At time " << Simulator::Now ().GetSeconds () <<"s DHCP ACK received");
		Ptr<Ipv4> ipv4 = GetNode ()->GetObject<Ipv4> ();
		int32_t ifIndex = ipv4->GetInterfaceForDevice (
				GetNode ()->GetDevice (m_device));
		/**
		 * Before assigning the newly received m_offeredAddress, remove the previous @IP
		 * However, one Ipv4Interface may have more than one Ipv4Address (e.g. IP Aliasing)
		 * That's why we need to iterate each Ipv4Address on NetDevice where DHCP client manages.
		 * ATTENTION: I don't know why but I actually encounter a bug where one interface
		 * has several IP addresses among them several are 0.0.0.0!
		 */
		for (uint32_t i = 0; i < ipv4->GetNAddresses (ifIndex); i++)
			{
				Ipv4Address currIpAddr = ipv4->GetAddress (ifIndex, i).GetLocal ();
				if (currIpAddr == m_myAddress or currIpAddr == Ipv4Address ("0.0.0.0"))
					{
						ipv4->RemoveAddress (ifIndex, i);
						i--;
						//TODO: I don't know the negative impact if removing the break...
					}
			}
		ipv4->AddAddress (
				ifIndex,
				Ipv4InterfaceAddress ((Ipv4Address) m_offeredAddress, m_myMask));
		ipv4->SetUp (ifIndex);
		NS_LOG_DEBUG(
				""<<(Ipv4Address) m_offeredAddress<<" should be added to interface "<<ifIndex);
		for (uint32_t i = 0; i < ipv4->GetNAddresses (ifIndex); i++)
			{
				NS_LOG_DEBUG(
						""<<ipv4->GetAddress (ifIndex, i).GetLocal ()<<" has be actually added to interface "<<ifIndex<<" with index "<<i);
			}

		if (m_myAddress != m_offeredAddress)
			{
				// A different IP@ to the previously assigned IP@, trigger LISP
				m_trigLisp = true;
				m_newLease (m_offeredAddress);
				if (m_myAddress != Ipv4Address ("0.0.0.0"))
					{
						m_expiry (m_myAddress);
					}
			}
		m_myAddress = m_offeredAddress;
		if (m_gateway != Ipv4Address ("0.0.0.0"))
			{
				Ipv4StaticRoutingHelper ipv4RoutingHelper;
				Ptr<Ipv4StaticRouting> staticRouting =
						ipv4RoutingHelper.GetStaticRouting (ipv4);
				/**
				 * Given that DHCP client may send several DHCP Boot/Request message, it is possible
				 * taht DHCP client receives severals time the DHCP offer. Therefore:
				 * We should check whether the received gateway has been already in static routing table.
				 * If yes, we don't need to insert it into routing table.
				 */
				if (not isGateWayExist (m_gateway))
					{
						DhcpClient::RemoveAllZerosStaticRoute ();
						staticRouting->SetDefaultRoute (m_gateway, ifIndex, 0);
						//TODO: DHCP client should know the MR/MS address
						//TODO: Complete this part in the future...
						staticRouting->AddNetworkRouteTo (Ipv4Address ("10.1.6.0"),
																							Ipv4Mask ("255.255.255.0"),
																							m_gateway, m_device, 0);
						staticRouting->AddNetworkRouteTo (Ipv4Address ("10.1.4.0"),
																							Ipv4Mask ("255.255.255.0"),
																							m_gateway, m_device, 0);
						staticRouting->AddNetworkRouteTo (Ipv4Address ("10.1.5.0"),
																							Ipv4Mask ("255.255.255.0"),
																							m_gateway, m_device, 0);
//
//						staticRouting->AddNetworkRouteTo(Ipv4Address("0.0.0.0"), Ipv4Mask("/1"), 2);
//						staticRouting->AddNetworkRouteTo(Ipv4Address("128.0.0.0"), Ipv4Mask("/1"), 2);

					}
				/**
				 * Finally, two static route to ensure that traffic always uses EID as source address
				 * will be added when creating LISP-MN node.
				 * staticRouting->AddNetworkRouteTo(Ipv4Address("0.0.0.0"), Ipv4Mask("/1"), ifTunIndex);
				 * staticRouting->AddNetworkRouteTo(Ipv4Address("128.0.0.0"), Ipv4Mask("/1"), ifTunIndex);
				 */
			}

		m_remoteAddress = InetSocketAddress::ConvertFrom (from).GetIpv4 ();
		NS_LOG_INFO("Current DHCP Server is " << m_remoteAddress);

		// Do lisp-related manipulation, such as create database, update map entry in database.
		// Just m_trigLisp = true, which means has a different @IP, need to send the newly
		// EID-RLOC mapping to lispOverIp.
		if (DhcpClient::IsLispCompatible () and m_trigLisp)
			{
				NS_LOG_DEBUG(
						"A different assigned IP address! LISP processing should be called...");
				//To test callback, comment this invocation.
				//DhcpClient::LispDataBaseManipulation (DhcpClient::GetEid ());

				AcceptAckLispHandler(DhcpClient::GetEid ());
			}
		m_offerList.clear ();
		m_refreshEvent = Simulator::Schedule (m_renew, &DhcpClient::Request, this);
		m_rebindEvent = Simulator::Schedule (m_rebind, &DhcpClient::Request, this);
		m_timeout = Simulator::Schedule (m_lease, &DhcpClient::RemoveAndStart,
																			this);
		m_state = REFRESH_LEASE;
		// Do not forget to reset the following flag as false
		m_trigLisp = false;
	}

	void
	DhcpClient::RemoveAllZerosStaticRoute ()
	{
		Ptr<Ipv4> ipv4 = GetNode ()->GetObject<Ipv4> ();
		Ipv4StaticRoutingHelper ipv4RoutingHelper;
		Ptr<Ipv4StaticRouting> staticRouting = ipv4RoutingHelper.GetStaticRouting (
				ipv4);
		int32_t ifIndex = ipv4->GetInterfaceForDevice (
				GetNode ()->GetDevice (m_device));
		for (uint32_t i = 0; i < staticRouting->GetNRoutes (); i++)
			{
				if (staticRouting->GetRoute (i).GetGateway () == Ipv4Address ("0.0.0.0")
						&& staticRouting->GetRoute (i).GetInterface () == (uint32_t) ifIndex
						&& staticRouting->GetRoute (i).GetDest ()
								== Ipv4Address ("0.0.0.0"))
					{
						NS_LOG_DEBUG("Removing all 0 static route...");
						staticRouting->RemoveRoute (i);
					}
			}
	}

	bool
	DhcpClient::isGateWayExist (Ipv4Address m_gateway)
	{
		bool result = false;
		Ptr<Ipv4> ipv4 = GetNode ()->GetObject<Ipv4> ();
		Ipv4StaticRoutingHelper ipv4RoutingHelper;
		Ptr<Ipv4StaticRouting> staticRouting = ipv4RoutingHelper.GetStaticRouting (
				ipv4);
		int32_t ifIndex = ipv4->GetInterfaceForDevice (
				GetNode ()->GetDevice (m_device));
		for (uint32_t i = 0; i < staticRouting->GetNRoutes (); i++)
			{
				if (staticRouting->GetRoute (i).GetGateway () == m_gateway
						&& staticRouting->GetRoute (i).GetInterface () == (uint32_t) ifIndex
						&& staticRouting->GetRoute (i).GetDest ()
								== Ipv4Address ("0.0.0.0"))
					{
						result = true;
						break;
					}
			}
		return result;
	}

	bool
	DhcpClient::IsLispCompatible ()
	{
		return GetNode ()->GetObject<LispOverIpv4> () != 0;
	}

	bool
	DhcpClient::IsLispDataBasePresent ()
	{
		bool result = false;
		if (DhcpClient::IsLispCompatible ())
			{
				result =
						((GetNode ()->GetObject<LispOverIpv4> ())->GetMapTablesV4 ()->GetNMapEntriesLispDataBase ()
								!= 0);
			}
		return result;
	}

	uint32_t
	DhcpClient::GetIfTunIndex ()
	{
		NS_LOG_FUNCTION(this);
		Ptr<Ipv4> ipv4 = GetNode ()->GetObject<Ipv4> ();
		uint32_t ifTunIndex = 0;
		uint32_t tunDeviceIndex = 1;
		/**
		 * Ignore the device with index 0 => Loopback device
		 * Iterate Net device index and compare its type id name to find virtual net device index.
		 * Then get @IP address
		 */
		for (tunDeviceIndex = 1; tunDeviceIndex < GetNode ()->GetNDevices ();
				tunDeviceIndex++)
			{
				Ptr<NetDevice> curDev = GetNode ()->GetDevice (tunDeviceIndex);
				// Be careful with compare() method.
				if (curDev->GetInstanceTypeId ().GetName () == "ns3::VirtualNetDevice")
					{
						NS_LOG_DEBUG("TUN device index: "<<unsigned(tunDeviceIndex));
						NS_LOG_DEBUG(
								"Device type: "<<curDev->GetInstanceTypeId ().GetName ());
						ifTunIndex = ipv4->GetInterfaceForDevice (
								GetNode ()->GetDevice (tunDeviceIndex));
						NS_LOG_DEBUG("TUN interface Index: "<< unsigned(ifTunIndex));
						/**
						 * An intutive difference between Ipv4InterfaceAddress and Ipv4Address is:
						 * the former container Ipv4Address along with Ipv4Maks, broacast, etc...
						 * The information we can see from "ifconfig eth0"
						 * TODO: Attention we assume that one LISP-MN just have one @IP!
						 * It is trival to extend to support several @IP on TUN, which is not necessary
						 * in this case!
						 */
						//TODO: How to process the case where more than one TUN device on a node?
						break;
					}
			}
		//If return 0, means no TUN device
		return ifTunIndex;
	}

	Ptr<EndpointId>
	DhcpClient::GetEid ()
	{
		NS_LOG_FUNCTION(this);
		Ptr<EndpointId> eid;
		uint32_t ifTunIndex = DhcpClient::GetIfTunIndex ();
		Ptr<Ipv4> ipv4 = GetNode ()->GetObject<Ipv4> ();
		if (ifTunIndex)
			{
				Ipv4Address eidAddress = ipv4->GetAddress (ifTunIndex, 0).GetLocal ();
//      Ipv4Mask eidMask = ipv4->GetAddress (ifTunIndex, 0).GetMask ();
				// LISP-MN, as a single machine, we use "/32" as EID mask
				eid = Create<EndpointId> (eidAddress, Ipv4Mask ("/32"));
				NS_LOG_DEBUG("The retrieved EID:"<< eid->Print());
			}
		/**
		 * TODO: Maybe we should consider the case where TUN net device is installed.
		 * But, no @IP is assigned!
		 */
		return eid;
	}


	void
	DhcpClient::AcceptAckLispHandler(Ptr<EndpointId> eid)
	{
		//m_myAddress has already been confirmed.
		//This is a local RLOC from LISP point of view
		NS_LOG_FUNCTION(this<<eid<<&m_updateRlocCb);
		m_updateRlocCb(static_cast<Address>(m_myAddress));
		//TODO: Add one method to create MapEntry for EID-Local RLOC
		Ptr<MapEntry> mapEntry = Create<MapEntryImpl> ();
		Ptr<Locators> locators = Create<LocatorsImpl> ();
		Ptr<RlocMetrics> rlocMetrics = Create<RlocMetrics> (100, 100, true);
		rlocMetrics->SetMtu (1500);
		rlocMetrics->SetIsLocalIf (true);
		Ptr<Locator> locator = Create<Locator> (m_myAddress);
		locator->SetRlocMetrics (rlocMetrics);
		locators->InsertLocator (locator);
		mapEntry->SetEidPrefix (eid);
		mapEntry->setIsNegative (0);
		mapEntry->SetLocators (locators);
		m_allocationIpCb(mapEntry);
	}

	void
	DhcpClient::RemoveAndStart ()
	{
		NS_LOG_FUNCTION_NOARGS ();
		Simulator::Remove (m_nextOfferEvent);
		Simulator::Remove (m_refreshEvent);
		Simulator::Remove (m_rebindEvent);
		Simulator::Remove (m_timeout);

		Ptr<Ipv4> ipv4MN = GetNode ()->GetObject<Ipv4> ();
		int32_t ifIndex = ipv4MN->GetInterfaceForDevice (
				GetNode ()->GetDevice (m_device));
		NS_ASSERT(ifIndex >= 0);
		for (uint32_t i = 0; i < ipv4MN->GetNAddresses (ifIndex); i++)
			{
				if (ipv4MN->GetAddress (ifIndex, i).GetLocal () == m_myAddress)
					{
						ipv4MN->RemoveAddress (ifIndex, i);
						break;
					}
			}
		m_expiry (m_myAddress);
		Ipv4StaticRoutingHelper ipv4RoutingHelper;
		Ptr<Ipv4StaticRouting> staticRouting = ipv4RoutingHelper.GetStaticRouting (
				ipv4MN);
		uint32_t i;
		for (i = 0; i < staticRouting->GetNRoutes (); i++)
			{
				if (staticRouting->GetRoute (i).GetGateway () == m_gateway
						&& staticRouting->GetRoute (i).GetInterface () == (uint32_t) ifIndex
						&& staticRouting->GetRoute (i).GetDest ()
								== Ipv4Address ("0.0.0.0"))
					{
						staticRouting->RemoveRoute (i);
						break;
					}
			}
		StartApplication ();
	}

	void
	DhcpClient::SetAlloIpCallback(AlloIpCallback cb)
	{
		m_allocationIpCb = cb;
	}

	void
	DhcpClient::SetUpdateRlocCallback(UpdateRlocCallback cb)
	{
		NS_LOG_FUNCTION(this<<&cb);
		m_updateRlocCb = cb;
	}

} // Namespace ns3
