/*
 * TunNetDevice.cpp
 *
 *  Created on: Jan 30, 2018
 *      Author: qsong
 */

#include "tun-net-device.h"
#include "ns3/log.h"
#include "ns3/node.h"
#include "ns3/uinteger.h"
#include "ns3/mac48-address.h"
#include "ns3/channel.h"
#include "ns3/ipv4-static-routing-helper.h"

#include "ns3/ipv4.h"

namespace ns3
{

	NS_LOG_COMPONENT_DEFINE ("TunNetDevice");

	NS_OBJECT_ENSURE_REGISTERED(TunNetDevice);

	TypeId
	TunNetDevice::GetTypeId (void)
	{
		static TypeId tid = TypeId ("ns3::TunNetDevice")
				.SetParent<NetDevice> ()
				.SetGroupName ("NetDevice")
				.AddConstructor<TunNetDevice> ()
				.AddAttribute (
						"Mtu",
						"The MAC-level Maximum Transmission Unit",
						UintegerValue (1500),
						MakeUintegerAccessor (&TunNetDevice::SetMtu,
																	&TunNetDevice::GetMtu),
						MakeUintegerChecker<uint16_t> ())
				.AddTraceSource (
						"MacTx", "Trace source indicating a packet has arrived "
						"for transmission by this device",
						MakeTraceSourceAccessor (&TunNetDevice::m_macTxTrace),
						"ns3::Packet::TracedCallback")
				.AddTraceSource (
						"MacPromiscRx", "A packet has been received by this device, "
						"has been passed up from the physical layer "
						"and is being forwarded up the local protocol stack.  "
						"This is a promiscuous trace,",
						MakeTraceSourceAccessor (&TunNetDevice::m_macPromiscRxTrace),
						"ns3::Packet::TracedCallback")
				.AddTraceSource (
						"MacRx", "A packet has been received by this device, "
						"has been passed up from the physical layer "
						"and is being forwarded up the local protocol stack.  "
						"This is a non-promiscuous trace,",
						MakeTraceSourceAccessor (&TunNetDevice::m_macRxTrace),
						"ns3::Packet::TracedCallback")
				//
				// Trace sources designed to simulate a packet sniffer facility (tcpdump).
				//
				.AddTraceSource (
						"Sniffer", "Trace source simulating a non-promiscuous "
						"packet sniffer attached to the device",
						MakeTraceSourceAccessor (&TunNetDevice::m_snifferTrace),
						"ns3::Packet::TracedCallback")
				.AddTraceSource (
						"PromiscSniffer", "Trace source simulating a promiscuous "
						"packet sniffer attached to the device",
						MakeTraceSourceAccessor (&TunNetDevice::m_promiscSnifferTrace),
						"ns3::Packet::TracedCallback");
		return tid;
	}

	TunNetDevice::TunNetDevice ()
	{
		// TODO Auto-generated constructor stub
		// The MAC addresss is randomly chooosed. No matter.
		SetAddress (Mac48Address::Allocate());
	  m_needsArp = false;
	  m_supportsSendFrom = true;
	  m_isPointToPoint = true;
	}

	TunNetDevice::TunNetDevice (Ptr<NetDevice> realDev)
	{
		// TODO Auto-generated constructor stub
		// The MAC addresss is randomly chooosed. No matter.
		SetAddress (Mac48Address::Allocate());
	  m_needsArp = false;
	  m_supportsSendFrom = true;
	  m_isPointToPoint = true;
	  m_RealDev = realDev;
	}

	TunNetDevice::~TunNetDevice ()
	{
		// TODO Auto-generated destructor stub
	}
	void
	TunNetDevice::SetNeedsArp (bool needsArp)
	{
	  NS_LOG_FUNCTION ( this << needsArp);
	  m_needsArp = needsArp;
	}

	void
	TunNetDevice::SetSupportsSendFrom (bool supportsSendFrom)
	{
	  NS_LOG_FUNCTION ( this << supportsSendFrom);
	  m_supportsSendFrom = supportsSendFrom;
	}

	void
	TunNetDevice::SetIsPointToPoint (bool isPointToPoint)
	{
	  NS_LOG_FUNCTION ( this << isPointToPoint);
	  m_isPointToPoint = isPointToPoint;
	}

	bool
	TunNetDevice::SetMtu (const uint16_t mtu)
	{
	  NS_LOG_FUNCTION ( this << mtu );
	  m_mtu = mtu;
	  return true;
	}

	uint16_t
	TunNetDevice::GetMtu (void) const
	{
	  return m_mtu;
	}

	Address
	TunNetDevice::GetAddress (void) const
	{
	  return m_myAddress;
	}

	void
	TunNetDevice::SetAddress (Address addr)
	{
	  m_myAddress = addr;
	}


	Ptr<Node>
	TunNetDevice::GetNode (void) const
	{
	  return m_node;
	}

	void
	TunNetDevice::SetNode (Ptr<Node> node)
	{
	  m_node = node;
	}


	bool
	TunNetDevice::NeedsArp (void) const
	{
	  return m_needsArp;
	}

	void
	TunNetDevice::SetReceiveCallback (NetDevice::ReceiveCallback cb)
	{
	  NS_LOG_FUNCTION (this);
	  m_rxCallback = cb;
	}

	void
	TunNetDevice::SetPromiscReceiveCallback (NetDevice::PromiscReceiveCallback cb)
	{
	  NS_LOG_FUNCTION (this);
	  m_promiscRxCallback = cb;
	}

	bool
	TunNetDevice::SupportsSendFrom () const
	{
	  NS_LOG_FUNCTION (this);
	  return m_supportsSendFrom;
	}

	bool
	TunNetDevice::IsLinkUp (void) const
	{
	  return true;
	}

	void
	TunNetDevice::AddLinkChangeCallback (Callback<void> callback)
	{
	}

	bool
	TunNetDevice::IsBroadcast (void) const
	{
	  return true;
	}

	Address
	TunNetDevice::GetBroadcast (void) const
	{
	  return Mac48Address ("ff:ff:ff:ff:ff:ff");
	}

	bool
	TunNetDevice::IsMulticast (void) const
	{
	  return false;
	}

	bool TunNetDevice::IsBridge (void) const
	{
	  NS_LOG_FUNCTION (this);
	  return false;
	}

	bool
	TunNetDevice::IsPointToPoint (void) const
	{
	  return m_isPointToPoint;
	}
	Address TunNetDevice::GetMulticast (Ipv4Address multicastGroup) const
	{
	  return Mac48Address ("ff:ff:ff:ff:ff:ff");
	}

	Address TunNetDevice::GetMulticast (Ipv6Address addr) const
	{
	  return Mac48Address ("ff:ff:ff:ff:ff:ff");
	}

	void
	TunNetDevice::SetIfIndex (const uint32_t index)
	{
	  NS_LOG_FUNCTION ( this << index);
	  m_index = index;
	}

	uint32_t
	TunNetDevice::GetIfIndex (void) const
	{
	  NS_LOG_FUNCTION (this);
	  return m_index;
	}

	Ptr<Channel>
	TunNetDevice::GetChannel (void) const
	{
	  NS_LOG_FUNCTION (this);
	  return Ptr<Channel> ();
	}
	bool
	TunNetDevice::Receive (Ptr<Packet> packet, uint16_t protocol,
	                           const Address &source, const Address &destination,
	                           PacketType packetType)
	{
		NS_LOG_FUNCTION ( this << packet << protocol << source << destination << packetType << "Receive ECHO!!!!!!!!!!!!!!!!!!");
	  //
	  // For all kinds of packetType we receive, we hit the promiscuous sniffer
	  // hook and pass a copy up to the promiscuous callback.  Pass a copy to
	  // make sure that nobody messes with our packet.
	  //
	  m_promiscSnifferTrace (packet);
	  if (!m_promiscRxCallback.IsNull ())
	    {
	      m_macPromiscRxTrace (packet);
	      m_promiscRxCallback (this, packet, protocol, source, destination, packetType);
	    }

	  //
	  // If this packet is not destined for some other host, it must be for us
	  // as either a broadcast, multicast or unicast.  We need to hit the mac
	  // packet received trace hook and forward the packet up the stack.
	  //
	  if (packetType != PACKET_OTHERHOST)
	    {
	      m_snifferTrace (packet);
	      m_macRxTrace (packet);
	      return m_rxCallback (this, packet, protocol, source);
	    }
	  return true;
	}

	void
	TunNetDevice::SetSendCallback (SendCallback sendCb)
	{
	  m_sendCb = sendCb;
	}
	bool
	TunNetDevice::Send (Ptr<Packet> packet, const Address& dest, uint16_t protocolNumber)
	{
		/**
		 * Upate, 02-03-2018, Qipeng
		 * The send method of this TUN device will call the send method of IP protocol (Ipv4 or Ipv6)
		 * The signature:
		 * Ipv4L3Protocol::Send(
		 * 				Ptr<Packet> packet,
		 * 				Ipv4Address source,
		 * 				Ipv4Address dest,
		 * 				uint8_t protocol,
		 * 				Ptr<Ipv4Route> route)
		 * We configure static route as follow:
		 * 	  ipv4Stat->AddNetworkRouteTo(Ipv4Address("0.0.0.0"), Ipv4Mask("/1"), 2);
  	 *		ipv4Stat->AddNetworkRouteTo(Ipv4Address("128.0.0.0"), Ipv4Mask("/1"), 2);
  	 * so that the Echo application always choose TUN device to send packet. The latter then
  	 * uses default static (via gateway configured by DHCP client) and invokes the send method
  	 * of Ip level. Ip level protocol (adapted to support LISP) is in charges of LISP encapsulation and transmission
		 *
		 * One question: input parameter `dest` is a MAC address or Ip address?
		 * I guess it is a MAC address, but we don't care its value since the implementation of this method will call
		 * Ip send method which will can call send method of WifiNetDevice. WifiNetDevice is in charge of searching
		 * `dst`
		 */
		NS_LOG_FUNCTION ( this << packet << dest << protocolNumber << "TUN needs to send...");
	  m_macTxTrace (packet);
	  Ipv4Header iph;
	  packet->PeekHeader (iph);
	  Socket::SocketErrno err;
	  Ptr<Ipv4> ipv4 = m_node->GetObject<Ipv4>();
	  Ipv4StaticRoutingHelper ipv4RoutingHelper;
	  ipv4->GetRoutingProtocol();
	  Ptr<Ipv4StaticRouting> staticRouting = ipv4RoutingHelper.GetStaticRouting (ipv4);
	  Ptr<Ipv4Route> defaultRoute = staticRouting->RouteOutput(packet, iph, m_RealDev, err);

//	  ipv4->Send(Ptr<Packet> packet,
//               Ipv4Address source,
//               Ipv4Address destination,
//               uint8_t protocol,
//               Ptr<Ipv4Route> route);
//	  if (m_sendCb (packet, GetAddress (), dest, protocolNumber))
		if (m_sendCb (packet, dest, protocolNumber))

	    {
	      return true;
	    }
	  return false;
	}

	bool
	TunNetDevice::SendFrom (Ptr<Packet> packet, const Address& source, const Address& dest, uint16_t protocolNumber)
	{
	  NS_ASSERT (m_supportsSendFrom);
	  m_macTxTrace (packet);
//	  if (m_sendCb (packet, source, dest, protocolNumber))
//	    {
//	      return true;
//	    }
	  return false;
	}

	void TunNetDevice::DoDispose ()
	{
	  m_node = 0;
	  NetDevice::DoDispose ();
	}

} /* namespace ns3 */
