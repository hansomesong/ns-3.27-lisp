/*
 * TunNetDevice.h
 *
 *  Created on: Jan 30, 2018
 *      Author: qipeng.song@imt-atlantique.fr
 *      		yue.li@telecom-paristech.fr
 */

#ifndef SRC_INTERNET_MODEL_LISP_DATA_PLANE_TUN_NET_DEVICE_H_
#define SRC_INTERNET_MODEL_LISP_DATA_PLANE_TUN_NET_DEVICE_H_

#include "ns3/net-device.h"
#include "ns3/packet.h"
#include "ns3/traced-callback.h"
#include "ns3/callback.h"

namespace ns3
{

	class TunNetDevice : public NetDevice
	{

	public:
		typedef Callback<bool, Ptr<Packet>, const Address&, uint16_t> SendCallback;

	  /**
	   * \brief get typeid
	   * \return typeid
	   */
	  static TypeId GetTypeId (void);

		TunNetDevice ();

		TunNetDevice (Ptr<NetDevice> realDev);

		virtual
		~TunNetDevice ();

	  /**
	   * \brief Set the user callback to be called when a L2 packet is to be transmitted
	   * \param transmitCb the new transmit callback
	   */
	  void SetSendCallback (SendCallback transmitCb);

	  /**
	   * \brief Configure whether the virtual device needs ARP
	   *
	   * \param needsArp the the 'needs arp' value that will be returned
	   * by the NeedsArp() method.  The method IsBroadcast() will also
	   * return this value.
	   */
	  void SetNeedsArp (bool needsArp);

	  /**
	   * \brief Configure whether the virtual device is point-to-point
	   *
	   * \param isPointToPoint the value that should be returned by the
	   * IsPointToPoint method for this instance.
	   */
	  void SetIsPointToPoint (bool isPointToPoint);

	  /**
	   * \brief Configure whether the virtual device supports SendFrom
	   */
	  void SetSupportsSendFrom (bool supportsSendFrom);

	  /**
	   * \brief Configure the reported MTU for the virtual device.
	   * \param mtu MTU value to set
	   * \return whether the MTU value was within legal bounds
	   */
	  bool SetMtu (const uint16_t mtu);


	  /**
	   * \param packet packet sent from below up to Network Device
	   * \param protocol Protocol type
	   * \param source the address of the sender of this packet.
	   * \param destination the address of the receiver of this packet.
	   * \param packetType type of packet received (broadcast/multicast/unicast/otherhost)
	   * \returns true if the packet was forwarded successfully, false otherwise.
	   *
	   * Forward a "virtually received" packet up
	   * the node's protocol stack.
	   */
	  bool Receive (Ptr<Packet> packet, uint16_t protocol,
	                const Address &source, const Address &destination,
	                PacketType packetType);

	  // inherited from NetDevice base class.
	  virtual void SetIfIndex (const uint32_t index);
	  virtual uint32_t GetIfIndex (void) const;
	  virtual Ptr<Channel> GetChannel (void) const;
	  virtual void SetAddress (Address address);
	  virtual Address GetAddress (void) const;
	  virtual uint16_t GetMtu (void) const;
	  virtual bool IsLinkUp (void) const;
	  virtual void AddLinkChangeCallback (Callback<void> callback);
	  virtual bool IsBroadcast (void) const;
	  virtual Address GetBroadcast (void) const;
	  virtual bool IsMulticast (void) const;
	  virtual Address GetMulticast (Ipv4Address multicastGroup) const;
	  virtual Address GetMulticast (Ipv6Address addr) const;
	  virtual bool IsPointToPoint (void) const;
	  virtual bool Send (Ptr<Packet> packet, const Address& dest, uint16_t protocolNumber);
	  virtual bool SendFrom (Ptr<Packet> packet, const Address& source, const Address& dest, uint16_t protocolNumber);
	  virtual Ptr<Node> GetNode (void) const;
	  virtual void SetNode (Ptr<Node> node);
	  virtual bool NeedsArp (void) const;
	  virtual void SetReceiveCallback (NetDevice::ReceiveCallback cb);
	  virtual void SetPromiscReceiveCallback (NetDevice::PromiscReceiveCallback cb);
	  virtual bool SupportsSendFrom () const;
	  virtual bool IsBridge (void) const;

	protected:
	  /**
	   * \brief Dispose of the object
	   */
	  virtual void DoDispose (void);

	private:
		/**
		 * \brief node which contains this device.
		 */
		Ptr<Node> m_node;
		Address m_myAddress;
		SendCallback m_sendCb;
	  /**
	   * \brief Callback to trace received non-promiscuous packets which will be forwarded up the local protocol stack.
	   */
		TracedCallback<Ptr<const Packet> > m_macRxTrace;
		TracedCallback<Ptr<const Packet> > m_macTxTrace;
		TracedCallback<Ptr<const Packet> > m_macPromiscRxTrace;
		TracedCallback<Ptr<const Packet> > m_snifferTrace;
		TracedCallback<Ptr<const Packet> > m_promiscSnifferTrace;
		ReceiveCallback m_rxCallback;
		PromiscReceiveCallback m_promiscRxCallback;
		std::string m_name;
		uint32_t m_index;
		uint16_t m_mtu;

		/**
	   * \brief flag whether ARP required.
	  */
		bool m_needsArp;
	  /**
	   * \brief flag whether it supports send from.
	  */
		bool m_supportsSendFrom;
		bool m_isPointToPoint;

		Ptr<NetDevice> m_RealDev;
	};

} /* namespace ns3 */

#endif /* SRC_INTERNET_MODEL_LISP_DATA_PLANE_TUN_NET_DEVICE_H_ */
