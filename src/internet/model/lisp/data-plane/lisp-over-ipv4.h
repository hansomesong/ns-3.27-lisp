/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 University of Liege
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
 * Author: Lionel Agbodjan <lionel.agbodjan@gmail.com>
 */
#ifndef LISP_OVER_IPV4_H_
#define LISP_OVER_IPV4_H_

#include "ns3/ptr.h"
#include "ns3/node.h"
#include "ns3/packet.h"
#include "lisp-over-ip.h"

namespace ns3
{

class Ipv4Header;
class Ipv4Route;
class MapTables;
class MapEntry;

/**
 * \class LispOverIpv4
 * \brief Abstract class for LISP data plane over the IPv4 protocol.
 */
class LispOverIpv4 : public LispOverIp {
public:

  /**
   * \brief Get the type ID.
   *
   * \return The object TypeId
   */
  static TypeId GetTypeId (void);

  /**
   * \brief Constructor
   */
  LispOverIpv4 ();

  /**
   * \brief Destructor
   */
  virtual
  ~LispOverIpv4 ();

  /**
   * \enum MapStatus
   *
   * \brief Tells if a mapping exists or not (or if it does not matter).
   */
  enum MapStatus {
    No_Mapping = 0,//!< No_Mapping
    Mapping_Exist, //!< Mapping_Exist
    No_Need_Encap, //!< No_Need_Encap
  };

  /**
   * \brief Process outgoing LISP packets.
   *
   * Process a packet that is leaving the network. The packet will
   * be encapsulated and re-injected to be sent by the IP protocol
   * to the right ETR.
   *
   * \param packet The packet received from the source eid.
   * \param innerHeader The original IP header.
   * \param localMapping The mapping in the LISP database.
   * \param remoteMapping The mapping in the LISP cache
   * \param lispRoute The route to the ETR (needed by the IP protocol).
   */
  virtual void LispOutput (Ptr<Packet> packet, Ipv4Header const &innerHeader,
                           Ptr<const MapEntry> localMapping,
                           Ptr<const MapEntry> remoteMapping,
                           Ptr<Ipv4Route> lispRoute) = 0;

  /**
   * \brief Process incoming LISP packets
   *
   * Process a packet that is entering the network. The packet will be
   * decapsulated and re-injected to be sent by the IP protocol to the
   * right end host.
   *
   * \param packet
   * \param outerHeader
   */
  virtual void LispInput (Ptr<Packet> packet, Ipv4Header const &outerHeader) = 0;

  // NB we give references of pointer because we want pointers to be modified
  /**
   *
   * \param innerHeader
   * \param srcMapEntry
   * \param destMapEntry
   * \param mask
   * \return
   */
  virtual MapStatus IsMapForEncapsulation (Ipv4Header const &innerHeader, Ptr<MapEntry> &srcMapEntry, Ptr<MapEntry> &destMapEntry, Ipv4Mask mask) = 0;


  /**
   *
   * @param ipHeader
   * @param mask
   * @return
   */
  virtual bool NeedEncapsulation (Ipv4Header const &ipHeader, Ipv4Mask mask) = 0;


  /**
   *
   * @param packet
   * @param ipHeader
   * @return
   */
  virtual bool NeedDecapsulation (Ptr<const Packet> packet, Ipv4Header const &ipHeader) = 0;

  /**
   *
   * @param packet
   * @param udpLength
   * @param udpSrcPort
   * @return
   */
  virtual Ptr<Packet> LispEncapsulate (Ptr<Packet> packet, uint16_t udpLength, uint16_t udpSrcPort) = 0;

  /**
   *
   * @param currentDevice
   * @param protocol
   * @param packetType
   */
  void RecordReceiveParams (Ptr<NetDevice> currentDevice, uint16_t protocol, NetDevice::PacketType packetType);

protected:
  /*
   * Reception parameters
   */
  Ptr<NetDevice> m_currentDevice;
  uint16_t m_ipProtocol;
  NetDevice::PacketType m_currentPacketType;
};

} /* namespace ns3 */

#endif /* LISP_OVER_IPV4_H_ */
