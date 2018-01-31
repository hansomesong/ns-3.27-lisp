/*
 * lisp-protocol.h
 *
 *  Created on: 28 janv. 2016
 *      Author: lionel
 */

#ifndef LISP_OVER_IPV4_IMPL_H_
#define LISP_OVER_IPV4_IMPL_H_

#include <ns3/node.h>
#include "ns3/packet.h"
#include "ns3/ptr.h"
#include "lisp-over-ipv4.h"
#include "lisp-protocol.h"
#include "map-tables.h"

namespace ns3
{

class LispOverIpv4Impl : public LispOverIpv4
{

public:
  static TypeId GetTypeId (void);
  LispOverIpv4Impl ();
  virtual ~LispOverIpv4Impl ();

  /**
   *
   */
  void LispOutput (Ptr<Packet> packet, Ipv4Header const &innerHeader,
                   Ptr<const MapEntry> localMapping,
                   Ptr<const MapEntry> remoteMapping,
                   Ptr<Ipv4Route> lispRoute);

  /**
   *
   */
  void LispInput (Ptr<Packet> packet, Ipv4Header const &outerHeader);

  LispOverIpv4::MapStatus IsMapForEncapsulation (Ipv4Header const &innerHeader, Ptr<MapEntry> &srcMapEntry, Ptr<MapEntry> &destMapEntry, Ipv4Mask mask);

  /**
   *
   */
  bool NeedEncapsulation (Ipv4Header const &ipHeader, Ipv4Mask mask);

  /**
   *
   */
  bool NeedDecapsulation (Ptr<const Packet> packet, Ipv4Header const &ipHeader);

  /**
   *
   */
  Ptr<Packet> LispEncapsulate (Ptr<Packet> packet, uint16_t udpLength,
                        uint16_t udpSrcPort);

};

} /* namespace ns3 */

#endif /*LISP_OVER_IPV4_IMPL_H_ */
