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

#include "lisp-over-ipv6-impl.h"

#include <ns3/packet.h>
#include <ns3/ptr.h>
#include "ns3/log.h"
#include "lisp-protocol.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE ("LispOverIpv6Impl");

NS_OBJECT_ENSURE_REGISTERED (LispOverIpv6Impl);


TypeId
LispOverIpv6Impl::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::LispOverIpv6Impl")
    .SetParent<LispOverIpv6> ()
    .SetGroupName ("Lisp")
    .AddConstructor<LispOverIpv6Impl> ()
  ;
  return tid;
}

LispOverIpv6Impl::LispOverIpv6Impl ()
{
  NS_LOG_FUNCTION (this);
}

LispOverIpv6Impl::~LispOverIpv6Impl ()
{
  NS_LOG_FUNCTION (this);
}

Ptr<Packet>
LispOverIpv6Impl::LispEncapsulate (Ptr<Packet> packet, uint16_t udpLength, uint16_t udpSrcPort)
{
  return packet;
}

LispOverIpv4::MapStatus LispOverIpv6Impl::IsMapForEncapsulation (Ptr<const Packet> p, Ptr<MapEntry> srcMapEntry, Ptr<MapEntry> destMapEntry) const
{
  return LispOverIpv4::No_Mapping;
}

} /* namespace ns3 */
