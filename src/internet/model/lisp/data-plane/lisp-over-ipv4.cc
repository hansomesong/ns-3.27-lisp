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

#include "lisp-over-ipv4.h"
#include "ns3/log.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE ("LispOverIpv4");

NS_OBJECT_ENSURE_REGISTERED (LispOverIpv4);

TypeId
LispOverIpv4::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::LispOverIpv4")
    .SetParent<LispOverIp> ()
    .SetGroupName ("Lisp")
  ;
  return tid;
}

LispOverIpv4::LispOverIpv4 ()
{
  NS_LOG_FUNCTION (this);
}

LispOverIpv4::~LispOverIpv4 ()
{
  NS_LOG_FUNCTION (this);
}

void LispOverIpv4::RecordReceiveParams (Ptr<NetDevice> currentDevice, uint16_t protocol, NetDevice::PacketType packetType)
{
  m_currentDevice = currentDevice;
  m_ipProtocol = protocol;
  m_currentPacketType = packetType;
}



} /* namespace ns3 */
