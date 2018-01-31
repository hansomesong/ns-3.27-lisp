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

#include "map-resolver.h"
#include "ns3/log.h"
#include "ns3/address-utils.h"
#include "ns3/nstime.h"
#include "ns3/socket.h"
#include "ns3/simulator.h"
#include "ns3/socket-factory.h"
#include "ns3/packet.h"
#include "ns3/uinteger.h"
#include "ns3/lisp-over-ip.h"
namespace ns3
{

NS_LOG_COMPONENT_DEFINE ("MapResolver");

NS_OBJECT_ENSURE_REGISTERED (MapResolver);

MapResolver::MapResolver ()
{
  m_socket = 0;
  m_mrClientSocket = 0;
  m_event = EventId ();
  m_sent = 0;
  m_peerPort = LispOverIp::LISP_SIG_PORT;
  m_count = 100;
  m_interval = Seconds (1.0);
}

MapResolver::~MapResolver ()
{

}

TypeId MapResolver::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::MapResolver")
      .SetParent<Application> ()
      .SetGroupName ("Lisp")
      .AddConstructor<MapResolver> ()
      .AddAttribute ("MaxPackets",
                             "The maximum number of packets the application will send",
                             UintegerValue (100),
                             MakeUintegerAccessor (&MapResolver::m_count),
                             MakeUintegerChecker<uint32_t> ())
      .AddAttribute (
            "PeerPort",
            "The destination port of the packet",
            UintegerValue (LispOverIp::LISP_SIG_PORT), MakeUintegerAccessor (&MapResolver::m_peerPort),
            MakeUintegerChecker<uint16_t> ())
      .AddAttribute (
                  "Interval", "The time to wait between packets",
                  TimeValue (Seconds (1.0)),
                  MakeTimeAccessor (&MapResolver::m_interval), MakeTimeChecker ())

    ;
    return tid;
}

void MapResolver::DoDispose (void)
{
  Application::DoDispose();
}

void MapResolver::StartApplication (void)
{
  NS_LOG_FUNCTION (this);
}

void MapResolver::StopApplication (void)
{
  NS_LOG_FUNCTION (this);
}

void MapResolver::SendMapRequest (Ptr<MapRequestMsg> mapRequestMsg)
{
  NS_LOG_FUNCTION (this);
}

void MapResolver::HandleRead (Ptr<Socket> socket)
{
  NS_LOG_FUNCTION (this);
}

void MapResolver::HandleReadFromClient (Ptr<Socket> socket)
{
  NS_LOG_FUNCTION (this);
}

void MapResolver::ConnectToPeerAddress (Address peerAddress, uint16_t peerPort, Ptr<Socket> socket)
{
  if (Ipv4Address::IsMatchingType(peerAddress) == true)
    {
      socket->Bind();
      socket->Connect (InetSocketAddress (Ipv4Address::ConvertFrom(peerAddress), peerPort));
    }
  else if (Ipv6Address::IsMatchingType(peerAddress) == true)
    {
      socket->Bind6();
      socket->Connect (Inet6SocketAddress (Ipv6Address::ConvertFrom(peerAddress), peerPort));
    }
}

} /* namespace ns3 */
