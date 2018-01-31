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
#include "map-server.h"
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

NS_LOG_COMPONENT_DEFINE ("MapServer");

NS_OBJECT_ENSURE_REGISTERED (MapServer);

MapServer::MapServer ()
{
  m_socket = 0;
  m_msClientSocket = 0;
  m_event = EventId ();
  m_sent = 0;
  m_peerPort = LispOverIp::LISP_SIG_PORT;
  m_count = 100;
  m_interval = Seconds (60.0);
}

MapServer::~MapServer ()
{

}

TypeId MapServer::GetTypeId (void)
{
    static TypeId tid = TypeId ("ns3::MapServer")
        .SetParent<Application> ()
        .SetGroupName ("Lisp")
        .AddConstructor<MapServer> ()
        .AddAttribute ("MaxPackets",
                               "The maximum number of packets the application will send",
                               UintegerValue (100),
                               MakeUintegerAccessor (&MapServer::m_count),
                               MakeUintegerChecker<uint32_t> ())
        .AddAttribute (
              "PeerPort",
              "The destination port of the packet",
              UintegerValue (4342), MakeUintegerAccessor (&MapServer::m_peerPort),
              MakeUintegerChecker<uint16_t> ())
        .AddAttribute (
                    "Interval", "The time to wait between packets",
                    TimeValue (Seconds (60.0)),
                    MakeTimeAccessor (&MapServer::m_interval), MakeTimeChecker ())

      ;
      return tid;
}


void MapServer::DoDispose (void)
{
  Application::DoDispose();
}

void MapServer::StartApplication (void)
{
  NS_LOG_FUNCTION (this);
}

void MapServer::StopApplication (void)
{
  NS_LOG_FUNCTION (this);
}

void MapServer::SendMapReply (void)
{
  NS_LOG_FUNCTION (this);
}

void MapServer::HandleRead (Ptr<Socket> socket)
{
  NS_LOG_FUNCTION (this);
}

void MapServer::HandleReadFromClient (Ptr<Socket> socket)
{
  NS_LOG_FUNCTION (this);
}

} /* namespace ns3 */
