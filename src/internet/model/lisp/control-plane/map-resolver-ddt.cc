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
#include "map-resolver-ddt.h"
#include "ns3/log.h"
#include "ns3/nstime.h"
#include "ns3/socket.h"
#include "ns3/simulator.h"
#include "ns3/socket-factory.h"
#include "ns3/packet.h"
#include "ns3/uinteger.h"
#include "ns3/trace-source-accessor.h"
#include "map-request-msg.h"

namespace ns3
{
  NS_LOG_COMPONENT_DEFINE ("MapResolverDdt");

  NS_OBJECT_ENSURE_REGISTERED (MapResolverDdt);

MapResolverDdt::MapResolverDdt ()
{
  NS_LOG_DEBUG ("MapResolverDdt Application created");
}

MapResolverDdt::~MapResolverDdt ()
{
  m_socket = 0;
  m_mrClientSocket = 0;
}

TypeId MapResolverDdt::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::MapResolverDdt")
       .SetParent<Application> ()
       .SetGroupName ("Lisp")
       .AddConstructor<MapResolverDdt> ()
     ;
     return tid;
}

void MapResolverDdt::StartApplication (void)
{
  NS_LOG_FUNCTION (this);

  NS_LOG_DEBUG ("STARTING DDT MAP RESOLVER");
  if (m_socket == 0)
    {
      TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
       m_socket = Socket::CreateSocket (GetNode (), tid);
    }
  m_socket->SetRecvCallback (MakeCallback (&MapResolverDdt::HandleRead, this));
  if (m_mrClientSocket == 0)
    {
      TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
      m_mrClientSocket = Socket::CreateSocket (GetNode (), tid);
      InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), LispOverIp::LISP_SIG_PORT);
      m_mrClientSocket->Bind (local);
    }

  if (m_mrClientSocket6 == 0)
      {
        TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
        m_mrClientSocket6 = Socket::CreateSocket (GetNode (), tid);
        Inet6SocketAddress local6 = Inet6SocketAddress (Ipv6Address::GetAny (), LispOverIp::LISP_SIG_PORT);
        m_mrClientSocket6->Bind (local6);
      }
  m_mrClientSocket6->SetRecvCallback (MakeCallback (&MapResolverDdt::HandleReadFromClient, this));
  m_mrClientSocket->SetRecvCallback (MakeCallback (&MapResolverDdt::HandleReadFromClient, this));

  NS_LOG_DEBUG ("APPLICATION STARTED");
}

void MapResolverDdt::HandleReadFromClient (Ptr<Socket> socket)
{

  Ptr<Packet> packet;
  Address from;

  while ((packet = socket->RecvFrom (from)))
    {
      if (InetSocketAddress::IsMatchingType (from))
        {

        }
      else if (Inet6SocketAddress::IsMatchingType (from))
        {

        }
      uint8_t buf[packet->GetSize ()];
      packet->CopyData(buf, packet->GetSize ());
      uint8_t msg_type =buf[0] >> 4 ;

      if (msg_type == static_cast<uint8_t> (MapRequestMsg::GetMsgType ()))
        {
          NS_LOG_DEBUG ("GET map request on map resolver and Forward it to Map Server!");
          Ptr<MapRequestMsg> mapReqMsg = MapRequestMsg::Deserialize (buf);
          SendMapRequest (mapReqMsg);
        }
    }
}

void MapResolverDdt::StopApplication (void)
{
  NS_LOG_FUNCTION (this);

  if (m_socket != 0)
    {
      m_socket->Close ();
      m_socket->SetRecvCallback (MakeNullCallback<void, Ptr<Socket> > ());
      m_socket = 0;
    }

  if (m_mrClientSocket)
    {
      m_mrClientSocket->Close ();
      m_mrClientSocket->SetRecvCallback (MakeNullCallback<void, Ptr<Socket> > ());
      m_mrClientSocket = 0;
    }

  if (m_mrClientSocket6)
    {
      m_mrClientSocket6->Close ();
      m_mrClientSocket6->SetRecvCallback (MakeNullCallback<void, Ptr<Socket> > ());
      m_mrClientSocket6 = 0;
    }

  Simulator::Cancel (m_event);
}

void MapResolverDdt::SendMapRequest (Ptr<MapRequestMsg> mapRequestMsg)
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT (m_event.IsExpired ());

  uint8_t buf[64];
  mapRequestMsg->Serialize (buf);
  ConnectToPeerAddress (m_mapServerAddress, m_peerPort, m_socket);
  Ptr<Packet> p = Create<Packet> (buf, 64);
  m_socket->Send (p);
  NS_LOG_DEBUG ("Map Request sent to Map-Server!");
}

void MapResolverDdt::HandleRead (Ptr<Socket> socket)
{
  NS_LOG_FUNCTION (this);
}

void MapResolverDdt::SetMapServerAddress (Address mapServer)
{
  m_mapServerAddress = mapServer;
}

} /* namespace ns3 */
