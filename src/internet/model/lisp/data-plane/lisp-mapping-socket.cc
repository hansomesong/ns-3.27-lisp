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

#include "lisp-mapping-socket.h"
#include "ns3/log.h"
#include "ns3/node.h"
#include "ns3/packet.h"
#include "ns3/uinteger.h"
#include "ns3/integer.h"

namespace ns3
{
NS_LOG_COMPONENT_DEFINE ("LispMappingSocket");

NS_OBJECT_ENSURE_REGISTERED (LispMappingSocket);

TypeId
LispMappingSocket::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::LispMappingSocket")
    .SetParent<Socket> ()
    .SetGroupName ("Lisp")
    .AddConstructor<LispMappingSocket> ()
    .AddAttribute ("RcvBufSize",
                       "LispMappingSocket maximum receive buffer size (bytes)",
                       UintegerValue (131072),
                       MakeUintegerAccessor (&LispMappingSocket::GetRcvBufSize,
                                             &LispMappingSocket::SetRcvBufSize),
                       MakeUintegerChecker<uint32_t> ())
  ;
  return tid;
}

const uint8_t LispMappingSocket::MAPM_VERSION = 1;
const LispMappingSocket::MissMsgType LispMappingSocket::m_lispMissMsgType = LispMappingSocket::LISP_MISSMSG_EID;

LispMappingSocket::LispMappingSocket ()
{
  NS_LOG_FUNCTION_NOARGS ();
  m_connected = false;
  m_shutdownRecv = false;
  m_shutdownSend = false;
  m_errno = ERROR_NOTERROR;
  m_rxAvailable = 0;
}

LispMappingSocket::~LispMappingSocket ()
{
  NS_LOG_FUNCTION_NOARGS ();
}

void LispMappingSocket::SetNode (Ptr<Node> node)
{
  NS_LOG_FUNCTION (this);
  m_node = node;
}

void LispMappingSocket::SetLisp (Ptr<LispOverIp> lisp)
{
  NS_LOG_FUNCTION (this);
  m_lisp = lisp;
}

enum Socket::SocketErrno LispMappingSocket::GetErrno () const
{
  NS_LOG_FUNCTION (this);
  return m_errno;
}

enum Socket::SocketType LispMappingSocket::GetSocketType (void) const
{
   NS_LOG_FUNCTION (this);
   return NS3_SOCK_RAW;
}

Ptr<Node> LispMappingSocket::GetNode (void) const
{
  return m_node;
}

int LispMappingSocket::Bind (const Address &address)
{
  if (!MappingSocketAddress::IsMatchingType (address))
    {
      m_errno = ERROR_INVAL;
      return -1;
    }
  return 0;
}

int LispMappingSocket::Bind (void)
{
  //m_errno = Socket::ERROR_OPNOTSUPP;
  return 0;
}

int LispMappingSocket::Bind6 (void)
{
  return Bind ();
}

int LispMappingSocket::GetSockName (Address &address) const
{
  NS_LOG_FUNCTION (this << address);
  //NS_ASSERT (m_node);
  MappingSocketAddress ad = MappingSocketAddress::ConvertFrom (address);
  ad.SetAddress (m_lisp->GetNode ()->GetDevice (0)->GetAddress ());
  ad.SetSockIndex (m_lispSockIndex);
  address = ad;
return 0;

}

int LispMappingSocket::GetPeerName (Address &address) const
{
  //Implemented by Yue.
  //I'm not sure the following implementation is correct
  NS_LOG_FUNCTION (this << address);
  address = MappingSocketAddress::ConvertFrom (m_destAddres);
  return 0;

}

int LispMappingSocket::Close (void)
{
  if (m_shutdownRecv == true && m_shutdownSend == true)
  {
    m_errno = Socket::ERROR_BADF;
    return -1;
  }

  m_shutdownRecv = true;
  m_shutdownSend = true;
  return 0;
}

int LispMappingSocket::ShutdownSend (void)
{
  m_shutdownSend = true;
  return 0;
}

int LispMappingSocket::ShutdownRecv (void)
{
  m_shutdownRecv = true;
  return 0;
}

uint32_t LispMappingSocket::GetTxAvailable (void) const
{
  return 1000;
}

int LispMappingSocket::Send (Ptr<Packet> p, uint32_t flags)
{
  return SendTo (p, flags, m_destAddres);
}

int LispMappingSocket::SendTo (Ptr<Packet> p, uint32_t flags,
                    const Address &toAddress)
{
  NS_LOG_FUNCTION (this << p << flags << toAddress);

  if (!m_connected)
    {
      NS_LOG_LOGIC ("ERROR_BADF");
      m_errno = ERROR_BADF;
      return -1;
    }
  if (m_shutdownSend)
    {
     NS_LOG_LOGIC("ERROR_SHUTDOWN");
     m_errno = ERROR_SHUTDOWN;
     return -1;
    }
  if (!MappingSocketAddress::IsMatchingType (toAddress))
    {
      NS_LOG_LOGIC ("ERROR_AFNOSUPPORT");
            m_errno = ERROR_AFNOSUPPORT;
            return -1;
    }
  MappingSocketAddress address = MappingSocketAddress::ConvertFrom (toAddress);
  uint8_t sockIndex = address.GetSockIndex ();
  NS_LOG_DEBUG ("Send a packet to: "<<address<<", address Socket Index is: "<<unsigned(sockIndex));


  Address fromAddress = static_cast<Address> (MappingSocketAddress ());
  GetSockName (fromAddress);
  Ptr<LispMappingSocket> destSocket = m_lisp->GetMappingSocket (sockIndex);
  destSocket->Forward (p, fromAddress);
  return 0;
}

std::queue<Ptr<Packet> > LispMappingSocket::GetDeliveryQueue (void)
{
  return m_deliveryQueue;
}

uint32_t LispMappingSocket::GetRxAvailable (void) const
{
  return m_rxAvailable;
}
Ptr<Packet> LispMappingSocket::Recv (uint32_t maxSize, uint32_t flags)
{
  NS_LOG_FUNCTION (this << maxSize << flags);

  if (m_deliveryQueue.empty ())
    {
      return 0;
    }

  Ptr<Packet> p = m_deliveryQueue.front ();

  if (p->GetSize () < maxSize)
    {
      m_deliveryQueue.pop ();
      m_rxAvailable -= p->GetSize ();
    }
  else
    {
      p = 0;
    }
  return p;
}

Ptr<Packet> LispMappingSocket::RecvFrom (uint32_t maxSize, uint32_t flags,
                              Address &fromAddress)
{
  return Recv (maxSize, flags);
}

bool LispMappingSocket::SetAllowBroadcast (bool allowBroadcast)
{
  return false;
}

bool LispMappingSocket::GetAllowBroadcast (void) const
{
  return false;
}

int LispMappingSocket::Connect (const Address &address)
{
  if (m_connected)
    {
      m_errno = ERROR_ISCONN;
      goto error;
    }
  else if (m_shutdownRecv && m_shutdownSend)
    {
      m_errno = ERROR_BADF;
      goto error;
    }

  if (!MappingSocketAddress::IsMatchingType (address))
    {
      m_errno = ERROR_AFNOSUPPORT;
      goto error;
    }
  m_connected = true;
  m_destAddres = address;
  NotifyConnectionSucceeded ();
  return 0;

  error:
  NotifyConnectionFailed ();
  return -1;
}

int LispMappingSocket::Listen (void)
{
  NS_LOG_FUNCTION (this);
  m_errno = Socket::ERROR_OPNOTSUPP;
  return -1;
}

void LispMappingSocket::SetSockIndex (uint8_t sockIndex)
{
  m_lispSockIndex = sockIndex;
}

void LispMappingSocket::Forward (Ptr<const Packet> packet, const Address &from)
{
  NS_LOG_FUNCTION (this << packet << from);

  if ((m_rxAvailable + packet->GetSize ()) <= m_rcvBufSize)
    {
      Ptr<Packet> copy = packet->Copy ();
      // todo Add packet address tag
      m_deliveryQueue.push (copy);
      m_rxAvailable += copy->GetSize ();
      NotifyDataRecv ();
    }
  else
    {
      NS_LOG_WARN ("No receive buffer space available.  Drop.");
    }
}

uint32_t LispMappingSocket::GetRcvBufSize (void) const
{
  return m_rcvBufSize;
}
void LispMappingSocket::SetRcvBufSize (uint32_t rcvBufSize)
{
  m_rcvBufSize = rcvBufSize;
}

/*LispMappingSocket::MissMsgType LispMappingSocket::GetMissMsgType (void)
{
  return m_lispMissMsgType;
}

void LispMappingSocket::SetMissMsgType (MissMsgType missMessageType)
{
  m_lispMissMsgType = missMessageType;
}*/



} /* namespace ns3 */
