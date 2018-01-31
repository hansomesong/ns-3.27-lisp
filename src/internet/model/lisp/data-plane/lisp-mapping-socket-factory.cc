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
#include "lisp-mapping-socket-factory.h"
#include "lisp-over-ip.h"
#include "ns3/socket.h"
#include "ns3/assert.h"
namespace ns3
{

NS_OBJECT_ENSURE_REGISTERED (LispMappingSocketFactory);

TypeId LispMappingSocketFactory::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::LispMappingSocketFactory")
    .SetParent<SocketFactory> ()
    .SetGroupName ("Lisp")
  ;
  return tid;
  }

LispMappingSocketFactory::LispMappingSocketFactory ()
  : m_lisp (0)
{

}

LispMappingSocketFactory::~LispMappingSocketFactory ()
{
  NS_ASSERT (m_lisp == 0);
}

Ptr<Socket> LispMappingSocketFactory::CreateSocket (void)
{
  return m_lisp->CreateSocket ();
}

void LispMappingSocketFactory::SetLisp (Ptr<LispOverIp> lisp)
{
  m_lisp = lisp;
}

void LispMappingSocketFactory::DoDispose ()
{
  m_lisp = 0;
  SocketFactory::DoDispose ();
}

} /* namespace ns3 */
