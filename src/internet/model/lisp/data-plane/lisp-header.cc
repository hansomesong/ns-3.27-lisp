/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c)
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
 * Author:
 */

#include "ns3/assert.h"
#include "ns3/log.h"
#include "ns3/header.h"
#include "lisp-header.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("LispHeader");

NS_OBJECT_ENSURE_REGISTERED (LispHeader);

LispHeader::LispHeader ()
  : m_N (0),
    m_L (0),
    m_E (0),
    m_V (0),
    m_I (0),
    m_flags (0),
    m_nonce (0),
    m_lsbs (0),
    m_sourceMapVersion (0),
    m_destMapVersion (0),
    m_instanceId (0)
{

}

void LispHeader::SetNBit (uint8_t N)
{
  m_N = N;
}

uint8_t LispHeader::GetNBit (void) const
{
  return m_N;
}

void LispHeader::SetLBit (uint8_t L)
{
  m_L = L;
}

uint8_t LispHeader::GetLBit (void) const
{
  return m_L;
}

void LispHeader::SetEBit (uint8_t E)
{
  m_E = E;
}


uint8_t LispHeader::GetEBit (void) const
{
  return m_E;
}

void LispHeader::SetVBit (uint8_t V)
{
  m_V = V;
}

uint8_t LispHeader::GetVBit (void) const
{
  return m_V;
}

void LispHeader::SetIBit (uint8_t I)
{
  m_I = I;
}

uint8_t LispHeader::GetIBit (void) const
{
  return m_I;
}

void LispHeader::SetFlagsBits (uint8_t flags)
{
  m_flags = flags;
}

uint8_t LispHeader::GetFlagsBits (void) const
{
  return m_flags;
}

void LispHeader::SetNonce (uint32_t nonce)
{
  m_nonce = nonce;
}

uint32_t LispHeader::GetNonce (void) const
{
  return m_nonce;
}

void LispHeader::SetLSBs (uint32_t lsbs)
{
  m_lsbs = lsbs;
}

uint32_t LispHeader::GetLSBs (void) const
{
  return m_lsbs;
}

void LispHeader::SetSrcMapVersion (uint32_t mapVersion)
{
  m_sourceMapVersion = mapVersion;
}

uint32_t LispHeader::GetSrcMapVersion (void) const
{
  return m_sourceMapVersion;
}

void LispHeader::SetDestMapVersion (uint32_t mapVersion)
{
  m_destMapVersion = mapVersion;
}

uint32_t LispHeader:: GetDestMapVersion (void) const
{
  return m_destMapVersion;
}

void LispHeader::SetInstanceID (uint32_t instanceId)
{
  m_instanceId = instanceId;
}

uint32_t LispHeader::GetInstanceID (void) const
{
  return m_instanceId;
}

TypeId LispHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::LispHeader")
    .SetParent<Header> ()
    .SetGroupName ("Lisp")
    .AddConstructor<LispHeader> ()
  ;
  return tid;
}

TypeId LispHeader::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

void LispHeader::Print (std::ostream& os) const
{
  os << "N " << unsigned(m_N) << " "
     << "L " << unsigned(m_L) << " "
     << "E " << unsigned(m_E) << " "
     << "V " << unsigned(m_V) << " "
     << "I " << unsigned(m_I) << " "
     << "flags " << unsigned(m_flags)<< " "
     ;
  if (m_N)
     os << "Nonce " << (uint32_t) m_nonce<< " ";
  if (m_L)
    os << "Locator-Status-Bits " << (uint32_t) m_lsbs<< " ";
  if (m_V)
    {
      os << "Source Map-Version " << (uint32_t) m_sourceMapVersion<< " "
	  << "Destination Map-Version " << (uint32_t) m_destMapVersion << " ";
    }
  if (m_I)
    os << "Instance ID " << (uint32_t) m_instanceId << " )";
}

uint32_t LispHeader::GetSerializedSize () const
{
  return 8; // 8 Bytes
}

void LispHeader::Serialize (Buffer::Iterator start) const
{
  Buffer::Iterator i = start;
  uint8_t NLEVIFl = 0;

  NLEVIFl = (m_N << 7) | (m_L << 6) | (m_E << 5) | (m_V << 4) | (m_I << 3) |
      (m_flags);

  //NLEVIFl = (m_N << 6) | (m_L << 5) | (m_E << 4) | (m_V << 3) | (m_I << 2);//(m_flags);

  i.WriteU8 (NLEVIFl);
  if (m_N)
    i.WriteHtonU32 (m_nonce);
  if (m_L)
    i.WriteHtonU32(m_lsbs);
  if (m_V)
    {
      i.WriteHtonU32(m_sourceMapVersion);
      i.WriteHtonU32(m_destMapVersion);
    }
  if (m_I)
    i.WriteHtonU32(m_instanceId);
}

uint32_t LispHeader::Deserialize (Buffer::Iterator start)
{
  Buffer::Iterator i = start;
  uint32_t NLEVIFl = i.ReadU8 ();

  m_N = NLEVIFl >> 7;
  m_L = NLEVIFl >> 6;
  m_E = NLEVIFl >> 5;
  m_V = NLEVIFl >> 4;
  m_I = NLEVIFl >> 3;

  m_flags = NLEVIFl & 0x07;

  if (m_N)
    m_nonce = i.ReadNtohU32 ();
  if (m_L)
    m_lsbs = i.ReadNtohU32 ();
  if (m_V)
    {
      m_sourceMapVersion = i.ReadNtohU32 ();
      m_destMapVersion = i.ReadNtohU32 ();
    }
  if (m_I)
    m_instanceId = i.ReadNtohU32 ();

  return GetSerializedSize ();
}

} /* namespace ns3 */

