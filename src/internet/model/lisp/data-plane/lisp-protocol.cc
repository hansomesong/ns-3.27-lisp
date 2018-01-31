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

#include "lisp-protocol.h"
#include <ns3/assert.h>
#include "lisp-header.h"
#include "map-tables.h"
#include "lisp-header.h"
#include "string"
#include "lisp-mapping-socket.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE ("LispProtocol");

// LispStatistics
TypeId LispStatistics::GetTypeId (void)
{
  // TODO Add attributes
  static TypeId tid = TypeId ("ns3::LispStatistics")
  .SetParent<Object> ()
  .SetGroupName ("Lisp")
  .AddConstructor<LispStatistics> ()
  ;
  return tid;
}

LispStatistics::LispStatistics (void)
  : m_inputPackets (0),
    m_inputDifAfPackets (0),
    m_badSizePackets (0),
    m_noLocMapPresent (0),
    m_badDataLength (0),
    m_badSourceVersionNumber (0),
    m_badDestVesionNumber (0),
    m_outputPackets (0),
    m_outputDifAfPackets (0),
    m_cacheMissPackets (0),
    m_noValidRlocPackets (0),
    m_noValidMtuPackets (0),
    m_noEnoughBufferPacket (0),
    m_outputDropPackets (0)
{
  NS_LOG_FUNCTION (this);
}

LispStatistics::~LispStatistics ()
{

}

void LispStatistics::NoLocalMap (void)
{
  m_noLocMapPresent++;
}
void LispStatistics::BadDestVersionNumber (void)
{
  m_badDestVesionNumber++;
}
void LispStatistics::BadSrcVersionNumber (void)
{
  m_badSourceVersionNumber++;
}
void LispStatistics::IncInputPacket (void)
{
  m_inputPackets++;
}
void LispStatistics::IncBadSizePackets (void)
{
  m_badSizePackets++;
}
void LispStatistics::IncInputDifAfPackets (void)
{
  m_inputDifAfPackets++;
}
void LispStatistics::IncCacheMissPackets (void)
{
  m_cacheMissPackets++;
}
void LispStatistics::IncNoValidRloc (void)
{
  m_noValidRlocPackets++;
}
void LispStatistics::IncOutputDropPackets (void)
{
  m_outputDropPackets++;
}
void LispStatistics::IncNoValidMtuPackets (void)
{
  m_noValidMtuPackets++;
}
void LispStatistics::IncNoEnoughSpace (void)
{
  m_noEnoughBufferPacket++;
}

void LispStatistics::IncOutputPackets (void)
{
  m_outputPackets++;
}
void LispStatistics::IncOutputDifAfPackets (void)
{
  m_outputDifAfPackets++;
}








} /* namespace ns3 */
