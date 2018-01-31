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
#include "mapping-socket-address.h"
#include "ns3/log.h"
#include "ns3/assert.h"
#include "ns3/address.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE ("MappingSocketAddress");

MappingSocketAddress::MappingSocketAddress ()
{
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG (
		  "When default constructor (where m_address and m_lispSockIndex are not explicitly initialized) is called,"
		  <<"the m_lispSockIndex is: "<<unsigned(m_lispSockIndex)<<" the m_address is: "<<m_address
  );
}

MappingSocketAddress::MappingSocketAddress (const Address address, uint8_t sockIndex)
{
  NS_LOG_FUNCTION (this << address << (uint32_t) sockIndex);
  m_address = address;
  m_lispSockIndex = sockIndex;
}

MappingSocketAddress::~MappingSocketAddress ()
{
  NS_LOG_FUNCTION (this);
}

  /**
   * \brief Convert an instance of this class to a polymorphic Address instance.
   * \returns a new Address instance
   */
Address MappingSocketAddress::ConvertTo (void) const
{
	NS_LOG_FUNCTION (this);
    uint8_t buffer[Address::MAX_SIZE];
    buffer[0] = m_lispSockIndex & 0xff;
    NS_LOG_DEBUG("m_lispSockIndex is: "<<unsigned(m_lispSockIndex)<<" m_address is: "<<m_address);
    uint32_t copied = m_address.CopyAllTo (buffer + 1, Address::MAX_SIZE - 1);
    return Address (GetType (), buffer, 1 + copied);
}

MappingSocketAddress MappingSocketAddress::ConvertFrom (const Address &address)
{
    NS_ASSERT (IsMatchingType (address));
    uint8_t buffer[Address::MAX_SIZE];
    address.CopyTo (buffer);
    uint8_t sockIndex = buffer[0];
    Address addressAux;
    addressAux.CopyAllFrom(buffer + 1, Address::MAX_SIZE - 1);
    MappingSocketAddress mappingAddress;
    mappingAddress.SetAddress (addressAux);
    mappingAddress.SetSockIndex (sockIndex);

    return mappingAddress;
}

MappingSocketAddress::operator Address () const
{
  return ConvertTo ();
}

bool
MappingSocketAddress::IsMatchingType (const Address &address)
{
  return address.IsMatchingType (GetType ());
}

uint8_t
MappingSocketAddress::GetType (void)
{
  NS_LOG_FUNCTION_NOARGS ();
  static uint8_t type = Address::Register ();
  return type;
}

uint8_t MappingSocketAddress::GetSockIndex (void) const
{
  return m_lispSockIndex;
}

void MappingSocketAddress::SetSockIndex (uint8_t sockIndex)
{
  NS_LOG_FUNCTION (this << (uint32_t) sockIndex);
  m_lispSockIndex = sockIndex;
}

Address MappingSocketAddress::GetAddress (void) const
{
  return m_address;
}

void MappingSocketAddress::SetAddress (const Address address)
{
  NS_LOG_FUNCTION (this << address);
  m_address = address;
}

} /* namespace ns3 */
