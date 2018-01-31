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
#ifndef MAPPING_SOCKET_ADDRESS_H_
#define MAPPING_SOCKET_ADDRESS_H_

#include "ns3/address.h"

namespace ns3
{

class MappingSocketAddress
{
public:
  MappingSocketAddress ();
  MappingSocketAddress (const Address address, uint8_t sockIndex);
  virtual
  ~MappingSocketAddress ();

  uint8_t GetSockIndex (void) const;
  void SetSockIndex (uint8_t sockIndex);
  Address GetAddress (void) const;
  void SetAddress (const Address address);
  /**
   * \returns a new Address instance
   *
   * Convert an instance of this class to a polymorphic Address instance.
   */
  operator Address () const;

  /**
   * \param address a polymorphic address
   * \returns an Address
   * Convert a polymorphic address to an Mac48Address instance.
   * The conversion performs a type check.
   */
  static MappingSocketAddress ConvertFrom (const Address &address);

  /**
   * \param address address to test
   * \returns true if the address matches, false otherwise.
   */
  static bool IsMatchingType (const Address &address);

private:
  /**
   * \brief Return the Type of address.
   * \return type of address
   */
  static uint8_t GetType (void);

  /**
   * \brief Convert an instance of this class to a polymorphic Address instance.
   * \returns a new Address instance
   */
  Address ConvertTo (void) const;

  uint8_t m_lispSockIndex;
  Address m_address;
};

} /* namespace ns3 */

#endif /* MAPPING_SOCKET_ADDRESS_H_ */
