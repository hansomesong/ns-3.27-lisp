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
#ifndef MAP_SOCKET_MSG_HEADER_H_
#define MAP_SOCKET_MSG_HEADER_H_

#include "mapping-socket-msg-header.h"

// I do not understand why I put include syntax in source file
// in this header file. The compiler throws away lots error about MapEntryLocation

namespace ns3
{

class Address;
class Ipv6Prefix;
class Ipv4Mask;
class Locators;
class EndpointId;



class MappingSocketMsg :  public SimpleRefCount<MappingSocketMsg>
{
public:
  static TypeId GetTypeId (void);

  MappingSocketMsg ();
  MappingSocketMsg (Ptr<EndpointId> endPoint, Ptr<Locators> locatorsList);
  virtual ~MappingSocketMsg ();

  void Print (std::ostream& os);

  void SetEndPoint (Ptr<EndpointId> endpoint);
  Ptr<EndpointId> GetEndPointId (void);

  void SetLocators (Ptr<Locators> locatorsList);
  Ptr<Locators> GetLocators (void);

  void Serialize (uint8_t *buf);
  static Ptr<MappingSocketMsg> Deserialize (uint8_t *buf);

private:
  Ptr<EndpointId> m_endPoint; //!< prefix + netmask
  Ptr<Locators> m_locatorsList; //!< locators associated to the endpoint

};



} /* namespace ns3 */

#endif /* MAP_SOCKET_MSG_H_ */
