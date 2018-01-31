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
#ifndef LISP_MAPPING_SOCKET_FACTORY_H_
#define LISP_MAPPING_SOCKET_FACTORY_H_

#include "ns3/socket-factory.h"
#include "lisp-over-ip.h"
#include "ns3/ptr.h"

namespace ns3
{

class LispMappingSocketFactory : public SocketFactory
{
public:

  static TypeId GetTypeId (void);
  LispMappingSocketFactory ();
  virtual
  ~LispMappingSocketFactory ();

  void SetLisp (Ptr<LispOverIp> lisp);

  virtual Ptr<Socket> CreateSocket (void);

protected:
  virtual void DoDispose (void);

private:
  Ptr<LispOverIp> m_lisp;
};

} /* namespace ns3 */

#endif /* LISP_MAPPING_SOCKET_FACTORY_H_ */
