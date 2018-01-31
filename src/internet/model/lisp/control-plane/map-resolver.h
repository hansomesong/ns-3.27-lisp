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
#ifndef SRC_INTERNET_MODEL_LISP_CONTROL_PLANE_MAP_RESOLVER_H_
#define SRC_INTERNET_MODEL_LISP_CONTROL_PLANE_MAP_RESOLVER_H_

#include "ns3/application.h"
#include "ns3/event-id.h"
#include "ns3/ptr.h"
#include "ns3/traced-callback.h"
#include "map-request-msg.h"

namespace ns3
{

class Socket;
class Packet;

class MapResolver : public Application
{
public:
  MapResolver ();
  virtual
  ~MapResolver ();

  static TypeId GetTypeId (void);

  static void ConnectToPeerAddress (Address peerAddress, uint16_t peerPort, Ptr<Socket> socket);



protected:
  void DoDispose (void);
  uint32_t m_sent;
  uint32_t m_count;
  Time m_interval;
  EventId m_event;
  Ptr<Socket> m_mrClientSocket;
  Ptr<Socket> m_mrClientSocket6;
  Ptr<Socket> m_socket;
  uint16_t m_peerPort;
  Address m_peerAddress;

private:
  virtual void StartApplication (void);

  virtual void StopApplication (void);

  virtual void SendMapRequest (Ptr<MapRequestMsg> mapRequestMsg);

  virtual void HandleRead (Ptr<Socket> socket);

  virtual void HandleReadFromClient (Ptr<Socket> socket);


};

} /* namespace ns3 */

#endif /* SRC_INTERNET_MODEL_LISP_CONTROL_PLANE_MAP_RESOLVER_H_ */
