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
#ifndef SRC_INTERNET_MODEL_LISP_CONTROL_PLANE_LISP_CONTROL_MSG_H_
#define SRC_INTERNET_MODEL_LISP_CONTROL_PLANE_LISP_CONTROL_MSG_H_

#include "ns3/address.h"
#include "ns3/log.h"
#include "ns3/packet.h"
#include "ns3/ptr.h"
#include "ns3/simple-ref-count.h"

namespace ns3
{

class LispControlMsg : public SimpleRefCount<LispControlMsg>
{
public:
  LispControlMsg ();
  virtual
  ~LispControlMsg ();

  enum LispControlMsgType {
    MAP_REQUEST = 0x01,
    MAP_REPLY = 0x02,
    MAP_REGISTER = 0x03,
    MAP_NOTIFY = 0x04,
    MAP_REFERRAL = 0x06,
    MAP_ENCAP_CONTROL_MSG = 0x08,
  };

  /*
   * address families according to
   * http://www.iana.org/assignments/address-
   * family-numbers/address-family-numbers.xhtml
   */
  enum AddressFamily {
    IP = 1,
    IPV6 = 2,
  };


};

} /* namespace ns3 */

#endif /* SRC_INTERNET_MODEL_LISP_CONTROL_PLANE_LISP_CONTROL_MSG_H_ */
