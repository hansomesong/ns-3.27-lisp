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
#ifndef SRC_INTERNET_MODEL_LISP_CONTROL_PLANE_LISP_ENCAPSULATED_CONTROL_MSG_HEADER_H_
#define SRC_INTERNET_MODEL_LISP_CONTROL_PLANE_LISP_ENCAPSULATED_CONTROL_MSG_HEADER_H_

namespace ns3
{

// for map-requests
class LispEncapsulatedControlMsgHeader : public Header
{
public:
  static const LispControlMsg::LispControlMsgType msgType;
  LispEncapsulatedControlMsgHeader ();
  virtual
  ~LispEncapsulatedControlMsgHeader ();

private:
  uint8_t m_S:1;
  uint8_t m_D:1;
  uint8_t m_reserved:6;
};

} /* namespace ns3 */

#endif /* SRC_INTERNET_MODEL_LISP_CONTROL_PLANE_LISP_ENCAPSULATED_CONTROL_MSG_HEADER_H_ */
