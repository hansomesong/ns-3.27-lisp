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
#ifndef SRC_INTERNET_MODEL_LISP_CONTROL_PLANE_MAP_REPLY_MSG_H_
#define SRC_INTERNET_MODEL_LISP_CONTROL_PLANE_MAP_REPLY_MSG_H_

#include "ns3/lisp-control-msg.h"
#include "ns3/map-reply-record.h"
#include "ns3/map-tables.h"
#include "ns3/map-request-msg.h"

namespace ns3
{

class MapReplyMsg : public LispControlMsg
{
public:

  static const LispControlMsgType msgType;
  MapReplyMsg ();
  virtual
  ~MapReplyMsg ();

  void SetE (uint8_t m);
  uint8_t GetE (void);

  void SetP (uint8_t p);
  uint8_t GetP (void);

  void SetS (uint8_t s);
  uint8_t GetS (void);

  void SetRecordCount (uint8_t recCount);
  uint8_t GetRecordCount (void);

  void SetNonce (uint64_t nonce);
  uint64_t GetNonce (void);

  void SetRecord (Ptr<MapReplyRecord> record);
  Ptr<MapReplyRecord> GetRecord (void);

  void Serialize (uint8_t *buf) const;
  static Ptr<MapReplyMsg> Deserialize (uint8_t *buf);

  void Print (std::ostream& os) const;
  static LispControlMsg::LispControlMsgType GetMsgType (void);

private:
  Ptr<MapReplyRecord> m_record;
  uint8_t m_recordCount;
  uint8_t m_P:1; //!< Probe bit
  uint8_t m_E:1; //!< Echo Nonce enable bit
  uint8_t m_S:1; //!< Security bit
  uint8_t m_reserved:5;
  uint64_t m_nonce; //!< Nonce echoed from the map request

};

std::ostream& operator<< (std::ostream& os, MapReplyMsg const& mapReply);

} /* namespace ns3 */

#endif /* SRC_INTERNET_MODEL_LISP_CONTROL_PLANE_MAP_REPLY_MSG_H_ */
