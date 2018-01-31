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
#ifndef SRC_INTERNET_MODEL_LISP_CONTROL_PLANE_MAP_REQUEST_MSG_H_
#define SRC_INTERNET_MODEL_LISP_CONTROL_PLANE_MAP_REQUEST_MSG_H_

#include "ns3/address.h"
#include "ns3/lisp-control-msg.h"
#include "ns3/ptr.h"
#include "ns3/map-reply-record.h"
#include "ns3/map-request-record.h"

namespace ns3
{

class MapRequestMsg : public LispControlMsg
{
public:

  static const LispControlMsgType msgType;

  MapRequestMsg ();
  virtual
  ~MapRequestMsg ();

  void SetA (uint8_t a);
  uint8_t GetA (void);

  void SetM (uint8_t m);
  uint8_t GetM (void);

  void SetP (uint8_t p);
  uint8_t GetP (void);

  // Sollicit map request bit (smr bit) setter and getter
  void SetS (uint8_t s);
  uint8_t GetS (void);

  void SetP2 (uint8_t p);
  uint8_t GetP2 (void);
  /**
   * This is the SMR-invoked bit setter and getter.
   * This bit is set to 1 when an xTR is sending a Map-Request
   * in response to a received SMR-based Map-Request.
   */
  void SetS2 (uint8_t s);
  uint8_t GetS2 (void);

  void SetIrc (uint8_t irc);
  uint8_t GetIrc (void);

  void SetRecordCount (uint8_t recCount);
  uint8_t GetRecordCount (void);

  void SetNonce (uint64_t nonce);
  uint64_t GetNonce (void);

  void SetSourceEidAfi (AddressFamily afi);
  AddressFamily GetSourceEidAfi (void);

  void SetSourceEidAddr (Address sourceEid);
  Address GetSourceEidAddr (void);

  void SetItrRlocAddrIp (Address itrRlocAddr);
  Address GetItrRlocAddrIp (void);

  void SetItrRlocAddrIpv6 (Address itrRlocAddr);
  Address GetItrRlocAddrIpv6 (void);

  void SetMapRequestRecord (Ptr<MapRequestRecord> record);
  Ptr<MapRequestRecord> GetMapRequestRecord (void);

  void Serialize (uint8_t *buf) const;
  void SerializeOld (uint8_t *buf) const;
  static Ptr<MapRequestMsg> Deserialize (uint8_t *buf);
  static Ptr<MapRequestMsg> DeserializeOld (uint8_t *buf);
  //static Ptr<MapRequestMsg> DeserializeOld (uint8_t *buf);

  void Print (std::ostream& os) const;
  static LispControlMsg::LispControlMsgType GetMsgType (void);

private:
  uint8_t m_A:1; //!< authoritative bit, 0 for udp-based map-request sent by itr
                 //!< if 1 the destination site return the map reply rather than the mapping db system
  uint8_t m_M:1; //!< if set, map reply record segment is included
  uint8_t m_P:1; //!< Probe bit
  uint8_t m_S:1; //!< Sollicit map request bit (smr bit)
  uint8_t m_p:1; //!< P-ITR bit, set when a p-ITR sends a map request
  uint8_t m_s:1; //!< SMR-invoked bit.
  uint8_t m_reserved:2;
  /**
   * IRC:  This 5-bit field is the ITR-RLOC Count, which encodes the
      additional number of ('ITR-RLOC-AFI', 'ITR-RLOC Address') fields
      present in this message.  At least one (ITR-RLOC-AFI,
      ITR-RLOC-Address) pair MUST be encoded.  Multiple 'ITR-RLOC
      Address' fields are used, so a Map-Replier can select which
      destination address to use for a Map-Reply.  The IRC value ranges
      from 0 to 31.  For a value of 0, there is 1 ITR-RLOC address
      encoded; for a value of 1, there are 2 ITR-RLOC addresses encoded,
      and so on up to 31, which encodes a total of 32 ITR-RLOC
      addresses.
   *
   *
   */
  uint8_t m_irc;
  //!< Number of records. In this work it will be 1 all the time
  uint8_t m_recordCount;
  //!< Randomly generated nonce (must be copied in map reply)
  uint64_t m_nonce;
  //!< source eid address family
  AddressFamily m_sourceEidAfi;
  Address m_sourceEidAddress; //!< source eid Address
  // one for each address family. We only consider 2 addresses in this work.
  // The ITR put its own rloc
  Address m_itrRlocAddrIp;
  Address m_itrRlocAddrIpv6;
  /*
   * In this work, we just consider the case where
   * we have only one map request record (i.e., record count is 1)
   * see rfc 6830: "a sender MUST only send Map-Requests containing one record"
   */
  Ptr<MapRequestRecord> m_mapReqRec;

  /*
   * we don't use this in this work
   * see rfc6830 sec 6.1.2 Map-Request Message Format
   * placeholder: if we need it someday we can use it
   */
  Ptr<MapReplyRecord> mapRepRec;
};

} /* namespace ns3 */

#endif /* SRC_INTERNET_MODEL_LISP_CONTROL_PLANE_MAP_REQUEST_MSG_H_ */
