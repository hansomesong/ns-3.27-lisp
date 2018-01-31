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
#include "map-notify-msg.h"

namespace ns3
{
  NS_LOG_COMPONENT_DEFINE(" MapNotifyMsg");

  const int MapNotifyMsg::AUTHEN_LEN_SIZE = 2;
  const int MapNotifyMsg::NONCE_LEN = 8;
  const int MapNotifyMsg::KEYID_LEN = 2;

  MapNotifyMsg::MapNotifyMsg ()
  {
    m_nonce = 0;
    m_recordCount = 0;
    m_record = 0;
  }

  MapNotifyMsg::~MapNotifyMsg ()
  {
    m_record = 0;
  }

  uint8_t
  MapNotifyMsg::GetRecordCount (void)
  {
    return m_recordCount;
  }

  void
  MapNotifyMsg::SetRecordCount (uint8_t count)
  {
    m_recordCount = count;
  }

  uint32_t
  MapNotifyMsg::getAuthData ()
  {
    return m_authData;
  }

  void
  MapNotifyMsg::setAuthData (uint32_t authData)
  {
    m_authData = authData;
  }

  uint16_t
  MapNotifyMsg::getKeyId ()
  {
    return m_keyID;
  }

  void
  MapNotifyMsg::setKeyId (uint16_t keyId)
  {
    m_keyID = keyId;
  }

  void
  MapNotifyMsg::SetAuthDataLen (uint16_t authDataLen)
  {
    m_authDataLen = authDataLen;
  }

  uint16_t
  MapNotifyMsg::GetAuthDataLen (void)
  {
    return m_authDataLen;
  }

  void
  MapNotifyMsg::SetRecord (Ptr<MapReplyRecord> record)
  {
    m_record = record;
  }

  Ptr<MapReplyRecord>
  MapNotifyMsg::GetRecord (void)
  {
    return m_record;
  }

  void
  MapNotifyMsg::Serialize (uint8_t *buf)
  {
    //NS_LOG_FUNCTION(this);
    uint8_t type = static_cast<uint8_t> (LispControlMsg::MAP_NOTIFY) << 4;

    buf[0] = type;
    buf[1] = 0x00;
    buf[2] = 0x00;
    buf[3] = m_recordCount;

    for (int i = 0; i < MapNotifyMsg::NONCE_LEN; i++)
      {
	buf[4 + i] = (m_nonce >> 8 * (MapNotifyMsg::NONCE_LEN - i - 1)) & 0xffff;
      }
    // Key ID field
    //TODO: set field length as constant in class!
    buf[12] = (m_keyID >> 8) & 0xff;
    buf[13] = m_keyID & 0xff;

    for (uint8_t i = 0; i < MapNotifyMsg::AUTHEN_LEN_SIZE; i++)
      {
	buf[14 + i] = (m_authDataLen >> 8 * (MapNotifyMsg::AUTHEN_LEN_SIZE - 1 - i)) & 0xff;
      }

    uint8_t size = 16;
    // Authentication data field
    buf[16] = 0xaa;
    buf[17] = 0xbb;
    buf[18] = 0xcc;
    buf[19] = 0xdd;

    size += m_authDataLen;

    m_record->Serialize (buf + size);
  }

  Ptr<MapNotifyMsg>
  MapNotifyMsg::Deserialize (uint8_t * buf)
  {
    Ptr<MapNotifyMsg> msg = Create<MapNotifyMsg> ();

    msg->SetRecordCount (buf[3]);

    uint64_t nonce = 0;

    for (int i = 0; i < MapNotifyMsg::NONCE_LEN; i++)
      {
	nonce <<= 8;
	nonce |= buf[4 + i];
      }
    msg->SetNonce (nonce);

    uint8_t size = 12;

    // Retrieve key ID field
    uint16_t keyID = 0;
    for (int i = 0; i < MapNotifyMsg::KEYID_LEN; i++)
      {
	keyID <<= 8;
	keyID |= buf[size + i];
      }

    size += MapNotifyMsg::KEYID_LEN;

    // It's very dangereous to hard code the length of authentication data
    // field!!!
    // Authen Data field always holds 2 bytes!
    uint16_t authDataLen = 0;
    for (int i = 0; i < MapNotifyMsg::AUTHEN_LEN_SIZE; i++)
      {
	authDataLen <<= 8;
	authDataLen |= buf[size + i];
      }
    msg->SetAuthDataLen (authDataLen);
    size += MapNotifyMsg::AUTHEN_LEN_SIZE;
    uint32_t authData = 0;
    for (int i = 0; i < authDataLen; i++)
      {
	authData <<= 8;
	authData |= buf[size + i];
      }
    msg->setAuthData (authData);
    // authDataLen is the number of bytes held by auth data
    size += authDataLen;

    NS_LOG_DEBUG(
	"Decoded Record Count: "<<unsigned(buf[3])<< ";Decoded Key ID: "<<keyID<< ";Authentication data length: "<<authDataLen<< ";Authentication data: "<<authData);
    //but is actually the address of the first element in buf array!
    msg->SetRecord (MapReplyRecord::Deserialize (buf + size));
    return msg;
  }

  LispControlMsg::LispControlMsgType
  MapNotifyMsg::GetMsgType (void)
  {
    return LispControlMsg::MAP_NOTIFY;
  }

  void
  MapNotifyMsg::SetNonce (uint64_t nonce)
  {
    m_nonce = nonce;
  }
  uint64_t
  MapNotifyMsg::GetNonce (void)
  {
    return m_nonce;
  }

} /* namespace ns3 */
