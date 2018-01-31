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
#ifndef SRC_INTERNET_MODEL_LISP_CONTROL_PLANE_MAP_REGISTER_MSG_H_
#define SRC_INTERNET_MODEL_LISP_CONTROL_PLANE_MAP_REGISTER_MSG_H_

#include "lisp-control-msg.h"
#include "map-reply-msg.h"
/**
 *
 *
 *
 */

namespace ns3 {

class MapRegisterMsg: public LispControlMsg {
public:
	static const LispControlMsgType msgType;
	MapRegisterMsg();
	virtual
	~MapRegisterMsg();

	uint8_t GetP(void);
	void SetP(uint8_t p);

	uint8_t GetM(void);
	void SetM(uint8_t p);

	void SetNonce(uint64_t nonce);
	uint64_t GetNonce(void);

	uint8_t GetRecordCount(void);
	void SetRecordCount(uint8_t count);

	void SetAuthDataLen(uint16_t authDataLen);
	uint16_t GetAuthDataLen(void);

	void SetRecord(Ptr<MapReplyRecord> record);
	Ptr<MapReplyRecord> GetRecord(void);

	void Serialize(uint8_t *buf);
	static Ptr<MapRegisterMsg> Deserialize(uint8_t *buf);

	static LispControlMsg::LispControlMsgType GetMsgType(void);

	uint32_t getAuthData(void);

	void setAuthData(uint32_t authData);
	uint16_t getKeyId(void);

	void setKeyId(uint16_t keyId);

	uint8_t m_P;
	uint32_t reserved; //This field MUST be set to 0 on transmit and MUST be ignored on receipt
	uint8_t m_M;
	uint8_t m_recordCount;
	uint64_t m_nonce;
	//A configured ID to find the configured Message Authentication Code (MAC) algo and key value
	//used for the authentication function.
	uint16_t m_keyID;
	uint16_t m_authDataLen; //Authentication Data Length
	uint32_t m_authData;	//Authentication Data
	Ptr<MapReplyRecord> m_record;
};

} /* namespace ns3 */

#endif /* SRC_INTERNET_MODEL_LISP_CONTROL_PLANE_MAP_REGISTER_MSG_H_ */
