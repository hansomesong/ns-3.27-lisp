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
#include "map-register-msg.h"

namespace ns3 {
NS_LOG_COMPONENT_DEFINE ("MapRegisterMsg");

MapRegisterMsg::MapRegisterMsg() {
	m_P = 0;
	m_M = 0;
	m_nonce = 0;
	m_recordCount = 0;
	m_record = 0;
}

MapRegisterMsg::~MapRegisterMsg() {
	m_record = 0;
}

uint8_t MapRegisterMsg::GetP(void) {
	return m_P;
}

void MapRegisterMsg::SetP(uint8_t p) {
	m_P = p;
}

uint8_t MapRegisterMsg::GetM(void) {
	return m_M;
}

void MapRegisterMsg::SetM(uint8_t m) {
	m_M = m;
}

uint8_t MapRegisterMsg::GetRecordCount(void) {
	return m_recordCount;
}

void MapRegisterMsg::SetRecordCount(uint8_t count) {
	m_recordCount = count;
}


uint32_t MapRegisterMsg::getAuthData(){
	return m_authData;
}

void MapRegisterMsg::setAuthData(uint32_t authData) {
	m_authData = authData;
}

uint16_t MapRegisterMsg::getKeyId(){
	return m_keyID;
}

void MapRegisterMsg::setKeyId(uint16_t keyId) {
	m_keyID = keyId;
}

void MapRegisterMsg::SetAuthDataLen(uint16_t authDataLen){
	m_authDataLen = authDataLen;
}

uint16_t MapRegisterMsg::GetAuthDataLen(void){
	return m_authDataLen;
}


uint64_t GetNonce(void);

void MapRegisterMsg::SetRecord(Ptr<MapReplyRecord> record) {
	m_record = record;
}

Ptr<MapReplyRecord> MapRegisterMsg::GetRecord(void) {
	return m_record;
}

void MapRegisterMsg::Serialize(uint8_t *buf) {
	//NS_LOG_FUNCTION(this);
	uint8_t type = static_cast<uint8_t>(LispControlMsg::MAP_REGISTER) << 4;

	buf[0] = (type) | (m_P << 3);
	buf[1] = 0x00;
	buf[2] = m_M;
	buf[3] = m_recordCount;

	int nonce_size = 8;
	for(int i=0; i< nonce_size; i++){
		buf[4+i] = (m_nonce >> 8*(nonce_size-i-1)) & 0xffff;
	}
	// Key ID field
	buf[12] = (m_keyID >> 8 ) & 0xff;
	buf[13] = m_keyID & 0xff;
	// Authentication data length field
	// This is the length in octets of the authentication data field that follows this field
	int authen_len_size = 2;

	for (int i=0; i< authen_len_size;i++){
		buf[14+i] = (m_authDataLen >> 8*(authen_len_size - 1 -i)) & 0xff;
	}

	uint8_t size = 16;
	// Authentication data field
	buf[16] = 0xaa;
	buf[17] = 0xbb;
	buf[18] = 0xcc;
	buf[19] = 0xdd;

	size += m_authDataLen;

	m_record->Serialize(buf + size);
}

Ptr<MapRegisterMsg> MapRegisterMsg::Deserialize(uint8_t *buf) {
	Ptr<MapRegisterMsg> msg = Create<MapRegisterMsg>();

	msg->SetP((buf[0] >> 3) & 0x01);
	msg->SetM(buf[2]);

	msg->SetRecordCount(buf[3]);

	uint64_t nonce = 0;

	int nonce_size = 8;

	for(int i=0; i< nonce_size; i++){
		nonce <<= 8;
		nonce |= buf[4+i];
	}
	msg->SetNonce(nonce);
	uint8_t size = 12;

	// Retrieve key ID field
	int keyID_size = 2;
	uint16_t keyID = 0;
	for(int i=0; i< keyID_size; i++){
		keyID <<= 8;
		keyID |= buf[size+i];
	}

	size += keyID_size;

	// It's very dangereous to hard code the length of authentication data
	// field!!!
	// Authen Data field always holds 2 bytes!
	int authDataLen_size = 2;
	uint16_t authDataLen = 0;
	for(int i=0; i< authDataLen_size; i++){
		authDataLen <<= 8;
		authDataLen |= buf[size+i];
	}
	msg->SetAuthDataLen(authDataLen);
	size += authDataLen_size;
	uint32_t authData = 0;
	for(int i=0; i< authDataLen; i++){
		authData <<= 8;
		authData |= buf[size+i];
	}
	msg->setAuthData(authData);
	// authDataLen is the number of bytes held by auth data
	size += authDataLen;

	NS_LOG_DEBUG(
			"Decoded Record Count: "<<unsigned(buf[3])<<
			";Decoded Key ID: "<<keyID<<
			";Authentication data length: "<<authDataLen<<
			";Authentication data: "<<authData
	);
	//but is actually the address of the first element in buf array!
	msg->SetRecord(MapReplyRecord::Deserialize(buf + size));
	return msg;
}

LispControlMsg::LispControlMsgType MapRegisterMsg::GetMsgType(void) {
	return LispControlMsg::MAP_REGISTER;
}

void MapRegisterMsg::SetNonce(uint64_t nonce) {
	m_nonce = nonce;
}
uint64_t MapRegisterMsg::GetNonce(void) {
	return m_nonce;
}

} /* namespace ns3 */
