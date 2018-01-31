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
#include "map-reply-msg.h"

#include "map-reply-record.h"
#include "ns3/simple-map-tables.h"

namespace ns3 {


MapReplyMsg::MapReplyMsg() {
	m_P = 0;
	m_E = 0;
	m_S = 0;
}

MapReplyMsg::~MapReplyMsg() {
}

void MapReplyMsg::SetE(uint8_t e) {
	m_E = e;
}
uint8_t MapReplyMsg::GetE(void) {
	return m_E;
}

void MapReplyMsg::SetP(uint8_t p) {
	m_P = p;
}
uint8_t MapReplyMsg::GetP(void) {
	return m_P;
}

void MapReplyMsg::SetS(uint8_t s) {
	m_S = s;
}
uint8_t MapReplyMsg::GetS(void) {
	return m_S;
}

void MapReplyMsg::SetRecordCount(uint8_t recCount) {
	m_recordCount = recCount;
}
uint8_t MapReplyMsg::GetRecordCount(void) {
	return m_recordCount;
}

void MapReplyMsg::SetNonce(uint64_t nonce) {
	m_nonce = nonce;
}
uint64_t MapReplyMsg::GetNonce(void) {
	return m_nonce;
}

void MapReplyMsg::Serialize(uint8_t *buf) const {
	uint8_t msg_type = static_cast<uint8_t>(GetMsgType());
	uint8_t type_EPS = 0;
	uint8_t position = 0;
	type_EPS = (msg_type << 4) | (m_P << 3) | (m_E << 2) | (m_S << 1);
	buf[0] = type_EPS;
	buf[1] = m_reserved; // should be 0
	buf[2] = 0x00; // rest part of m_reserved, all bits are 0;
	buf[3] = m_recordCount;
	position += 4;
	buf[position] = (m_nonce >> 56) & 0xffff;
	buf[position+1] = (m_nonce >> 48) & 0xffff;
	buf[position+2] = (m_nonce >> 40) & 0xffff;
	buf[position+3] = (m_nonce >> 32) & 0xffff;
	buf[position+4] = (m_nonce >> 24) & 0xffff;
	buf[position+5] = (m_nonce >> 16) & 0xffff;
	buf[position+6] = (m_nonce >> 8) & 0xffff;
	buf[position+7] = (m_nonce >> 0) & 0xffff;
	position += 8;

	m_record->Serialize(buf + position);
}
Ptr<MapReplyMsg> MapReplyMsg::Deserialize(uint8_t *buf) {
	Ptr<MapReplyMsg> msg = Create<MapReplyMsg>();
	uint8_t position = 0;
	// The first bits are about Type, in this case 2
	// Do not forget to take and bitwise operation with 0x01 to extract each flag value
	uint8_t EPSres = buf[0];
	msg->SetP((EPSres >> 3) & 0x01);
	msg->SetE((EPSres >> 2) & 0x01);
	msg->SetS((EPSres >> 1) & 0x01);

	msg->SetRecordCount(buf[3]);
	position += 4;
	uint64_t nonce = 0;
	nonce |= buf[position];
	nonce <<= 8;
	nonce |= buf[position+1];
	nonce <<= 8;
	nonce |= buf[position+2];
	nonce <<= 8;
	nonce |= buf[position+3];
	nonce <<= 8;
	nonce |= buf[position+4];
	nonce <<= 8;
	nonce |= buf[position+5];
	nonce <<= 8;
	nonce |= buf[position+6];
	nonce <<= 8;
	nonce |= buf[position+7];

	msg->SetNonce(nonce);
	position += 8;
	msg->SetRecord(MapReplyRecord::Deserialize(buf + position));
	return msg;
}

void MapReplyMsg::SetRecord(Ptr<MapReplyRecord> record) {
	m_record = record;
}
Ptr<MapReplyRecord> MapReplyMsg::GetRecord(void) {
	return m_record;
}

void MapReplyMsg::Print(std::ostream& os) const {
	os << "E: " << unsigned(m_E) << " P: " << unsigned(m_P) << " S: "
			<< unsigned(m_S) << " Nonce: " << unsigned(m_nonce)
			<< " Record Count: " << unsigned(m_recordCount) << " Reply-Record: ";
	m_record->Print(os);
	os << std::endl;
}

LispControlMsg::LispControlMsgType MapReplyMsg::GetMsgType(void) {
	return LispControlMsg::MAP_REPLY;
}

std::ostream& operator<< (std::ostream& os, MapReplyMsg const& mapReply){
  mapReply.Print(os);
  return os;
}


} /* namespace ns3 */
