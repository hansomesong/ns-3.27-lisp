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
#include "map-request-msg.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("MapRequestMsg");

const LispControlMsg::LispControlMsgType msgType = LispControlMsg::MAP_REQUEST;

MapRequestMsg::MapRequestMsg() :
		m_A(0), m_M(0), m_P(0), m_S(0), m_p(0), m_s(0), m_reserved(0), m_irc(0), m_recordCount(
				1), m_nonce(0) {
	m_sourceEidAfi = LispControlMsg::IP;
	m_sourceEidAddress = static_cast<Address>(Ipv4Address());
	m_itrRlocAddrIp = static_cast<Address>(Ipv4Address());
	m_itrRlocAddrIpv6 = static_cast<Address>(Ipv6Address());
	m_mapReqRec = 0;
}

MapRequestMsg::~MapRequestMsg() {
	// TODO Auto-generated destructor stub
}

void MapRequestMsg::SetA(uint8_t a) {
	m_A = a;
}

uint8_t MapRequestMsg::GetA(void) {
	return m_A;
}

void MapRequestMsg::SetM(uint8_t m) {
	m_M = m;
}
uint8_t MapRequestMsg::GetM(void) {
	return m_M;
}

void MapRequestMsg::SetP(uint8_t p) {
	m_P = p;
}
uint8_t MapRequestMsg::GetP(void) {
	return m_P;
}

void MapRequestMsg::SetS(uint8_t s) {
	m_S = s;
}
uint8_t MapRequestMsg::GetS(void) {
	return m_S;
}

void MapRequestMsg::SetP2(uint8_t p) {
	m_p = p;
}
uint8_t MapRequestMsg::GetP2(void) {
	return m_p;
}

void MapRequestMsg::SetS2(uint8_t s) {
	m_s = s;
}
uint8_t MapRequestMsg::GetS2(void) {
	return m_s;
}

void MapRequestMsg::SetIrc(uint8_t irc) {
	m_irc = irc;
}
uint8_t MapRequestMsg::GetIrc(void) {
	return m_irc;
}

void MapRequestMsg::SetRecordCount(uint8_t recCount) {
	m_recordCount = recCount;
}
uint8_t MapRequestMsg::GetRecordCount(void) {
	return m_recordCount;
}

void MapRequestMsg::SetNonce(uint64_t nonce) {
	m_nonce = nonce;
}
uint64_t MapRequestMsg::GetNonce(void) {
	return m_nonce;
}

void MapRequestMsg::SetSourceEidAfi(AddressFamily afi) {
	m_sourceEidAfi = afi;
}

LispControlMsg::AddressFamily MapRequestMsg::GetSourceEidAfi(void) {
	return m_sourceEidAfi;
}

void MapRequestMsg::SetSourceEidAddr(Address sourceEid) {
	m_sourceEidAddress = sourceEid;
}
Address MapRequestMsg::GetSourceEidAddr(void) {
	return m_sourceEidAddress;
}

void MapRequestMsg::SetItrRlocAddrIp(Address itrRlocAddr) {
	m_itrRlocAddrIp = itrRlocAddr;
}
Address MapRequestMsg::GetItrRlocAddrIp(void) {
	return m_itrRlocAddrIp;
}

void MapRequestMsg::SetItrRlocAddrIpv6(Address itrRlocAddr) {
	m_itrRlocAddrIpv6 = itrRlocAddr;
}
Address MapRequestMsg::GetItrRlocAddrIpv6(void) {
	return m_itrRlocAddrIpv6;
}

void MapRequestMsg::SetMapRequestRecord(Ptr<MapRequestRecord> record) {
	m_mapReqRec = record;
}

Ptr<MapRequestRecord> MapRequestMsg::GetMapRequestRecord(void) {
	return m_mapReqRec;
}

void MapRequestMsg::Serialize(uint8_t *buf) const {
	uint8_t type = static_cast<uint8_t>(LispControlMsg::MAP_REQUEST);
	uint8_t type_AMPS = 0;

	type_AMPS = type << 4 |(m_A << 3) | (m_M << 2) | (m_P << 1) | m_S;
	// Byte for flag p,s and 6 bits of reserved field
	uint8_t ps_partialReseverd = 0 | (m_p << 7) | (m_s << 6);

	buf[0] = type_AMPS;
	buf[1] = ps_partialReseverd;
	buf[2] = m_reserved | m_irc;
	buf[3] = m_recordCount;

	buf[4] = (m_nonce >> 56) & 0xffff;
	buf[5] = (m_nonce >> 48) & 0xffff;
	buf[6] = (m_nonce >> 40) & 0xffff;
	buf[7] = (m_nonce >> 32) & 0xffff;
	buf[8] = (m_nonce >> 24) & 0xffff;
	buf[9] = (m_nonce >> 16) & 0xffff;
	buf[10] = (m_nonce >> 8) & 0xffff;
	buf[11] = (m_nonce >> 0) & 0xffff;

	buf[12] = 0x00;
	buf[13] = static_cast<uint8_t>(m_sourceEidAfi);

	uint8_t size = 14;
	if (m_sourceEidAfi == LispControlMsg::IP) {
		Ipv4Address::ConvertFrom(m_sourceEidAddress).Serialize(buf + size);
		size += 4;
	} else if (m_sourceEidAfi == LispControlMsg::IPV6) {
		Ipv6Address::ConvertFrom(m_sourceEidAddress).Serialize(buf + size);
		size += 16;
	}
	// Now serialize ITR-RLOC-AFI 1 and ITR-RLOC-Address 1
	if (!Ipv4Address().IsEqual(Ipv4Address::ConvertFrom(m_itrRlocAddrIp))) {
		buf[size] =0x00;
		size += 1;
		buf[size] = 1;
		size++;
		Ipv4Address::ConvertFrom(m_itrRlocAddrIp).Serialize(buf + size);
		size += 4;
	} else {
		buf[size] = 0;
		size++;
	}

//	if (!Ipv6Address::ConvertFrom(m_itrRlocAddrIpv6).IsEqual(Ipv6Address())) {
//		buf[size] = 1;
//		size++;
//		Ipv6Address::ConvertFrom(m_itrRlocAddrIpv6).Serialize(buf + size);
//		size += 16;
//	} else {
//		buf[size] = 0;
//		size++;
//	}

	m_mapReqRec->Serialize(buf + size);
}

void MapRequestMsg::SerializeOld(uint8_t *buf) const {
	uint8_t AMPSpsres = 0;

	AMPSpsres = (m_A << 7) | (m_M << 6) | (m_P << 5) | (m_S << 4) | (m_p << 3)
			| (m_s << 2) | (m_reserved);

	buf[0] = static_cast<uint8_t>(LispControlMsg::MAP_REQUEST);
	buf[1] = AMPSpsres;
	buf[2] = m_irc;
	buf[3] = m_recordCount;

	buf[4] = (m_nonce >> 56) & 0xffff;
	buf[5] = (m_nonce >> 48) & 0xffff;
	buf[6] = (m_nonce >> 40) & 0xffff;
	buf[7] = (m_nonce >> 32) & 0xffff;
	buf[8] = (m_nonce >> 24) & 0xffff;
	buf[9] = (m_nonce >> 16) & 0xffff;
	buf[10] = (m_nonce >> 8) & 0xffff;
	buf[11] = (m_nonce >> 0) & 0xffff;

	buf[12] = static_cast<uint8_t>(m_sourceEidAfi);

	uint8_t size = 13;
	if (m_sourceEidAfi == LispControlMsg::IP) {
		Ipv4Address::ConvertFrom(m_sourceEidAddress).Serialize(buf + size);
		size += 4;
	} else if (m_sourceEidAfi == LispControlMsg::IPV6) {
		Ipv6Address::ConvertFrom(m_sourceEidAddress).Serialize(buf + size);
		size += 16;
	}

	if (!Ipv4Address().IsEqual(Ipv4Address::ConvertFrom(m_itrRlocAddrIp))) {
		buf[size] = 0x00;
		size += 1;
		buf[size] = 1;
		size++;
		Ipv4Address::ConvertFrom(m_itrRlocAddrIp).Serialize(buf + size);
		size += 4;
	} else {
		buf[size] = 0;
		size++;
	}

//	if (!Ipv6Address::ConvertFrom(m_itrRlocAddrIpv6).IsEqual(Ipv6Address())) {
//		buf[size] = 1;
//		size++;
//		Ipv6Address::ConvertFrom(m_itrRlocAddrIpv6).Serialize(buf + size);
//		size += 16;
//	} else {
//		buf[size] = 0;
//		size++;
//	}

	m_mapReqRec->Serialize(buf + size);
}

Ptr<MapRequestMsg> MapRequestMsg::Deserialize(uint8_t *buf) {
	NS_LOG_FUNCTION_NOARGS();
	Ptr<MapRequestMsg> msg = Create<MapRequestMsg>();
	// The first bits are about Type, in this case 1
	// Do not forget to take and bitwise operation with 0x01 to extract each flag value
	uint8_t type_AMPS = buf[0];
	msg->SetA((type_AMPS >> 3) & 0x01);
	msg->SetM((type_AMPS >> 2) & 0x01);
	msg->SetP((type_AMPS >> 1) & 0x01);
	msg->SetS(type_AMPS & 0x01);
	uint8_t ps_partialReseverd = buf[1];

	msg->SetP2(ps_partialReseverd >> 7);
	msg->SetS2(ps_partialReseverd >> 6);

	msg->SetIrc(buf[2]);
	msg->SetRecordCount(buf[3]);

	uint64_t nonce = 0;
	nonce |= buf[4];
	nonce <<= 8;
	nonce |= buf[5];
	nonce <<= 8;
	nonce |= buf[6];
	nonce <<= 8;
	nonce |= buf[7];
	nonce <<= 8;
	nonce |= buf[8];
	nonce <<= 8;
	nonce |= buf[9];
	nonce <<= 8;
	nonce |= buf[10];
	nonce <<= 8;
	nonce |= buf[11];

	msg->SetNonce(nonce);
	//Actually, buf[12] = 0, cause source-Eid-AFi holds 2 bytes. Only the second byte counts
	msg->SetSourceEidAfi(static_cast<LispControlMsg::AddressFamily>(buf[13]));
	NS_LOG_DEBUG(
			"Decoded IRC: "<<unsigned(buf[2])<<
			";Decoded Nonce: "<<static_cast<uint64_t>(nonce)<<
			";Decoded Source EID family: "<<static_cast<LispControlMsg::AddressFamily>(buf[13])

	);
	uint8_t size = 14;

	if (msg->GetSourceEidAfi() == LispControlMsg::IP) {
		Address decoded_eid = static_cast<Address>(Ipv4Address::Deserialize(buf + size));
		msg->SetSourceEidAddr(decoded_eid);
		NS_LOG_DEBUG("Decode source EID Address: "<<Ipv4Address::ConvertFrom(decoded_eid));
		size += 4;
	} else {
		msg->SetSourceEidAddr(
				static_cast<Address>(Ipv6Address::Deserialize(buf + size)));
		size += 16;
	}
	//First byte of ITR-RLOC AFI is 00, skip it...
	size++;
	if (buf[size] == 1) {
		size++;
		Address decoded_itr_rloc = (Ipv4Address::Deserialize(buf + size));
		msg->SetItrRlocAddrIp(decoded_itr_rloc);
		NS_LOG_DEBUG("Decode ITR RLOC Address (sending map request): "<<Ipv4Address::ConvertFrom(decoded_itr_rloc));
		size += 4;
	} else {
		msg->SetItrRlocAddrIp(static_cast<Address>(Ipv4Address()));
		size++;
	}

//	if (buf[size] == 1) {
//		size++;
//		msg->SetItrRlocAddrIpv6(
//				static_cast<Address>(Ipv6Address::Deserialize(buf + size)));
//		size += 16;
//	} else {
//		msg->SetItrRlocAddrIpv6(static_cast<Address>(Ipv6Address()));
//		size++;
//	}

	msg->SetMapRequestRecord(MapRequestRecord::Deserialize(buf + size));
	return msg;
}

Ptr<MapRequestMsg> MapRequestMsg::DeserializeOld(uint8_t *buf) {
	Ptr<MapRequestMsg> msg = Create<MapRequestMsg>();

	uint8_t AMPSpsres = buf[1];
	msg->SetA(AMPSpsres >> 7);
	msg->SetM(AMPSpsres >> 6);
	msg->SetP(AMPSpsres >> 5);
	msg->SetS(AMPSpsres >> 4);
	msg->SetP2(AMPSpsres >> 3);
	msg->SetS2(AMPSpsres >> 2);

	msg->SetIrc(buf[2]);
	msg->SetRecordCount(buf[3]);

	uint64_t nonce = 0;
	nonce |= buf[4];
	nonce <<= 8;
	nonce |= buf[5];
	nonce <<= 8;
	nonce |= buf[6];
	nonce <<= 8;
	nonce |= buf[7];
	nonce <<= 8;
	nonce |= buf[8];
	nonce <<= 8;
	nonce |= buf[9];
	nonce <<= 8;
	nonce |= buf[10];
	nonce <<= 8;
	nonce |= buf[11];

	msg->SetNonce(nonce);

	msg->SetSourceEidAfi(static_cast<LispControlMsg::AddressFamily>(buf[12]));

	uint8_t size = 13;

	if (msg->GetSourceEidAfi() == LispControlMsg::IP) {
		msg->SetSourceEidAddr(
				static_cast<Address>(Ipv4Address::Deserialize(buf + size)));
		size += 4;
	} else {
		msg->SetSourceEidAddr(
				static_cast<Address>(Ipv6Address::Deserialize(buf + size)));
		size += 16;
	}

	if (buf[size] == 1) {
		size++;
		msg->SetItrRlocAddrIp(
				static_cast<Address>(Ipv4Address::Deserialize(buf + size)));
		size += 4;
	} else {
		msg->SetItrRlocAddrIp(static_cast<Address>(Ipv4Address()));
		size++;
	}

	if (buf[size] == 1) {
		size++;
		msg->SetItrRlocAddrIpv6(
				static_cast<Address>(Ipv6Address::Deserialize(buf + size)));
		size += 16;
	} else {
		msg->SetItrRlocAddrIpv6(static_cast<Address>(Ipv6Address()));
		size++;
	}

	msg->SetMapRequestRecord(MapRequestRecord::Deserialize(buf + size));
	return msg;
}

void MapRequestMsg::Print(std::ostream& os) const {
	os << "A " << unsigned(m_A) << " " << "M " << unsigned(m_M) << " " << "P "
			<< unsigned(m_P) << " " << "S " << unsigned(m_S) << " " << "p "
			<< unsigned(m_p) << " " << "s " << unsigned(m_s) << " " << "IRC "
			<< unsigned(m_irc) << " " << "Record Count "
			<< unsigned(m_recordCount) << " " << "Nonce " << unsigned(m_nonce)
			<< " " << "EID AFI " << unsigned(static_cast<int>(m_sourceEidAfi))
			<< " ";

	if (m_sourceEidAfi == LispControlMsg::IP)
		os << "Source EID " << Ipv4Address::ConvertFrom(m_sourceEidAddress)
				<< " ";
	else if (m_sourceEidAfi == LispControlMsg::IPV6)
		os << "Source EID " << Ipv6Address::ConvertFrom(m_sourceEidAddress)
				<< " ";

	if (!Ipv4Address::ConvertFrom(m_itrRlocAddrIp).IsEqual(Ipv4Address())) {
		os << "ITR RLOC v4 " << Ipv4Address::ConvertFrom(m_itrRlocAddrIp)
				<< " ";
	}
	if (!Ipv6Address::ConvertFrom(m_itrRlocAddrIpv6).IsEqual(Ipv6Address())) {
		os << "ITR RLOC v6 " << Ipv6Address::ConvertFrom(m_itrRlocAddrIpv6)
				<< " ";
	}

	os << "\nRequest Record ";
	m_mapReqRec->Print(os);
}

LispControlMsg::LispControlMsgType MapRequestMsg::GetMsgType(void) {
	return LispControlMsg::MAP_REQUEST;
}



} /* namespace ns3 */
