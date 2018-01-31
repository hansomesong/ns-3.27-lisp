/*
 * map-request-record.cc
 *
 *  Created on: Apr 4, 2017
 *      Author: qsong
 */

#include "map-request-record.h"

namespace ns3 {

MapRequestRecord::MapRequestRecord() {
	m_afi = LispControlMsg::IP;
	m_eidMaskLenght = 0;
	m_eidPrefix = static_cast<Address>(Ipv4Address());
}

MapRequestRecord::MapRequestRecord(Address eidPrefix, uint8_t eidMaskLength) {
	m_afi = LispControlMsg::IP;
	m_eidPrefix = eidPrefix;
	m_eidMaskLenght = eidMaskLength;
}

MapRequestRecord::~MapRequestRecord() {

}

LispControlMsg::AddressFamily MapRequestRecord::GetAfi(void) {
	return m_afi;
}
void MapRequestRecord::SetAfi(LispControlMsg::AddressFamily afi) {
	m_afi = afi;
}

void MapRequestRecord::SetMaskLenght(uint8_t maskLenght) {
	m_eidMaskLenght = maskLenght;
}
uint8_t MapRequestRecord::GetMaskLength(void) {
	return m_eidMaskLenght;
}

void MapRequestRecord::SetEidPrefix(Address prefix) {
	m_eidPrefix = prefix;
}
Address MapRequestRecord::GetEidPrefix(void) {
	return m_eidPrefix;
}

void MapRequestRecord::Serialize(uint8_t *buf) const {
	// First byte for reserved field
	int position = 0;
	buf[position] = 0x00;
	position += 1;
	// EID mask len
	buf[position] = m_eidMaskLenght;
	position += 1;
	// 3,4th byte for EID-Prefix-AFI
	buf[position] = 0x00;
	position += 1;
	buf[position] = static_cast<uint8_t>(m_afi);
	position += 1;
	if (m_afi == LispControlMsg::IP) {
		Ipv4Address::ConvertFrom(m_eidPrefix).Serialize(buf + position);
	} else
		Ipv6Address::ConvertFrom(m_eidPrefix).Serialize(buf + position);
}

//void MapRequestRecord::SerializeOld(uint8_t *buf) const {
//	buf[0] = static_cast<uint8_t>(m_afi);
//	buf[1] = m_eidMaskLenght;
//	if (m_afi == LispControlMsg::IP) {
//		Ipv4Address::ConvertFrom(m_eidPrefix).Serialize(buf + 2);
//	} else
//		Ipv6Address::ConvertFrom(m_eidPrefix).Serialize(buf + 2);
//}

Ptr<MapRequestRecord> MapRequestRecord::Deserialize(uint8_t *buf) {
	Ptr<MapRequestRecord> record = Create<MapRequestRecord>();
	int position = 0;
	record->SetAfi(static_cast<LispControlMsg::AddressFamily>(buf[3]));
	record->SetMaskLenght(buf[1]);
	position = 4;
	if (record->GetAfi() == LispControlMsg::IP)
		record->SetEidPrefix(
				static_cast<Address>(Ipv4Address::Deserialize(buf + position)));
	else
		record->SetEidPrefix(
				static_cast<Address>(Ipv6Address::Deserialize(buf + position)));
	return record;
}

//Ptr<MapRequestRecord> MapRequestRecord::Deserialize(uint8_t *buf) {
//	Ptr<MapRequestRecord> record = Create<MapRequestRecord>();
//
//	record->SetAfi(static_cast<LispControlMsg::AddressFamily>(buf[0]));
//	record->SetMaskLenght(buf[1]);
//
//	if (record->GetAfi() == LispControlMsg::IP)
//		record->SetEidPrefix(
//				static_cast<Address>(Ipv4Address::Deserialize(buf + 2)));
//	else
//		record->SetEidPrefix(
//				static_cast<Address>(Ipv6Address::Deserialize(buf + 2)));
//	return record;
//}

void MapRequestRecord::Print(std::ostream& os) {

	os << "EID PREFIX AFI " << unsigned(static_cast<int>(m_afi)) << " "
			<< "Mask Length " << unsigned(m_eidMaskLenght);
	if (m_afi == LispControlMsg::IP)
		os << "EID prefix" << Ipv4Address::ConvertFrom(m_eidPrefix) << " ";
	else if (m_afi == LispControlMsg::IPV6)
		os << "EID prefix" << Ipv6Address::ConvertFrom(m_eidPrefix) << " ";
}

} /* namespace ns3 */
