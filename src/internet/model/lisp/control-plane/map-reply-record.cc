/*
 * MapReplyRecordNew.cpp
 *
 *  Created on: Mar 31, 2017
 *      Author: qsong
 */

#include "map-reply-record.h"
#include "ns3/simple-map-tables.h"
#include "ns3/locators-impl.h"

namespace ns3 {
NS_LOG_COMPONENT_DEFINE("MapReplyRecord");

const uint32_t MapReplyRecord::m_defaultRecordTtl = 0xffffffff;

MapReplyRecord::MapReplyRecord() :
		m_recordTtl(0), m_locatorCount(0), m_eidMaskLength(0), m_A(0), m_reserved(
				0), m_mapVersionNumber(0) {
	m_act = Drop;
	m_eidPrefixAfi = LispControlMsg::IP;
	m_eidPrefix = static_cast<Address>(Ipv4Address());
}

MapReplyRecord::~MapReplyRecord() {

}

void MapReplyRecord::SetRecordTtl(uint32_t recordTtl) {
	m_recordTtl = recordTtl;
}

uint32_t MapReplyRecord::GetRecordTtl(void) {
	return m_recordTtl;
}

void MapReplyRecord::SetLocatorCount(uint8_t locCount) {
	m_locatorCount = locCount;
}

uint8_t MapReplyRecord::GetLocatorCount(void) {
	return m_locatorCount;
}

void MapReplyRecord::SetEidMaskLength(uint8_t eidMaskLength) {
	m_eidMaskLength = eidMaskLength;
}

uint8_t MapReplyRecord::GetEidMaskLength(void) {
	return m_eidMaskLength;
}

void MapReplyRecord::SetAct(ACT act) {
	m_act = act;
}

MapReplyRecord::ACT MapReplyRecord::GetAct(void) {
	return m_act;
}

void MapReplyRecord::SetA(uint8_t a) {
	m_A = a;
}

uint8_t MapReplyRecord::GetA(void) {
	return m_A;
}

void MapReplyRecord::SetMapVersionNumber(uint16_t versionNumber) {
	m_mapVersionNumber = versionNumber;
}

uint16_t MapReplyRecord::GetMapVersionNumber(void) {
	return m_mapVersionNumber;
}

LispControlMsg::AddressFamily MapReplyRecord::GetEidAfi(void) {
	return m_eidPrefixAfi;
}

void MapReplyRecord::SetEidAfi(LispControlMsg::AddressFamily afi) {
	m_eidPrefixAfi = afi;
}

void MapReplyRecord::SetLocators(Ptr<Locators> locators) {
	m_locators = locators;
	if (locators)
		m_locatorCount = locators->GetNLocators();
	else
		m_locatorCount = 0;
}

Ptr<Locators> MapReplyRecord::GetLocators(void) {
	return m_locators;
}

void MapReplyRecord::Serialize(uint8_t *buf) {
	uint8_t size = 0;
	for (int i = 0; i < MapReplyRecord::RECORD_TTL_LEN; i++) {
		buf[size + i] = (m_recordTtl
				>> 8 * (MapReplyRecord::RECORD_TTL_LEN - 1 - i)) & 0xff;
	}
	size += MapReplyRecord::RECORD_TTL_LEN;
	buf[size] = m_locatorCount;
	size += 1;
	buf[size] = m_eidMaskLength;
	size += 1;
	//m_act is of enum type ACT, first convert to uint8_t then left shift
	buf[size] = static_cast<uint8_t>(m_act) << 5 | (m_A << 4) | 0x00;
	size += 1;
	buf[size] = 0x00; // The rest 8 bits for reserved field.
	size += 1;
	// RFC6830: Rsvd is not explained. I set it as 0000

	buf[size] = (m_mapVersionNumber >> 8) & 0xff;
	size += 1;
	buf[size] = (m_mapVersionNumber >> 0) & 0xff;
	size += 1;

	if (Ipv4Address::IsMatchingType(m_eidPrefix)) {
		// We mainly focus IPv4 (01) and IPv6(02). EID-Prefix-AFI occupy two bytes.
		// So EID-Prefix-AFI occupyies tw
		buf[size] = 0x00;
		size += 1;
		buf[size] = static_cast<uint8_t>(LispControlMsg::IP);
		// Do not forget to increment variable size after put EID-Prefix-AFI into buffer
		size++;
		Ipv4Address::ConvertFrom(m_eidPrefix).Serialize(buf + size);
		size += 4;
	} else if (Ipv6Address::IsMatchingType(m_eidPrefix)) {
		buf[size] = 0x00;
		size += 1;
		buf[size] = static_cast<uint8_t>(LispControlMsg::IPV6);
		size++;
		Ipv6Address::ConvertFrom(m_eidPrefix).Serialize(buf + size);
		size += 16;
	}

	if (m_locators) {
		for (int i = 0; i < m_locators->GetNLocators(); i++) {
			Ptr<Locator> tmp_locator = m_locators->GetLocatorByIdx(i);
			Ptr<RlocMetrics> tmp_rlocmetrics = tmp_locator->GetRlocMetrics();
			buf[size] = static_cast<uint8_t>(tmp_rlocmetrics->GetPriority());
			size++;
			buf[size] = static_cast<uint8_t>(tmp_rlocmetrics->GetWeight());
			size++;
			// Serialize M_priority and M weight
			buf[size] = 0x00;
			size++;
			buf[size] = 0x00;
			size++;
			// Serialize unused flags and L,p, R
			buf[size] = 0x00;
			size++;
			buf[size] = 0x00;
			size++;
			// Loc-AFI field. Indicating AFI family of next Locator field
			Address tmp_loc_addr = tmp_locator->GetRlocAddress();
			if (Ipv4Address::IsMatchingType(tmp_loc_addr)) {
				//if RLOC addr. is a IPv4
				// can be abstracted as a method
				buf[size] = 0x00;
				size += 1;
				buf[size] = static_cast<uint8_t>(LispControlMsg::IP);
				// Do not forget to increment variable size after put EID-Prefix-AFI into buffer
				size++;
				Ipv4Address::ConvertFrom(tmp_loc_addr).Serialize(buf + size);
				size += 4;
			} else {
				//case of ipv6. Donot care now!
				buf[size] = 0x00;
			}

//			std::cout << "ID " << i << " of "
//					<< Ipv4Address::ConvertFrom(m_eidPrefix) << std::endl;
//			std::cout << m_locators->GetLocatorByIdx(i)->Print() << std::endl;
		}
	} else
		// Do not forget to set current buffer content as zero
		// since in corresponding deserialize method, it depends current buf[size] value
		// (0 or 1) to decide whether it is necessary to decode for Locators
		buf[size] = 0;

}

Ptr<MapReplyRecord> MapReplyRecord::Deserialize(uint8_t *buf) {
	NS_LOG_FUNCTION_NOARGS ();
	Ptr<MapReplyRecord> record = Create<MapReplyRecord>();
	uint32_t recordTtl = 0;
	int size = 0;
	for (int i = 0; i < MapReplyRecord::RECORD_TTL_LEN; i++) {
		recordTtl <<= 8;
		recordTtl |= buf[size + i];
	}
	record->SetRecordTtl(recordTtl);
	size += MapReplyRecord::RECORD_TTL_LEN;
	record->SetLocatorCount(buf[size]);
	size += 1;
	uint8_t eidMaskLen = 0;
	eidMaskLen |= buf[size];
	record->SetEidMaskLength(eidMaskLen);
	size += 1;
	//current buffer element is ACT | A | Reserved (4 bits)
	record->SetAct(static_cast<ACT>(buf[size] >> 5));
	record->SetA(buf[size] >> 4);
	size += 1;
	//skip the rest part of reserved
	size += 1;

	uint16_t mapVersionNumber = 0;
	// Note that we do not consider Rsvd field!
	mapVersionNumber |= buf[size];
	size += 1;
	mapVersionNumber <<= 8;
	mapVersionNumber |= buf[size];
	record->SetMapVersionNumber(mapVersionNumber);
	NS_LOG_DEBUG(
			"Decoded Record TTL: "<<recordTtl<<
			";Decoded EID Mask length: "<<unsigned(eidMaskLen)<<
			";Decoded Map Version Number: "<<mapVersionNumber
	);
	size += 1;
	// skip the first byte of EID-Prefix-AFI
	uint16_t eid_prefix_afi = 0;
	eid_prefix_afi |= buf[size];
	eid_prefix_afi <<= 8;
	size += 1;
	eid_prefix_afi |= buf[size];
	size += 1;
	if (eid_prefix_afi == static_cast<uint16_t>(LispControlMsg::IP)) {
		record->SetEidAfi(LispControlMsg::IP);
		record->SetEidPrefix(
				static_cast<Address>(Ipv4Address::Deserialize(buf + size)));
		NS_LOG_DEBUG("Decode EID Address: "<<Ipv4Address::ConvertFrom(static_cast<Address>(Ipv4Address::Deserialize(buf + size))));
		size += 4;
	} else if (eid_prefix_afi == static_cast<uint16_t>(LispControlMsg::IPV6)) {
		record->SetEidAfi(LispControlMsg::IPV6);
		record->SetEidPrefix(
				static_cast<Address>(Ipv6Address::Deserialize(buf + size)));
		size += 16;
	}
	else{
		NS_LOG_ERROR("Unknown Address family Number:"<<eid_prefix_afi);
	}

	// Deserialize the list of RLOC (including)
	Ptr<Locators> locators = Create<LocatorsImpl>();
	for (uint8_t i = 0; i < record->GetLocatorCount(); i++) {
		Ptr<Locator> locator = Locator::DeserializedInMapReplyRecord(
				buf + size);
		locators->InsertLocator(locator);
		if (locator->GetRlocMetrics()->GetLocAfi() == RlocMetrics::IPv4) {
			// In a Map-Reply message, Locator part (within Record part)
			// holds 4+4+4 = 12 bytes;
			size += 12;
		} else if (locator->GetRlocMetrics()->GetLocAfi()
				== RlocMetrics::IPv6) {
			// Actually it is possible that a Ipv4 EID-Prefix can have a IPv6 RLOC address
			// For IPv6, Locator part holds 24 bytes
			size += 24;
		}
	}

	record->SetLocators(locators);

	return record;
}

void MapReplyRecord::SetEidPrefix(Address eidPrefix) {
	m_eidPrefix = eidPrefix;
	if (Ipv4Address::IsMatchingType(eidPrefix))
		m_eidPrefixAfi = LispControlMsg::IP;
	else
		m_eidPrefixAfi = LispControlMsg::IPV6;
}

Address MapReplyRecord::GetEidPrefix(void) {
	return m_eidPrefix;
}

void MapReplyRecord::Print(std::ostream& os) {

	os.flush();
	os << "\nEid prefix afi " << unsigned(static_cast<int>(m_eidPrefixAfi))
			<< " " << "Mask Length " << unsigned(m_eidMaskLength);
	if (m_eidPrefixAfi == LispControlMsg::IP)
		os << "EID prefix " << Ipv4Address::ConvertFrom(m_eidPrefix) << " ";
	else if (m_eidPrefixAfi == LispControlMsg::IPV6)
		os << "EID prefix " << Ipv6Address::ConvertFrom(m_eidPrefix) << " ";
	if (m_locators)
		os << "Locators: " << m_locators->Print() << std::endl;
}
} /* namespace ns3 */
