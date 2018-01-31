/*
 * mapping-socket-msg-header.cc
 *
 *  Created on: Apr 3, 2017
 *      Author: qsong
 */

#include "mapping-socket-msg-header.h"
#include "ns3/log.h"

namespace ns3 {
NS_LOG_COMPONENT_DEFINE("MappingSocketMsgHeader");
NS_OBJECT_ENSURE_REGISTERED(MappingSocketMsgHeader);

TypeId MappingSocketMsgHeader::GetTypeId(void) {
	static TypeId tid =
			TypeId("ns3::MappingSocketMsgHeader").SetParent<Header>().SetGroupName(
					"Lisp").AddConstructor<MappingSocketMsgHeader>();
	return tid;
}

TypeId MappingSocketMsgHeader::GetInstanceTypeId(void) const {
	return GetTypeId();
}

MappingSocketMsgHeader::MappingSocketMsgHeader() :
		m_mapVersion(0), m_mapType(0), m_mapFlags(0), m_mapAddresses(0), m_mapVersioning(
				0), /* Map Version number */
		m_mapRlocCount(0) /* Number of RLOCs appended*/
{
	// TODO Auto-generated constructor stub
}

MappingSocketMsgHeader::~MappingSocketMsgHeader() {
	// TODO Auto-generated destructor stub
}

void MappingSocketMsgHeader::Print(std::ostream& os) const {
	NS_LOG_FUNCTION(this << &os);

	os << "Map Version " << (int) m_mapVersion << " " << "Message Type "
			<< (int) m_mapType << " " << "Flags " << (int) m_mapFlags << " "
			<< "Addresses " << (int) m_mapAddresses << " "
			<< "Message Version Number " << (int) m_mapVersioning << " "
			<< "RLOC Number " << (int) m_mapRlocCount;
}

uint32_t MappingSocketMsgHeader::GetSerializedSize(void) const {
	return 15;
}

void MappingSocketMsgHeader::Serialize(Buffer::Iterator start) const {
	NS_LOG_FUNCTION(this << &start);
	Buffer::Iterator i = start;

	i.WriteU8(m_mapVersion);
	i.WriteU16(m_mapType);
	i.WriteU32(m_mapFlags);
	i.WriteU16(m_mapAddresses);
	i.WriteU16(m_mapVersioning);
	i.WriteU32(m_mapRlocCount);
}

uint32_t MappingSocketMsgHeader::Deserialize(Buffer::Iterator start) {
	NS_LOG_FUNCTION(this << &start);
	Buffer::Iterator i = start;

	m_mapVersion = i.ReadU8();
	m_mapType = i.ReadU16();
	m_mapFlags = i.ReadU32();
	m_mapAddresses = i.ReadU16();
	m_mapVersioning = i.ReadU16();
	m_mapRlocCount = i.ReadU32();

	return GetSerializedSize();
}

void MappingSocketMsgHeader::SetMapVersion(uint8_t mapVersion) {
	m_mapVersion = mapVersion;
}
uint8_t MappingSocketMsgHeader::GetMapVersion(void) {
	return m_mapVersion;
}

void MappingSocketMsgHeader::SetMapType(uint16_t mapType) {
	m_mapType = mapType;
}
uint16_t MappingSocketMsgHeader::GetMapType(void) {
	return m_mapType;
}

void MappingSocketMsgHeader::SetMapFlags(uint32_t mapFlags) {
	m_mapFlags = mapFlags;
}

uint32_t MappingSocketMsgHeader::GetMapFlags(void) {
	return m_mapFlags;
}

void MappingSocketMsgHeader::SetMapAddresses(uint16_t mapAddresses) {
	m_mapAddresses = mapAddresses;
}

uint16_t MappingSocketMsgHeader::GetMapAddresses(void) {
	return m_mapAddresses;
}

void MappingSocketMsgHeader::SetMapVersioning(uint16_t mapVersioning) {
	m_mapVersioning = mapVersioning;
}

uint16_t MappingSocketMsgHeader::GetMapVersioning(void) {
	return m_mapVersioning;
}

void MappingSocketMsgHeader::SetMapRlocCount(uint32_t mapRlocCount) {
	m_mapRlocCount = mapRlocCount;
}

uint32_t MappingSocketMsgHeader::GetMapRlocCount(void) {
	return m_mapRlocCount;
}

} /* namespace ns3 */
