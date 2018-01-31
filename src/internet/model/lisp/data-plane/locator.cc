/*
 * locator.cc
 *
 *  Created on: Apr 1, 2017
 *      Author: qsong
 */

#include "locator.h"
#include "ns3/log.h"
#include "ns3/assert.h"

namespace ns3 {
NS_LOG_COMPONENT_DEFINE("Locator");
// Locator
Locator::Locator() {
	NS_LOG_FUNCTION(this);
	m_rlocAddress = static_cast<Address>(Ipv4Address());
}
Locator::Locator(Address rlocAddress) {
	NS_ASSERT(
			Ipv4Address::IsMatchingType(rlocAddress)
					|| Ipv6Address::IsMatchingType(rlocAddress));

	m_rlocAddress = rlocAddress;
	m_metrics = Create<RlocMetrics>(0, 0, 1);
}

Locator::~Locator() {
}

Address Locator::GetRlocAddress(void) const {
	return m_rlocAddress;
}

void Locator::SetRlocAddress(const Address &rlocAddress) {
	NS_ASSERT(
			Ipv4Address::IsMatchingType(rlocAddress)
					|| Ipv6Address::IsMatchingType(rlocAddress));
	m_rlocAddress = rlocAddress;
}

Ptr<RlocMetrics> Locator::GetRlocMetrics(void) const {
	return m_metrics;
}

void Locator::SetRlocMetrics(Ptr<RlocMetrics> metrics) {
	m_metrics = metrics;
}

std::string Locator::Print(void) {
	std::string locator = std::string();
	std::stringstream str;
	if (Ipv4Address::IsMatchingType(m_rlocAddress)) {
		Ipv4Address::ConvertFrom(m_rlocAddress).Print(str);
		locator += "\tRLOC address: " + str.str() + "\n";
	} else if (Ipv6Address::IsMatchingType(m_rlocAddress)) {
		// TODO do same for Ipv6
		Ipv6Address::ConvertFrom(m_rlocAddress).Print(str);
		locator += "\tRLOC address: " + str.str() + "\n";
	}

	if (m_metrics != 0)
		locator += "\t" + m_metrics->Print();

	str.str(std::string());
	return locator;
}

uint8_t Locator::Serialize(uint8_t *buf) {
	uint8_t size = 0;
	if (Ipv4Address::IsMatchingType(m_rlocAddress)) {
		buf[0] = 1;
		Ipv4Address::ConvertFrom(m_rlocAddress).Serialize(buf + 1);
		size += 5;
		if (m_metrics) {
			buf[5] = 1;
			size += m_metrics->Serialize(buf + 6);
		} else
			buf[5] = 0;
	} else {
		buf[0] = 0;
		Ipv6Address::ConvertFrom(m_rlocAddress).Serialize(buf + 1);
		size += 17;
		if (m_metrics) {
			buf[17] = 1;
			size += m_metrics->Serialize(buf + 18);
		} else
			buf[17] = 0;
	}
	size++; // metrix present indicator
	return size;
}

Ptr<Locator> Locator::DeserializedInMapReplyRecord(const uint8_t *buf) {
	NS_ASSERT(buf);
	uint8_t position=0;
	Ptr<Locator> locator = Create<Locator>();
	Ptr<RlocMetrics> rlocMetrics = RlocMetrics::DeserializedInMapReplyRecord(buf);
	locator->SetRlocMetrics(rlocMetrics);
	//According to RFC6830, fields from priority to Loc-AFI
	position += 8;
	if (rlocMetrics->GetLocAfi() == RlocMetrics::IPv4 ){
		locator->SetRlocAddress(static_cast<Address>(Ipv4Address::Deserialize(buf + position)));
	}
	else if (rlocMetrics->GetLocAfi() == RlocMetrics::IPv6 ){
		locator->SetRlocAddress(static_cast<Address>(Ipv4Address::Deserialize(buf + position)));
	}
	return locator;
}

// When we are sure Locator::DeserializedInMapReplyRecord(const uint8_t *buf)
// works well, we can delete the following memebr method.
Ptr<Locator> Locator::Deserialized(const uint8_t *buf) {
	NS_ASSERT(buf);

	Ptr<Locator> locator = Create<Locator>();

	if (buf[0]) {
		locator->SetRlocAddress(
				static_cast<Address>(Ipv4Address::Deserialize(buf + 1)));
		if (buf[5])
			locator->SetRlocMetrics(RlocMetrics::Deserialized(buf + 6));
	} else {
		locator->SetRlocAddress(
				static_cast<Address>(Ipv4Address::Deserialize(buf + 1)));
		if (buf[17])
			locator->SetRlocMetrics(RlocMetrics::Deserialized(buf + 18));
	}
	return locator;
}

} /* namespace ns3 */
