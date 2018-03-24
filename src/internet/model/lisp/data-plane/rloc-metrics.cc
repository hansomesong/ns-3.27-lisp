/*
 * rloc-metrics.cc
 *
 *  Created on: Apr 1, 2017
 *      Author: qsong
 */

#include "rloc-metrics.h"
#include "lisp-over-ip.h"

namespace ns3 {

// RlocMetrics

//NS_OBJECT_ENSURE_REGISTERED (RlocMetrics);
/*TypeId RlocMetrics::GetTypeId (void)
 {
 // TODO Add attributes
 static TypeId tid = TypeId ("ns3::RlocMetrics")
 .SetParent<Object> ()
 .SetGroupName ("Lisp")
 .AddConstructor<RlocMetrics> ()
 ;
 return tid;
 }*/

NS_LOG_COMPONENT_DEFINE("RlocMetrics");

RlocMetrics::RlocMetrics() :
		m_priority(0), m_mpriority(0), m_weight(0), m_mweight(0), m_rlocIsUp(
				true), m_rlocIsLocalInterface(false), m_txNoncePresent(false), m_rxNoncePresent(
				false), m_txNonce(0), m_rxNonce(0), m_mtu(0) {

}

RlocMetrics::RlocMetrics(uint8_t priority, uint8_t weight) {
	m_priority = priority;
	m_weight = weight;
	// by default locator is up
	m_rlocIsUp = true;
	// mtu not set yet
	m_mtu = 0;
}

RlocMetrics::RlocMetrics(uint8_t priority, uint8_t mpriority, uint8_t weight,
		uint8_t mweight) {
	m_mpriority = mpriority;
	m_mweight = mweight;
	m_priority = priority;
	m_weight = weight;
	// by default locator is up
	m_rlocIsUp = true;
	// mtu not set yet
	m_mtu = 0;
}

RlocMetrics::RlocMetrics(uint8_t priority, uint8_t weight, bool reachable) {
	m_priority = priority;
	m_weight = weight;
	// by default locator is up
	m_rlocIsUp = reachable;
	// default 0 : mtu not set yet
	m_mtu = 0;
}

RlocMetrics::~RlocMetrics() {
}

uint8_t RlocMetrics::GetPriority(void) const {
	return m_priority;
}

void RlocMetrics::SetPriority(uint8_t priority) {
	NS_ASSERT(priority >= 0 && priority <= LispOverIp::LISP_MAX_RLOC_PRIO);
	m_priority = priority;
}

uint8_t RlocMetrics::GetWeight(void) const {
	return m_weight;
}

void RlocMetrics::SetWeight(uint8_t weight) {
	m_weight = weight;
}

uint32_t RlocMetrics::GetMtu(void) const {
	return m_mtu;
}

void RlocMetrics::SetMtu(uint32_t mtu) {
	m_mtu = mtu;
}

bool RlocMetrics::IsUp(void) {
	return m_rlocIsUp;
}

void RlocMetrics::SetUp(bool status) {
	m_rlocIsUp = true;
}

bool RlocMetrics::IsLocalInterface(void) {
	return m_rlocIsLocalInterface;
}

bool RlocMetrics::IsRxNoncePresent(void) {
	return m_rxNoncePresent;
}

void RlocMetrics::SetRxNoncePresent(bool rxNoncePresent) {
	m_rxNoncePresent = rxNoncePresent;
}

void RlocMetrics::SetIsLocalIf(bool isLocal) {
	m_rlocIsLocalInterface = isLocal;
}

bool RlocMetrics::IsTxNoncePresent(void) {
	return m_txNoncePresent;
}

uint32_t RlocMetrics::GetTxNonce(void) {
	return m_txNonce;
}

void RlocMetrics::SetTxNoncePresent(bool txNoncePresent) {
	m_txNoncePresent = txNoncePresent;
}

uint32_t RlocMetrics::GetRxNonce(void) {
	return m_rxNonce;
}
void RlocMetrics::SetRxNonce(uint32_t rxNonce) {
	m_rxNonce = rxNonce;
}

void RlocMetrics::SetTxNonce(uint32_t txNonce) {
	m_txNonce = txNonce;
}

void RlocMetrics::SetFlagL(bool flag) {

	m_flagL = flag;
}

bool RlocMetrics::GetFlagL(void) {
	return m_flagL;
}

void RlocMetrics::SetFlagP(bool flag) {
	m_flagp = flag;
}

bool RlocMetrics::GetFlagP(void) {
	return m_flagp;
}

void RlocMetrics::SetFlagR(bool flag) {
	m_flagR = flag;
}

bool RlocMetrics::GetFlagR(void) {
	return m_flagR;
}

RlocMetrics::RlocAfi RlocMetrics::GetLocAfi(void){
	return m_locAfi;
}

void RlocMetrics::SetLocAfi(RlocAfi afi){
	m_locAfi = afi;
}

std::string RlocMetrics::Print() {
	std::stringstream str;

	str << "Priority: " << unsigned(m_priority) << "\t" << "Weight: "
			<< unsigned(m_weight) << "\tRxNonce: " << m_rxNonce << "\tTxNonce: "
			<< m_txNonce << "\tMTU: " << m_mtu << "\tUP: "
			<< unsigned(m_rlocIsUp) << "\tLocal If: "
			<< unsigned(m_rlocIsLocalInterface) << "\n";
	return str.str();
}

uint8_t RlocMetrics::Serialize(uint8_t *buf) {
	buf[0] = m_priority;
	buf[1] = m_weight;
	uint8_t flags = 0;

	if (IsRxNoncePresent())
		flags |= RLOCF_RXNONCE;
	if (IsTxNoncePresent())
		flags |= RLOC_TXNONCE;
	if (IsUp())
		flags |= RLOCF_UP;
	if (IsLocalInterface())
		flags |= RLOCF_LIF;

	buf[2] = flags;

	// rx nonce
	buf[3] = (m_rxNonce >> 24) & 0xff;
	buf[4] = (m_rxNonce >> 16) & 0xff;
	buf[5] = (m_rxNonce >> 8) & 0xff;
	buf[6] = (m_rxNonce >> 0) & 0xff;

	// tx nonce
	buf[7] = (m_txNonce >> 24) & 0xff;
	buf[8] = (m_txNonce >> 16) & 0xff;
	buf[9] = (m_txNonce >> 8) & 0xff;
	buf[10] = (m_txNonce >> 0) & 0xff;

	// mtu
	buf[11] = (m_mtu >> 24) & 0xff;
	buf[12] = (m_mtu >> 16) & 0xff;
	buf[13] = (m_mtu >> 8) & 0xff;
	buf[14] = (m_mtu >> 0) & 0xff;

	return 15;
}

Ptr<RlocMetrics> RlocMetrics::DeserializedInMapReplyRecord(const uint8_t *buf){
	NS_LOG_FUNCTION_NOARGS();
	uint8_t priority, weight, mpriority, mweight;
	priority = buf[0];
	weight = buf[1];
	mpriority = buf[2];
	mweight = buf[3];

	Ptr<RlocMetrics> rlocMetrics = Create<RlocMetrics>(priority, mpriority, weight,
			mweight);

	// set for flag L
	if (buf[5] & RLOCF_L) {
		//Yue: I'm not sure if I well understand member attribute: isLocalIf.
		//I think it is identical to flag L in Map-Reply message
		//Thus, we set these two attributes at the same time.
		rlocMetrics->SetIsLocalIf(true);
		rlocMetrics->SetFlagL(true);
	} else {
		rlocMetrics->SetIsLocalIf(false);
		rlocMetrics->SetFlagL(false);
	}

	/* set flag p, Lionel has not taken into acount this flag.
	 * According to RF6830
	 *  p: When this bit is set, an ETR informs the RLOC-Probing ITR that the
	 locator address for which this bit is set is the one being
	 RLOC-probed and MAY be different from the source address of the
	 Map-Reply.  An ITR that RLOC-probes a particular Locator MUST use
	 this Locator for retrieving the data structure used to store the
	 fact that the Locator is reachable.  The p-bit is set for a single
	 Locator in the same Locator-Set.  If an implementation sets more
	 than one p-bit erroneously, the receiver of the Map-Reply MUST
	 select the first Locator.  The p-bit MUST NOT be set for
	 Locator-Set records sent in Map-Request and Map-Register messages.
	 * */

	if (buf[5] & RLOCF_p) {
		rlocMetrics->SetFlagP(true);
	} else {
		rlocMetrics->SetFlagP(false);
	}

	/*Set flag R
	 *    R: This is set when the sender of a Map-Reply has a route to the
	 Locator in the Locator data record.  This receiver may find this
	 useful to know if the Locator is up but not necessarily reachable
	 from the receiver's point of view.  See also Section 6.4 for
	 another way the R-bit may be used.

	 I'm not sure if flag R represents status up/down
	 * */
	if (buf[5] & RLOCF_R){
		rlocMetrics->SetUp(true);
		rlocMetrics->SetFlagR(true);
	}
	else{
		rlocMetrics->SetUp(false);
		rlocMetrics->SetFlagR(false);
	}

	uint16_t locAfi=0;
	locAfi |= buf[6];
	locAfi <<= 8;
	locAfi |= buf[7];

	if(locAfi == static_cast<uint16_t>(IPv4)){
		rlocMetrics->SetLocAfi(IPv4);
	}
	else if (locAfi == static_cast<uint16_t>(IPv6)){
		rlocMetrics->SetLocAfi(IPv6);
	}
	else{
		NS_LOG_DEBUG("Undefined AFI!!!");
	}
	return rlocMetrics;
}

Ptr<RlocMetrics> RlocMetrics::Deserialized(const uint8_t *buf) {
	Ptr<RlocMetrics> rlocMetrics = Create<RlocMetrics>();
	rlocMetrics->SetPriority(buf[0]);
	rlocMetrics->SetWeight(buf[1]);

	uint8_t flags = buf[2];
	if (flags & RLOCF_RXNONCE) {
		rlocMetrics->SetRxNoncePresent(true);
	}
	if (flags & RLOC_TXNONCE) {
		rlocMetrics->SetTxNoncePresent(true);
	}
	if (flags & RLOCF_LIF) {
		rlocMetrics->SetIsLocalIf(true);
	}
	if (flags & RLOCF_UP) {
		rlocMetrics->SetUp(true);
	} else
		rlocMetrics->SetUp(false);

	uint32_t rxNonce = 0;
	rxNonce |= buf[3];
	rxNonce <<= 8;
	rxNonce |= buf[4];
	rxNonce <<= 8;
	rxNonce |= buf[5];
	rxNonce <<= 8;
	rxNonce |= buf[6];

	rlocMetrics->SetRxNonce(rxNonce);

	uint32_t txNonce = 0;
	txNonce |= buf[7];
	txNonce <<= 8;
	txNonce |= buf[8];
	txNonce <<= 8;
	txNonce |= buf[9];
	txNonce <<= 8;
	txNonce |= buf[10];

	rlocMetrics->SetTxNonce(txNonce);

	uint32_t mtu = 0;
	mtu |= buf[11];
	mtu <<= 8;
	mtu |= buf[12];
	mtu <<= 8;
	mtu |= buf[13];
	mtu <<= 8;
	mtu |= buf[14];

	rlocMetrics->SetMtu(mtu);

	return rlocMetrics;
}

} /* namespace ns3 */
