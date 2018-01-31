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
 * 	 : Qipeng Song <qpsong@gmail.com>
 * 	 : Yue Li <selene.cnfr@gmail.com>
 */

#include "lisp-etr-itr-application.h"
#include "ns3/log.h"
#include "ns3/address-utils.h"
#include "ns3/nstime.h"
#include "ns3/socket.h"
#include "ns3/simulator.h"
#include "ns3/socket-factory.h"
#include "ns3/packet.h"
#include "ns3/uinteger.h"
#include "ns3/random-variable-stream.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/map-register-msg.h"
#include "ns3/map-reply-msg.h"
#include "map-resolver.h"
#include "ns3/lisp-mapping-socket.h"
#include "ns3/mapping-socket-msg.h"
#include "ns3/ipv4-interface-address.h"
#include "ns3/ipv4.h"
#include "ns3/ipv6.h"
#include <climits>
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/ipv6-routing-protocol.h"
#include "ns3/ipv4-route.h"
#include "ns3/ipv6-route.h"
#include "ns3/endpoint-id.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("LispEtrItrApplication");

NS_OBJECT_ENSURE_REGISTERED(LispEtrItrApplication);

const uint8_t LispEtrItrApplication::MAX_REQUEST_NB = 3;

TypeId LispEtrItrApplication::GetTypeId(void) {
	static TypeId tid = TypeId("ns3::LispEtrItrApplication").SetParent<
			Application>().SetGroupName("Lisp").AddConstructor<
			LispEtrItrApplication>().AddAttribute("PeerPort",
			"The destination port of the packet",
			UintegerValue(LispOverIp::LISP_SIG_PORT),
			MakeUintegerAccessor(&LispEtrItrApplication::m_peerPort),
			MakeUintegerChecker<uint16_t>()).AddAttribute("Interval",
			"The time to wait between packets", TimeValue(Seconds(3.0)),
			MakeTimeAccessor(&LispEtrItrApplication::m_interval),
			MakeTimeChecker());
	return tid;
}

LispEtrItrApplication::LispEtrItrApplication() {
	NS_LOG_FUNCTION(this);
	NS_LOG_DEBUG("Constructor of LispEtrItrApplication is called!");
	m_lispMappingSocket = 0;
	m_event = EventId();
	m_sent = 0;
	m_requestSent = 0;
	m_lispProtoAddress = Address(); // invalid address
	m_recvIvkSmr = false;
}

LispEtrItrApplication::~LispEtrItrApplication() {

}

void LispEtrItrApplication::SetMapServerAddresses(
		std::list<Address> mapServerAddresses) {
	m_mapServerAddress = mapServerAddresses;
}

void LispEtrItrApplication::SetMapTables(Ptr<MapTables> mapTablesV4,
		Ptr<MapTables> mapTablesV6) {
	m_mapTablesV4 = mapTablesV4;
	m_mapTablesV6 = mapTablesV6;
}

void LispEtrItrApplication::AddMapResolverLoc(Ptr<Locator> locator) {
	m_mapResolverRlocs.push_back(locator);
}

void LispEtrItrApplication::DoDispose(void) {
	Application::DoDispose();
}

void LispEtrItrApplication::StartApplication(void) {
	if (m_lispMappingSocket == 0) {
		NS_LOG_DEBUG(
				"Trying to create a lisp mapping socket (between lisp and xTR) which connects to lispProto");
		TypeId tid = TypeId::LookupByName("ns3::LispMappingSocketFactory");
		m_lispMappingSocket = Socket::CreateSocket(GetNode(), tid);
		Ptr<LispOverIp> lisp = m_lispMappingSocket->GetNode()->GetObject<
				LispOverIp>();
		m_lispProtoAddress = lisp->GetLispMapSockAddress();
		m_lispMappingSocket->Bind();
		m_lispMappingSocket->Connect(m_lispProtoAddress);
		//m_lispProtoAddress is a special kind of address defined for comm between lisp and xTR
		NS_LOG_DEBUG(
				"LispEtrItrApplication has connected to " << m_lispProtoAddress);
	}
	m_lispMappingSocket->SetRecvCallback(
			MakeCallback(&LispEtrItrApplication::HandleMapSockRead, this));

	if (m_socket == 0) {
		NS_LOG_DEBUG(
				"Trying to create a UDP socket which connects to Map Server");
		TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
		m_socket = Socket::CreateSocket(GetNode(), tid);
		if (Ipv4Address::IsMatchingType(m_mapServerAddress.front())) {
			m_socket->Bind();
			m_socket->Connect(
					InetSocketAddress(
							Ipv4Address::ConvertFrom(
									m_mapServerAddress.front()), m_peerPort));
		} else if (Ipv6Address::IsMatchingType(m_mapServerAddress.front())) {
			m_socket->Bind6();
			m_socket->Connect(
					Inet6SocketAddress(
							Ipv6Address::ConvertFrom(
									m_mapServerAddress.front()), m_peerPort));
		}
	}
	// Qipeng: Useful to send map register message...
	m_socket->SetRecvCallback(
			MakeCallback(&LispEtrItrApplication::HandleRead, this));

	// Qipeng: bind to local address in order to receive lisp control plan message: map request/reply, etc.
	// However, it seems that xTR never use the following two sockets. See whether we should remove them.
	if (m_lispCtlMsgRcvSocket == 0) {
		TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
		m_lispCtlMsgRcvSocket = Socket::CreateSocket(GetNode(), tid);
		InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(),
				LispOverIp::LISP_SIG_PORT);
		m_lispCtlMsgRcvSocket->Bind(local);
	}

	if (m_lispCtlMsgRcvSocket6 == 0) {
		TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
		m_lispCtlMsgRcvSocket6 = Socket::CreateSocket(GetNode(), tid);
		Inet6SocketAddress local6 = Inet6SocketAddress(Ipv6Address::GetAny(),
				LispOverIp::LISP_SIG_PORT);
		m_lispCtlMsgRcvSocket6->Bind(local6);
	}
	m_lispCtlMsgRcvSocket6->SetRecvCallback(
			MakeCallback(&LispEtrItrApplication::HandleReadControlMsg, this));
	m_lispCtlMsgRcvSocket->SetRecvCallback(
			MakeCallback(&LispEtrItrApplication::HandleReadControlMsg, this));
	ScheduleTransmit(Seconds(0.));
	NS_LOG_DEBUG("Lisp xTR Application Starts");
}

void LispEtrItrApplication::StopApplication(void) {
	NS_LOG_FUNCTION(this);

	if (m_lispMappingSocket != 0) {
		m_lispMappingSocket->Close();
		m_lispMappingSocket->SetRecvCallback(
				MakeNullCallback<void, Ptr<Socket> >());
		m_lispMappingSocket = 0;
	}

	if (m_socket != 0) {
		m_socket->Close();
		m_socket->SetRecvCallback(MakeNullCallback<void, Ptr<Socket> >());
		m_socket = 0;
	}

	if (m_lispCtlMsgRcvSocket) {
		m_lispCtlMsgRcvSocket->Close();
		m_lispCtlMsgRcvSocket->SetRecvCallback(
				MakeNullCallback<void, Ptr<Socket> >());
		m_lispCtlMsgRcvSocket = 0;
	}

	if (m_lispCtlMsgRcvSocket6) {
		m_lispCtlMsgRcvSocket6->Close();
		m_lispCtlMsgRcvSocket6->SetRecvCallback(
				MakeNullCallback<void, Ptr<Socket> >());
		m_lispCtlMsgRcvSocket6 = 0;
	}

	Simulator::Cancel(m_event);
}

void LispEtrItrApplication::ScheduleTransmit(Time dt) {
	m_event = Simulator::Schedule(dt, &LispEtrItrApplication::SendMapRegisters,
			this);
}

void LispEtrItrApplication::SendMapRegisters(void) {
	NS_LOG_FUNCTION(this);
	NS_ASSERT(m_event.IsExpired());
	std::list<Ptr<MapEntry> > mapEntries;
	// It's better to do this check earlier, e.g., move this check in StartApplication method...
	// For Yue to implement this
	if (m_mapTablesV4->GetNMapEntriesLispDataBase() == 0
			and m_mapTablesV6->GetNMapEntriesLispDataBase() == 0) {
		NS_LOG_WARN(
				"Map Register sending is terminated due to empty LISP database...");
		return;
	}
	m_mapTablesV4->GetMapEntryList(MapTables::IN_DATABASE, mapEntries);
	m_mapTablesV6->GetMapEntryList(MapTables::IN_DATABASE, mapEntries);
	// Iterate mapEntries to construct Map-Register message.
	for (std::list<Ptr<MapEntry> >::const_iterator it = mapEntries.begin();
			it != mapEntries.end(); ++it) {
		// variable it here is a pointer of pointer => *it is actually another smart pointer.
		Ptr<MapRegisterMsg> msg = LispEtrItrApplication::GenerateMapRegister(
				*it);
		/**
		 * Calculating the bytes held by Map-Register Message...
		 * BUF_SIZE = 16+AuthDataLen+16+12*LocatorCount
		 */
		uint8_t BUF_SIZE = 16 + msg->GetAuthDataLen() + 16
				+ 12 * msg->GetRecord()->GetLocatorCount();
		uint8_t *buf = new uint8_t[BUF_SIZE];
		//std::memset(buf, 0, sizeof(buf));
		msg->Serialize(buf);
		MapResolver::ConnectToPeerAddress(m_mapServerAddress.front(),
				LispOverIp::LISP_SIG_PORT, m_socket);
		Ptr<Packet> p = Create<Packet>(buf, BUF_SIZE);
		m_socket->Send(p);
		NS_LOG_DEBUG(
				"Map-Register message sent to "<<Ipv4Address::ConvertFrom(m_mapServerAddress.front()));
	}

	++m_sent;

	/*if (m_sent < m_count)
	 {
	 ScheduleTransmit (m_interval);
	 }*/
}

void LispEtrItrApplication::SendToLisp(Ptr<Packet> packet) {
	NS_LOG_FUNCTION(this);
	m_lispMappingSocket->Send(packet);

	/*if (m_sent < m_count)
	 {
	 ScheduleTransmit (m_interval);
	 }*/
	NS_LOG_DEBUG(
			"LispEtrItrApplication sent a packet to lispOverIp by lispMappingSocket! \nThe packet is: "<<*packet);
}

void LispEtrItrApplication::HandleReadControlMsg(Ptr<Socket> socket) {
	NS_LOG_FUNCTION(this);
	Ptr<Packet> packet;
	Address from;
	while ((packet = socket->RecvFrom(from))) {
		uint8_t buf[packet->GetSize()];
		packet->CopyData(buf, packet->GetSize());
		// Do not forget to right shift first byte to obtain message type..
		uint8_t msg_type = buf[0] >> 4;
		if (msg_type == MapRequestMsg::GetMsgType()) {
			Ptr<MapRequestMsg> requestMsg = MapRequestMsg::Deserialize(buf);
			Ptr<Packet> reactedPacket;
			/**
			 * After reception of a map request, the possible reactions:
			 * 1) A conventional map reply
			 * 2) An invoked-SMR
			 * 3) A map reply to an invoked-SMR
			 * No matter to react to a SMR or normal map request message, the destination RLOC is the same.
			 * Update: 2018-01-24, In case of reception of a SMR, perhaps it is better to send an
			 * invoked-SMR to map resolver/map server. Because in case of double encapsulation issue in
			 * LISP-MN, the first invoked-SMR will be surely failed.
			 */
			if (requestMsg->GetItrRlocAddrIp()
					!= static_cast<Address>(Ipv4Address()))
				MapResolver::ConnectToPeerAddress(
						requestMsg->GetItrRlocAddrIp(), m_peerPort, m_socket);
			else if ((requestMsg->GetItrRlocAddrIpv6()
					!= static_cast<Address>(Ipv6Address())))
				MapResolver::ConnectToPeerAddress(
						requestMsg->GetItrRlocAddrIpv6(), m_peerPort, m_socket);
			else
				NS_LOG_ERROR(
						"NO valid ITR address (neither ipv4 nor ipv6) to send back Map Message!!!");

			if (requestMsg->GetS() == 0) {
				uint8_t newBuf[256];
				NS_LOG_DEBUG(
						"Receive one Map-Request on ETR from " << Ipv4Address::ConvertFrom (requestMsg->GetItrRlocAddrIp ())<<". Prepare a Map Reply Message.");
				Ptr<MapReplyMsg> mapReply =
						LispEtrItrApplication::GenerateMapReply(requestMsg);
				mapReply->Serialize(newBuf);
				reactedPacket = Create<Packet>(newBuf, 256);
				Send(reactedPacket);
				//TODO: we should add check for the return value of Send method.
				// Since it is possible that map reply has not been sent due to cache miss...
				NS_LOG_DEBUG(
						"Map Reply Sent to " << Ipv4Address::ConvertFrom (requestMsg->GetItrRlocAddrIp()));
			} else if (requestMsg->GetS() == 1 and requestMsg->GetS2() == 0) {
				/**
				 * case: reception of SMR and not a SMR-invoked Map Request.
				 * In case of double encapsulation, it is possible that first transmission
				 * of invoked SMR after reception of SMR is lost, because the destination
				 * address of invoked-SMR is the LISP-MN, which cannot be found
				 * in the cache of the considered xTR!
				 * Thus, it is necessary to first check whether the destination @IP
				 * is a known RLOC by xTR. If not, go to send SMR procedure
				 * for LISP-MN's new Local RLOC
				 *
				 */
				Address rlocAddr = requestMsg->GetItrRlocAddrIp ();
				NS_LOG_DEBUG(
						"Receive a SMR on ETR from " << Ipv4Address::ConvertFrom (rlocAddr)<<". Prepare an invoked Map Request Message.");
				if(m_node->GetObject<LispOverIpv4>()->IsLocatorInList (rlocAddr)){
					// If RLOC is in the list of lispOverIpv4 object => RLOC is really a RLOC of some xTR
					// Then we can send invoked-SMR to the xTR
					SendInvokedSmrMsg(requestMsg);
				}
				else{
					NS_LOG_DEBUG(
					"The destination IP address: "
					<<Ipv4Address::ConvertFrom (rlocAddr)
					<<" is not a known RLOC! In LISP-MN, it maybe an EID...Send Map request to query it"
					);
					m_mapReqMsg.push_back(requestMsg);
					Ptr<EndpointId> eid = Create<EndpointId>(rlocAddr);
					// Generate a map request and send it...
					SendMapRequest(GenerateMapRequest(eid));
					//TODO:2018-01-26:
					// Saved the invoked-SMR message.
					//	Remember to add something in map reply process part.
					// The saved invoked-SMR should be sent upon reception of map reply...
				}
			} else if (requestMsg->GetS() == 1 and requestMsg->GetS2() == 1) {
				/**
				 * case: reception of a SMR-invoked Map Request.
				 * Unlike a normal map reply, we don't care the EID-prefix conveyed in this message.
				 * We will put the changed mapping in map-reply
				 */
				/**
				 * TODO: how to know which mapping entry is changed in database?
				 * For LISP-MN, it is simple, since its database just has one mapping entry.
				 * This a normal Map Rely...
				 */
				NS_LOG_DEBUG(
						"Receive an SMR-invoked Request on ETR from " << Ipv4Address::ConvertFrom (requestMsg->GetItrRlocAddrIp ())<<". Prepare a Map Reply Message.");
				m_recvIvkSmr = true;
				// Given reception of SMR-invoked map request, remove the scheduled event.
				Simulator::Remove(m_resendSmrEvent);
				uint8_t newBuf[256];
				// Instead of response the queried EID-prefix, maReply conveys the content of database!
				Ptr<MapReplyMsg> mapReply =
						LispEtrItrApplication::GenerateMapReply4ChangedMapping(
								requestMsg);
				mapReply->Serialize(newBuf);
				reactedPacket = Create<Packet>(newBuf, 256);
				/**
				 * It's better to send map reply in response to invoked SMR to MR/MS according to
				 * RFC6830. Up to Yue to implement it.
				 */
				Send(reactedPacket);
				NS_LOG_DEBUG(
						"A Map Reply Message Sent to " << Ipv4Address::ConvertFrom (requestMsg->GetItrRlocAddrIp())<<" in response to an invoked SMR");
			} else {
				NS_LOG_ERROR(
						"Unknown M bit value. Either 0 or 1. No other values!");
			}
		} else if (msg_type
				== static_cast<uint8_t>(MapReplyMsg::GetMsgType())) {
			NS_LOG_DEBUG(
					"Msg Type "<< unsigned (msg_type)<<": GET a MAP REPLY");
			// Get Map Reply
			Ptr<MapReplyMsg> replyMsg = MapReplyMsg::Deserialize(buf);
			// prepare mapping socket message body+header
			Ptr<MappingSocketMsg> mapSockMsg = GenerateMapSocketAddMsgBody(
					replyMsg);
			MappingSocketMsgHeader mapSockHeader =
					GenerateMapSocketAddMsgHeader(replyMsg);
			NS_ASSERT_MSG(mapSockMsg != 0,
					"Cannot create map socket message body !!! Please check why.");
			uint8_t buf[256];
			mapSockMsg->Serialize(buf);
			/**
			 * In my opinion, it is not necessary to create a packet with 256 bytes as payload.
			 * In fact, we just need to ensure that length of buffer is more than the length of mapSockMsg.
			 * Since this is just an exchange between LispOverIp(data plan) and LispEtrItrApplication(control plan)
			 * Both data plan and control plan are on the same node.
			 * It is like a socket communication between user and kernel space.
			 */
			Ptr<Packet> packet = Create<Packet>(buf, 256);
			packet->AddHeader(mapSockHeader);
			// Send to lispOverIp object so that it can insert the mapping entry in Cache.
			// ATTENTION: with SMR, before inserting one map entry, should first check its presence in Cache.
			// Now we apply a replacement strategy: if the EID-prefix already in Cache, replace it with the new
			// One.
			SendToLisp(packet);
			// Don't forget to remove Eid in pending list...
			DeleteFromMapReqList(mapSockMsg->GetEndPointId());
			/**
			 * After reception of map reply and insertion of received EID-RLOC mapping into cache,
			 * remember to check if the map request messages with received EID are present in m_mapReqMsg. If yes,
			 * remember to send them
			 * It is better to move this method to cache database.
			 */
			for(std::list<Ptr<MapRequestMsg>>::const_iterator it = m_mapReqMsg.begin(); it!=m_mapReqMsg.end();++it){
				//
			}

		} else if (msg_type
				== static_cast<uint8_t>(LispControlMsg::MAP_NOTIFY)) {
			/**
			 * After reception of Map notify message, two possible reactions:
			 * 1) In traditional LISP, no reaction after Map registration procedure
			 * 2) In LISP-MN mode, if supporting SMR, a SMR message should be sent out!
			 * As a first step, we choose to send map request to all RLOCs in the cache.
			 * Maybe it's better to have a flag for mapTablesChanged. Only it is true,
			 * SMR procedure will be triggered. In addition, we also need a mechanism to
			 * quickly check whether database is changed (Map versioning???)
			 */
			if (m_mapTablesV4->GetNMapEntriesLispCache() != 0
					or m_mapTablesV6->GetNMapEntriesLispCache() != 0) {
				NS_LOG_DEBUG(
						"Receive a map notify message. xTR's Cache is not empty. Trigger SMR procedure for every entry in Cache...");
				LispEtrItrApplication::SendSmrMsg();
				/**
				 * After sendng SMR, don't forget to schedule a resend event for SMR
				 * Since for double encapsulation case, surely the first trial will be failed.
				 */
				m_resendSmrEvent = Simulator::Schedule(Seconds(2.0),
						&LispEtrItrApplication::SendSmrMsg, this);
				m_recvIvkSmr = false;
			}
		} else
			NS_LOG_ERROR("Problem with packet!");
	}
}
//TODO: In RFC6830, xTR will send SMR to all xTRs contacted in the last minutes.
// As a first step, we send SMR to all xTRs in the cache databases.
void LispEtrItrApplication::SendSmrMsg() {
	// First extract all map entries in m_mapTablesV4 and m_mapTablesV6
	NS_LOG_FUNCTION(this);
	std::list<Ptr<MapEntry> > mapEntries;
	m_mapTablesV4->GetMapEntryList(MapTables::IN_CACHE, mapEntries);
	m_mapTablesV6->GetMapEntryList(MapTables::IN_CACHE, mapEntries);
	Ptr<EndpointId> tmpEid;
	Address dstRlocAddr;
	for (std::list<Ptr<MapEntry> >::const_iterator it = mapEntries.begin();
			it != mapEntries.end(); ++it) {
		tmpEid = (*it)->GetEidPrefix();
		/**
		 * Send map request directly to the remote contacted xTR.
		 * Since 1) the RLOC address (maybe more than one, we choose the first valid one) is alreay in cache
		 * 2) send to MR/MS so that the latter forward map request message costs much time.
		 * TODO: RFC6830 requires to send a SMR to each Locator... see 6.6.2
		 */
		dstRlocAddr =
				(*it)->GetLocators()->SelectFirsValidRloc()->GetRlocAddress();
		Ptr<MapRequestMsg> mapReqMsg =
				LispEtrItrApplication::GenerateMapRequest(tmpEid);
		//IMPORTANT: set SMR bit!!!
		mapReqMsg->SetS(1);
		uint8_t bufMapReq[64];
		mapReqMsg->Serialize(bufMapReq);
		Ptr<Packet> packetSmrMsg;
		packetSmrMsg = Create<Packet>(bufMapReq, 64);
		// TODO: Should add SMR rate-limiting method...just like what we do for map request
		/**
		 * In fact, the name of MapResolver::ConnectToPeerAddress is somewhat misleading. It has nothing to do
		 * with MapResolver. Just because MapRsolver has encapsulated socket bind and connect manipulation
		 * into an independent function. We call it to avoid reinventing the wheel.
		 * Thus, it is necessary to do socket bind and connect again for map-register and map-reply manipulation.
		 */
		MapResolver::ConnectToPeerAddress(dstRlocAddr,
				LispOverIp::LISP_SIG_PORT, m_socket);
		// Before sending map message, first should make sure the m_socket connect to the correct @IP and port number!
		Send(packetSmrMsg);
		NS_LOG_DEBUG("A SMR message has been sent to xTR "<<dstRlocAddr);
	}

}

void LispEtrItrApplication::SendInvokedSmrMsg(Ptr<MapRequestMsg> smr) {
	Ptr<Packet> reactedPacket;
	/**
	 * Determine the dst@IP of this sending: the RLOC of xTR sending SMR
	 * This @IP is in the received SMR
	 */
	if (smr->GetItrRlocAddrIp() != static_cast<Address>(Ipv4Address())){
		MapResolver::ConnectToPeerAddress(smr->GetItrRlocAddrIp(), m_peerPort,
				m_socket);
		NS_LOG_DEBUG("Now the socket has been binded to:"<<Ipv4Address::ConvertFrom(smr->GetItrRlocAddrIp()));
	}

	else if ((smr->GetItrRlocAddrIpv6() != static_cast<Address>(Ipv6Address())))
		MapResolver::ConnectToPeerAddress(smr->GetItrRlocAddrIpv6(), m_peerPort,
				m_socket);
	else
		NS_LOG_ERROR(
				"NO valid ITR address (neither ipv4 nor ipv6) to send back Map Message!!!");

	uint8_t newBuf[256];
	//TODO: verify if we can directly copy map request message as SMR-invoked map request
	// We just need to change SMR-invoked bit as 1 and change the source ITR's address
	//TODO: now I'm lost about defintion of m_mapResolverRlocs...
	smr->SetS2(1);
	Address itrAddress = GetLocalAddress(
			m_mapResolverRlocs.front()->GetRlocAddress());
	if (Ipv4Address::IsMatchingType(itrAddress)) {
		smr->SetItrRlocAddrIp(itrAddress);
		NS_LOG_DEBUG("Invoked-SMR's source RLOC field has been set as:"<<Ipv4Address::ConvertFrom(itrAddress));
	} else {
		smr->SetItrRlocAddrIpv6(itrAddress);
	}
	// TODO:Actually, we also choose a new nonce number. (RFC6830)
	smr->Serialize(newBuf);
	reactedPacket = Create<Packet>(newBuf, 256);
	// IMPORTANT: set invoked-SMR bit as 1.
	// TODO: Question: what about bit S? 1 or 0 ? or whatever?

	/**
	 * Update: 2018-01-24
	 * In the past, the invoked-SMR is sent directly the xTR that initiates the received SMR
	 * Now we send the invoked-SMR to map server/map resolver
	 * Update: 2018-01-26
	 * Sending an invoked-SMR to map server or xTR initiating SMR has
	 * impact to handover performance...
	 */

	Send(reactedPacket);
	NS_LOG_DEBUG(
			"Invoked Map Request Message Sent to " << Ipv4Address::ConvertFrom (itrAddress));

}


void LispEtrItrApplication::SendMapRequest(Ptr<MapRequestMsg> mapReqMsg){
	//TODO: whehter here 64 bytes here is good...
	uint8_t bufMapReq[64];
	mapReqMsg->Serialize(bufMapReq);
	Ptr<Packet> packetMapReqMsg;
	packetMapReqMsg = Create<Packet>(bufMapReq, 64);
	MapResolver::ConnectToPeerAddress(
			m_mapResolverRlocs.front()->GetRlocAddress(),
			LispOverIp::LISP_SIG_PORT, m_socket);
	Send(packetMapReqMsg);
}

void LispEtrItrApplication::HandleMapSockRead(Ptr<Socket> lispMappingSocket) {
	NS_LOG_FUNCTION(this);
	Ptr<Packet> packet;
	Address from;

	while ((packet = lispMappingSocket->RecvFrom(from))) {
		NS_LOG_DEBUG(
				"Receive something from LispOverIp (data plan) through lispMappingSocket:)");
		MappingSocketMsgHeader sockMsgHdr;
		packet->RemoveHeader(sockMsgHdr);
		uint8_t buf[packet->GetSize()];
		packet->CopyData(buf, packet->GetSize());
		Ptr<MappingSocketMsg> msg = MappingSocketMsg::Deserialize(buf);
		//Show LISP control plan message header
		NS_LOG_DEBUG("MSG HEADER: " << sockMsgHdr);
		if (sockMsgHdr.GetMapType()
				== static_cast<uint16_t>(LispMappingSocket::MAPM_MISS)) {
			// Means that: in kernel space, CacheLookup has been tried, but find nothing... => MAPM_MISS
			// To support LISP-MN, we allows map request for each EID can be up to 3 times
			Ptr<EndpointId> eid = msg->GetEndPointId();
			uint8_t currRqstNb = 0;
			if (IsInRequestList(eid))
				currRqstNb = GetRequestCount(eid);
			if (not IsInRequestList(eid)
					or currRqstNb != LispEtrItrApplication::MAX_REQUEST_NB) {
				NS_LOG_DEBUG(
						"Remote EID has been requested "<<unsigned(currRqstNb)<< " times. Start to generate map request message.");
				Ptr<MapRequestMsg> mapReqMsg =
						LispEtrItrApplication::GenerateMapRequest(eid);
				//I'm not sure what happens if one insert a key-value pair into map if the key is alreay existing.
				if (currRqstNb == 0)
					// AddInMapReqList will populate m_requestCounter and m_requestList
					AddInMapReqList(eid, mapReqMsg);
				else
					// Increment by 1
					m_requestCounter.find(eid)->second++;
				// why each time we want to send map request we bind and connect socket operations??
				SendMapRequest(mapReqMsg);
				NS_LOG_DEBUG(
						"Hence, A Mapping request has been sent in control plan to query for EID..."<<msg->GetEndPointId()->Print());
			} else {
				// if and only isInRequestList and count = max.allowed.nb
				NS_LOG_DEBUG(
						"Remote EID has been requested up to "<<unsigned(LispEtrItrApplication::MAX_REQUEST_NB)<<" times! Give up...");
				return;
			}

		} else if (sockMsgHdr.GetMapType()
				== static_cast<uint16_t>(LispMappingSocket::MAPM_REGISTER)) {
			LispEtrItrApplication::SendMapRegisters();
			NS_LOG_DEBUG(
					"Reception from Lisp data plan (LispOverIpv4): Lisp database base is updated. Send a new Map Register message.");
		}
	}
}

std::list<Ptr<MapRequestMsg>> LispEtrItrApplication::GetMapRequestMsgList(){

	return m_mapReqMsg;
}

MappingSocketMsgHeader LispEtrItrApplication::GenerateMapSocketAddMsgHeader(
		Ptr<MapReplyMsg> replyMsg) {
	MappingSocketMsgHeader mapSockHeader;
	Ptr<MapReplyRecord> replyRecord = replyMsg->GetRecord();
	mapSockHeader.SetMapType(LispMappingSocket::MAPM_ADD);
	mapSockHeader.SetMapVersioning(replyRecord->GetMapVersionNumber());
	if (replyRecord) {
		mapSockHeader.SetMapRlocCount(replyRecord->GetLocatorCount());
		if (!replyRecord->GetLocatorCount()) {
			// Negative map reply
			mapSockHeader.SetMapFlags(
					(int) mapSockHeader.GetMapFlags()
							| static_cast<int>(LispMappingSocket::MAPF_NEGATIVE));
			mapSockHeader.SetMapAddresses(
					(int) mapSockHeader.GetMapAddresses()
							| static_cast<int>(LispMappingSocket::MAPA_EIDMASK));
		}
		mapSockHeader.SetMapAddresses(
				(int) mapSockHeader.GetMapAddresses()
						| static_cast<int>(LispMappingSocket::MAPA_RLOC));
	}
	NS_LOG_DEBUG(
			"After receiving a Map Reply, " "ITR construct a Mapping-Socket-Control MSG with MSG HEADER: "<<mapSockHeader);
	return mapSockHeader;

}

Ptr<MappingSocketMsg> LispEtrItrApplication::GenerateMapSocketAddMsgBody(
		Ptr<MapReplyMsg> replyMsg) {
	Ptr<MappingSocketMsg> mapSockMsg = Create<MappingSocketMsg>();
	Ptr<MapReplyRecord> replyRecord = replyMsg->GetRecord();
	if (replyRecord) {
		mapSockMsg->SetLocators(replyRecord->GetLocators());
	} else
		NS_LOG_ERROR("There must always be a Record in the Map Reply message!");

	std::stringstream ss;
	ss << "/" << (int) replyRecord->GetEidMaskLength();
	Ptr<EndpointId> eid;
	if (replyRecord->GetEidAfi() == LispControlMsg::IP) {
		eid = Create<EndpointId>(replyRecord->GetEidPrefix(),
				Ipv4Mask(ss.str().c_str()));
		mapSockMsg->SetEndPoint(eid);
	} else {
		eid = Create<EndpointId>(replyRecord->GetEidPrefix(),
				Ipv6Prefix(ss.str().c_str()));
		mapSockMsg->SetEndPoint(eid);
	}
	//TODO: OK, I think, this message contains an entry which will be inserted
	// Cache Database...
	NS_LOG_DEBUG(
			"The Built Mapping-Socket-Control MSG's content: "<<mapSockMsg->GetLocators()->Print());
	return mapSockMsg;
}

Ptr<MapRegisterMsg> LispEtrItrApplication::GenerateMapRegister(
		Ptr<MapEntry> mapEntry) {
	Ptr<MapRegisterMsg> msg = Create<MapRegisterMsg>();
	Ptr<MapReplyRecord> record = Create<MapReplyRecord>();
	record->SetRecordTtl(static_cast<uint32_t>(0xffffffff));
	/**
	 * TODO: We consider M bit is set by default as 1
	 * so that Map Server will sends a Map Notify Message once upon reception
	 * of Map Register message.
	 */
	msg->SetM(1);
	msg->SetP(0);
	msg->SetNonce(0); // Nonce is 0 for map register
	msg->setKeyId(static_cast<uint16_t>(0xface));
	msg->SetAuthDataLen(04); // Set
	record->SetEidPrefix(mapEntry->GetEidPrefix()->GetEidAddress());
	if (record->GetEidAfi() == LispControlMsg::IP)
		record->SetEidMaskLength(
				mapEntry->GetEidPrefix()->GetIpv4Mask().GetPrefixLength());
	else if (record->GetEidAfi() == LispControlMsg::IPV6)
		record->SetEidMaskLength(
				mapEntry->GetEidPrefix()->GetIpv6Prefix().GetPrefixLength());
	// Get list of Locator and save it into record
	record->SetLocators(mapEntry->GetLocators());
	record->SetMapVersionNumber(mapEntry->GetVersionNumber());
	msg->SetRecord(record);
	msg->SetRecordCount(1);
	return msg;
}

Ptr<MapReplyMsg> LispEtrItrApplication::GenerateMapReply(
		Ptr<MapRequestMsg> requestMsg) {
	Ptr<MapReplyMsg> mapReply = Create<MapReplyMsg>(); // Smart pointer, default value is 0
	Ptr<MapRequestRecord> record = requestMsg->GetMapRequestRecord();
	Ptr<MapEntry> entry;
	if (record->GetAfi() == LispControlMsg::IP) {
		// TODO May be use mapping socket instead
		entry = m_mapTablesV4->DatabaseLookup(record->GetEidPrefix());
	} else if (record->GetAfi() == LispControlMsg::IPV6) {
		entry = m_mapTablesV6->DatabaseLookup(record->GetEidPrefix());
	}
	if (entry == 0) {
		//TODO: Should implement negative map-reply case.
		NS_LOG_DEBUG("Send Negative Map-Reply");
	} else {
		NS_LOG_DEBUG("Send Map-Reply to ITR");

		Ptr<MapReplyRecord> replyRecord = Create<MapReplyRecord>();

		mapReply->SetNonce(requestMsg->GetNonce());
		mapReply->SetRecordCount(1);
		replyRecord->SetAct(MapReplyRecord::NoAction);
		replyRecord->SetA(1);
		replyRecord->SetMapVersionNumber(entry->GetVersionNumber());
		replyRecord->SetRecordTtl(MapReplyRecord::m_defaultRecordTtl);
		replyRecord->SetEidPrefix(entry->GetEidPrefix()->GetEidAddress());

		if (entry->GetEidPrefix()->IsIpv4())
			replyRecord->SetEidMaskLength(
					entry->GetEidPrefix()->GetIpv4Mask().GetPrefixLength());
		else
			replyRecord->SetEidMaskLength(
					entry->GetEidPrefix()->GetIpv6Prefix().GetPrefixLength());

		replyRecord->SetLocators(entry->GetLocators());
		mapReply->SetRecord(replyRecord);
		NS_LOG_DEBUG(
				"MAP REPLY READY, Its content is as follows:\n"<<*mapReply);
	}
	return mapReply;
}

Ptr<MapReplyMsg> LispEtrItrApplication::GenerateMapReply4ChangedMapping(
		Ptr<MapRequestMsg> requestMsg) {
	/**
	 * Put all changed EID-RLOC mappings into map reply message, which will be
	 * returned to xTRs sending invoked-SMR.
	 * As a simple implementation, we simpy put all LISP database's mapping entries into
	 * map reply. Note that, one map-rely message can convey more than one map record!
	 */
	NS_LOG_FUNCTION(this);
	Ptr<MapReplyMsg> mapReply = Create<MapReplyMsg>(); // Smart pointer, default value is 0
	NS_ASSERT(m_event.IsExpired());
	std::list<Ptr<MapEntry> > mapEntries;
	NS_ASSERT_MSG(
			m_mapTablesV4->GetNMapEntriesLispDataBase()
					+ m_mapTablesV6->GetNMapEntriesLispDataBase() != 0,
			"Number of Lisp Database's map entries (ipv4 or ipv6) should not be 0!");
	m_mapTablesV4->GetMapEntryList(MapTables::IN_DATABASE, mapEntries);
	m_mapTablesV6->GetMapEntryList(MapTables::IN_DATABASE, mapEntries);
	Ptr<MapEntry> entry;
	/**
	 * For the case where database just has one map entry, the following code works well.
	 * However, we should implement a list of records. Question:
	 * It is possible that a map reply can convey more than one record?
	 * For example, a LISP-MN with ipv4 and ipv6 address at the same time.
	 * I think yes...
	 */
	for (std::list<Ptr<MapEntry> >::const_iterator it = mapEntries.begin();
			it != mapEntries.end(); ++it) {
		NS_LOG_DEBUG("Prepare Map-Reply in response to invoked SMR");
		entry = *it;
		Ptr<MapReplyRecord> replyRecord = Create<MapReplyRecord>();

		mapReply->SetNonce(requestMsg->GetNonce());
		mapReply->SetRecordCount(1);
		replyRecord->SetAct(MapReplyRecord::NoAction);
		replyRecord->SetA(1);
		replyRecord->SetMapVersionNumber(entry->GetVersionNumber());
		replyRecord->SetRecordTtl(MapReplyRecord::m_defaultRecordTtl);
		replyRecord->SetEidPrefix(entry->GetEidPrefix()->GetEidAddress());

		if (entry->GetEidPrefix()->IsIpv4())
			replyRecord->SetEidMaskLength(
					entry->GetEidPrefix()->GetIpv4Mask().GetPrefixLength());
		else
			replyRecord->SetEidMaskLength(
					entry->GetEidPrefix()->GetIpv6Prefix().GetPrefixLength());

		replyRecord->SetLocators(entry->GetLocators());
		mapReply->SetRecord(replyRecord);
		NS_LOG_DEBUG(
				"MAP REPLY READY, Its content is as follows:\n"<<*mapReply);
	}
	return mapReply;
}

Ptr<MapRequestMsg> LispEtrItrApplication::GenerateMapRequest(
		Ptr<EndpointId> eid) {
	Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable>();
	SeedManager::SetSeed(++m_seed % ULONG_MAX);
	// Build map request message, application layer meesage
	Ptr<MapRequestMsg> mapReqMsg = Create<MapRequestMsg>();
	Address itrAddress = GetLocalAddress(
			m_mapResolverRlocs.front()->GetRlocAddress());
	if (Ipv4Address::IsMatchingType(itrAddress)) {
		NS_LOG_DEBUG(
				"Lisp data plan cannot find mapping for " << eid->Print () << " on cache database of " << Ipv4Address::ConvertFrom (itrAddress));
		mapReqMsg->SetItrRlocAddrIp(itrAddress);
	} else {
		mapReqMsg->SetItrRlocAddrIpv6(itrAddress);
	}
	mapReqMsg->SetIrc(0);
	mapReqMsg->SetNonce(uv->GetInteger(0, UINT_MAX));
	Address eidAddress = eid->GetEidAddress();
	uint8_t maskLength = 0;
	if (Ipv4Address::IsMatchingType(eidAddress))
		maskLength = 32;
	else if (Ipv6Address::IsMatchingType(eidAddress))
		maskLength = 128;
	/**
	 * why Lionel set source EID address as an any Ipv4 address?
	 * Now I know, becase msg object just contains the dest EID (no source EID)!
	 * It is difficult to know which one is the source EID
	 */
	mapReqMsg->SetSourceEidAddr(static_cast<Address>(Ipv4Address()));
	mapReqMsg->SetSourceEidAddr(static_cast<Address>(Ipv4Address()));
	mapReqMsg->SetSourceEidAfi(LispControlMsg::IP);
	mapReqMsg->SetMapRequestRecord(
			Create<MapRequestRecord>(eidAddress, maskLength));
	return mapReqMsg;
}

void LispEtrItrApplication::Send(Ptr<Packet> packet) {
	NS_LOG_FUNCTION(this);

	NS_ASSERT(m_event.IsExpired());

	m_socket->Send(packet);
	++m_sent;
}

Address LispEtrItrApplication::GetLocalAddress(Address address) {

	/*Ptr<Ipv4> ipv4 = m_node->GetObject<Ipv4> ();
	 Ptr<NetDevice> device;;
	 if (ipv4)
	 {
	 device = ipv4->GetNetDevice (1);
	 Ipv4InterfaceAddress ifAddress = ipv4->GetAddress (device->GetIfIndex(), 0);
	 NS_LOG_DEBUG ("LOCAL ADDRESS --- " << ifAddress.GetLocal ());
	 return static_cast<Address> (ifAddress.GetLocal ());
	 }
	 else
	 {
	 Ptr<Ipv6> ipv6 = m_node->GetObject<Ipv6> ();
	 device = ipv6->GetNetDevice (1);
	 Ipv6InterfaceAddress ifAddress = ipv6->GetAddress (device->GetIfIndex(), 0);
	 return static_cast<Address> (ifAddress.GetAddress ());
	 }*/
	Socket::SocketErrno errno_;
	Address srcAddress;
	Ptr<Locator> srcLocator;
	// if the destination is Ipv4
	if (Ipv4Address::IsMatchingType(address)) {
		Ptr<Ipv4Route> route = 0;
		Ptr<Ipv4RoutingProtocol> routingProtocol =
				m_node->GetObject<Ipv4>()->GetRoutingProtocol();

		if (routingProtocol != 0) {
			Ipv4Header header = Ipv4Header();
			header.SetDestination(Ipv4Address::ConvertFrom(address));
			route = routingProtocol->RouteOutput(0, header, 0, errno_);
		} else {
			NS_LOG_ERROR("Map resolver unreachable: routingProtocol == 0");
		}

		if (route) {
			// we know the route and we get the output interface
			int32_t interface =
					m_node->GetObject<Ipv4>()->GetInterfaceForDevice(
							route->GetOutputDevice());
			// we get the interface address
			Ipv4Address ipv4SrcAddress = (m_node->GetObject<Ipv4>()->GetAddress(
					interface, 0)).GetLocal();
			srcAddress = static_cast<Address>(ipv4SrcAddress);
		} else {
			NS_LOG_WARN("No route to Map-Resolver. Return.");
			return static_cast<Address>(Ipv4Address());
		}
	} else if (Ipv6Address::IsMatchingType(address)) {
		Ptr<Ipv6Route> route = 0;
		Ptr<Ipv6RoutingProtocol> routingProtocol =
				m_node->GetObject<Ipv6>()->GetRoutingProtocol();
		if (routingProtocol) {
			Ipv6Header header = Ipv6Header();
			header.SetDestinationAddress(Ipv6Address::ConvertFrom(address));
			route = routingProtocol->RouteOutput(0, header, 0, errno_);
		} else {
			NS_LOG_ERROR("Map resolver unreachable: routingProtocol == 0");
		}

		if (route) {
			// we know the route and we get the output interface
			int32_t interface =
					m_node->GetObject<Ipv6>()->GetInterfaceForDevice(
							route->GetOutputDevice());
			// we get the inteface address
			Ipv6Address ipv6SrcAddress = (m_node->GetObject<Ipv6>()->GetAddress(
					interface, 0)).GetAddress();
			srcAddress = static_cast<Address>(ipv6SrcAddress);
		} else {
			NS_LOG_WARN("No route to Map-Resolver. Return.");
			return static_cast<Address>(Ipv6Address());
		}
	}

	return srcAddress; // should not happen
}

bool LispEtrItrApplication::IsInRequestList(Ptr<EndpointId> eid) const {
	if (m_requestList.find(eid) != m_requestList.end()) {
		return true;
	}
	return false;
}

bool LispEtrItrApplication::IsInRequestCounter(Ptr<EndpointId> eid) const {
	if (m_requestCounter.find(eid) != m_requestCounter.end()) {
		return true;
	}
	return false;
}

uint8_t LispEtrItrApplication::GetRequestCount(Ptr<EndpointId> eid) {
	return m_requestCounter.at(eid);
}

void LispEtrItrApplication::AddInMapReqList(Ptr<EndpointId> eid,
		Ptr<MapRequestMsg> reqMsg) {
	m_requestList.insert(
			std::pair<Ptr<EndpointId>, Ptr<MapRequestMsg> >(eid, reqMsg));
	m_requestCounter.insert(std::pair<Ptr<EndpointId>, uint8_t>(eid, 1));
}

void LispEtrItrApplication::DeleteFromMapReqList(Ptr<EndpointId> eid) {
	m_requestList.erase(eid);
	m_requestCounter.erase(eid);
	if (Ipv4Address::IsMatchingType(eid->GetEidAddress())) {
		NS_LOG_DEBUG(
				"LispEtrItrApplication removes EID prefix: "<<Ipv4Address::ConvertFrom(eid->GetEidAddress())<<" ("<<eid->GetIpv4Mask()<<") from PENDING LIST");
	} else if (Ipv6Address::IsMatchingType(eid->GetEidAddress())) {
		NS_LOG_DEBUG(
				"LispEtrItrApplication removes EID prefix: "<<Ipv6Address::ConvertFrom(eid->GetEidAddress())<<" ("<<eid->GetIpv4Mask()<<") from PENDING LIST");
	} else {
		NS_LOG_DEBUG(
				"LispEtrItrApplication removes unknown EID prefix: "<<eid->GetEidAddress()<<" from PENDING LIST");

	}

}
//TODO: Qipeng: I thinks this method is not useful...
void LispEtrItrApplication::HandleRead(Ptr<Socket> socket) {

}

} /* namespace ns3 */
