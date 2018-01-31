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
#include "mapping-socket-msg.h"
#include "ns3/ipv4-address.h"
#include "ns3/ipv6-address.h"
#include "lisp-protocol.h"
#include "map-tables.h"
#include "simple-map-tables.h"
#include "ns3/log.h"

namespace ns3 {
NS_LOG_COMPONENT_DEFINE("MappingSocketMsg");

NS_OBJECT_ENSURE_REGISTERED(MappingSocketMsg);

TypeId MappingSocketMsg::GetTypeId(void) {
	static TypeId tid =
			TypeId("ns3::MappingSocketMsg").SetGroupName("Lisp").AddConstructor<
					MappingSocketMsgHeader>();
	return tid;
}

MappingSocketMsg::MappingSocketMsg() {

}

MappingSocketMsg::MappingSocketMsg(Ptr<EndpointId> endPoint,
		Ptr<Locators> locatorsList) {
	m_endPoint = endPoint;
	m_locatorsList = locatorsList;
}

MappingSocketMsg::~MappingSocketMsg() {

}

void MappingSocketMsg::Print(std::ostream& os) {
//	  std::string str_map_soc_msg = "EID prefix: ";
//	  Address m_eidAddress = m_endPoint->GetEidAddress();
//	  if (Ipv4Address::IsMatchingType(m_eidAddress))
//	    {
//	      std::stringstream str;
//	      Ipv4Address::ConvertFrom (m_eidAddress).Print(str);
//	      str_map_soc_msg += str.str ();
//	      str.str (std::string ());
//	      m_eidAddress:Print(str);
//	      str_map_soc_msg +=  str.str ();
//	    }
//	  else
//	    {
//	      std::stringstream str;
//	      Ipv6Address::ConvertFrom (m_eidAddress).Print(str);
//	      str_map_soc_msg += str.str () + "\n";
//	      str.str (std::string ());
//	      m_prefix.Print (str);
//	      str_map_soc_msg += "ipv6 prefix: " + str.str () + "\n\n";
//	    }
//
//	  return str_map_soc_msg;
}

void MappingSocketMsg::SetEndPoint(Ptr<EndpointId> endPoint) {
	m_endPoint = endPoint;
}

Ptr<EndpointId> MappingSocketMsg::GetEndPointId(void) {
	return m_endPoint;
}

void MappingSocketMsg::SetLocators(Ptr<Locators> locatorsList) {
	m_locatorsList = locatorsList;
}

Ptr<Locators> MappingSocketMsg::GetLocators(void) {
	return m_locatorsList;
}

void MappingSocketMsg::Serialize(uint8_t *buf) {
	uint8_t size = m_endPoint->Serialize(buf + 1);
	buf[0] = size;
	if (m_locatorsList) {
		buf[size + 1] = 1;
		m_locatorsList->Serialize(buf + size + 2);
	} else
		buf[size + 1] = 0;
}

Ptr<MappingSocketMsg> MappingSocketMsg::Deserialize(uint8_t *buf) {
	//When developing this method, you have to know *buf is populated by
	//which function
	uint8_t size = buf[0];
	Ptr<MappingSocketMsg> mapSockMsg = Create<MappingSocketMsg>();
	mapSockMsg->SetEndPoint(EndpointId::Deserialize(buf + 1));
	if (buf[size + 1])
		mapSockMsg->SetLocators(LocatorsImpl::Deserialize(buf + size + 2));
	return mapSockMsg;
}

} /* namespace ns3 */
