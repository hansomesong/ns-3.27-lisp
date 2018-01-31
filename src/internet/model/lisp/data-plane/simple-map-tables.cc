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

#include "ns3/simple-map-tables.h"

#include "ns3/ptr.h"
#include "ns3/ipv4.h"
#include "ns3/ipv4-header.h"
#include "ns3/ipv4-interface-address.h"
#include "ns3/ipv4-route.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/ipv6.h"
#include "ns3/ipv6-header.h"
#include "ns3/ipv6-interface-address.h"
#include "ns3/ipv6-route.h"
#include "ns3/ipv6-routing-protocol.h"
#include "ns3/address.h"
#include "ns3/node.h"
#include "ns3/packet.h"
#include "ns3/socket.h"
#include "ns3/ipv4-address.h"
#include "ns3/ipv6-address.h"
#include "ns3/lisp-header.h"
#include "ns3/assert.h"
#include "ns3/log.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("SimpleMapTables");

NS_OBJECT_ENSURE_REGISTERED(SimpleMapTables);

TypeId SimpleMapTables::GetTypeId() {
	static TypeId tid =
			TypeId("ns3::SimpleMapTables").SetParent<MapTables>().SetGroupName(
					"Lisp").AddConstructor<SimpleMapTables>();
	return tid;
}

SimpleMapTables::SimpleMapTables() {
	NS_LOG_FUNCTION(this);
}

SimpleMapTables::~SimpleMapTables() {
	NS_LOG_FUNCTION(this);
}

int
SimpleMapTables::GetNMapEntriesLispDataBase (void)
{
  return m_mappingDatabase.size();
}

int
SimpleMapTables::GetNMapEntriesLispCache (void)
{
  return m_mappingCache.size();
}

int SimpleMapTables::GetNMapEntries(void) {
  return m_mappingCache.size() + m_mappingDatabase.size();
}

std::ostream& operator<< (std::ostream &os, SimpleMapTables const &simpleMapTable)
{
  simpleMapTable.Print(os);
  return os;
}

void SimpleMapTables::Print (std::ostream &os) const
{
  //TODO: Currently, we just print the basic information in the Map Tables.
  //It can provide more detailed information.
  NS_LOG_FUNCTION(this);
  Ptr<EndpointId> tmp_eid;
  Ptr<MapEntry> tmp_mappingEntry;


  //The following typedef does not work:
  //to non-scalar type...
  //typedef std::map<Ptr<EndpointId>, Ptr<MapEntry>, CompareEndpointId>::iterator my_iterator;
  os<<"LISP Database:"<<std::endl;
  //TODO: Why in this method, we have to use "...::const_iterator" (otherwise compilation
  //crash), while in other method, we can juse "...::iterator"
  for (std::map<Ptr<EndpointId>, Ptr<MapEntry>, CompareEndpointId>::const_iterator it =
			m_mappingDatabase.begin(); it != m_mappingDatabase.end();
			++it)   {
      tmp_eid = it->first;
      os<<tmp_eid->Print()<<std::endl;
      tmp_mappingEntry = it->second;
      os<<tmp_mappingEntry->Print()<<std::endl;
  }
  os<<"LISP Cache:"<<std::endl;
  for (std::map<Ptr<EndpointId>, Ptr<MapEntry>, CompareEndpointId>::const_iterator it =
			m_mappingCache.begin(); it != m_mappingCache.end();
			++it)
  {
      tmp_eid = it->first;
      os<<tmp_eid->Print()<<std::endl;
      tmp_mappingEntry = it->second;
      os<<tmp_mappingEntry->Print();
  }

}

Ptr<MapEntry> SimpleMapTables::DatabaseLookup(const Address &eidAddress) {
	// TODO Turn eidAddress in prefix
	NS_LOG_FUNCTION(this);
	Ptr<EndpointId> eid = Create<EndpointId>(eidAddress);
	std::map<Ptr<EndpointId>, Ptr<MapEntry>, CompareEndpointId>::iterator it =
			m_mappingDatabase.find(eid);

	if (it != m_mappingDatabase.end()) {
		Ptr<MapEntry> entry = m_mappingDatabase.at(eid);
		MapTables::DbHit();
		NS_LOG_DEBUG("The found Mapping entry for EID: "<<eidAddress<<"\n"<<entry->GetEidPrefix());
		return entry;
	}
	MapTables::DbMiss();
	return 0;
}

Ptr<MapEntry> SimpleMapTables::CacheLookup(const Address &eidAddress) {
	// TODO Turn eidAddress in prefix
	NS_LOG_FUNCTION(this);
	Ptr<EndpointId> eid = Create<EndpointId>(eidAddress);
	NS_LOG_DEBUG("Searching in Cache Database for EID:"<<eid->GetEidAddress());
	if (m_mappingCache.find(eid) != m_mappingCache.end()) {
		MapTables::CacheHit();
		NS_LOG_DEBUG("Find mapping in Cache for EID:"<<eid->GetEidAddress());
		return m_mappingCache.at(eid);
	}
	NS_LOG_DEBUG("No search result in Cache Database for EID:"<<eid->GetEidAddress());
	MapTables::CacheMiss();
	return 0;
	// TODO investigate lock and unlock
}


Ptr<LispEtrItrApplication> SimpleMapTables::GetxTRApp(){
	return m_xTRApp;
};

void SimpleMapTables::SetxTRApp(Ptr<LispEtrItrApplication> xTRApp){
	m_xTRApp = xTRApp;
};

// Set an ipv4 eid
void SimpleMapTables::SetEntry(const Address &eidAddress, const Ipv4Mask &mask,
		Ptr<MapEntry> mapEntry, MapEntryLocation location) {
	//Note that this method is only valid for Ipv4Address
	NS_ASSERT(Ipv4Address::IsMatchingType(eidAddress));
	Ptr<EndpointId> eid = Create<EndpointId>();

	eid->SetIpv4Mask(mask);
	eid->SetEidAddress(
			static_cast<Address>(Ipv4Address::ConvertFrom(eidAddress).CombineMask(
					mask)));

	mapEntry->SetEidPrefix(eid);

	if (location == IN_DATABASE) {
		m_mutexDatabase.Lock();
		std::map<Ptr<EndpointId>, Ptr<MapEntry>, CompareEndpointId>::iterator it =
				m_mappingDatabase.find(eid);
		if (it != m_mappingDatabase.end())
		{
			//Before erase the found mapEntry, take out the map version number
			//update it (e.g. increment by one) and assign it to the new map entry
			uint16_t newerMapVersionNb = it->second->GetVersionNumber() + 1;
			if (newerMapVersionNb > 1)
			{
				//TODO: add method to set map version number
				//10-07-2017: Setter method has been added.
				//mapEntry->SetVersionNumber(newerMapVersionNb);
			}
			m_mappingDatabase.erase(it);
		}

		m_mappingDatabase.insert(
				std::pair<Ptr<EndpointId>, Ptr<MapEntry> >(eid, mapEntry));

		m_mutexDatabase.Unlock();
	}

	else if (location == IN_CACHE) {
		m_mutexCache.Lock();
		std::map<Ptr<EndpointId>, Ptr<MapEntry>, CompareEndpointId>::iterator it =
				m_mappingCache.find(eid);
		if (it != m_mappingCache.end())
			m_mappingCache.erase(it);
		m_mappingCache.insert(
				std::pair<Ptr<EndpointId>, Ptr<MapEntry> >(eid, mapEntry));
		m_mutexCache.Unlock();
		NS_LOG_DEBUG("Set an Mapping Entry for EID:"<<eid->GetEidAddress());
		/**
		 * TODO: Here we should care about whether we could send the saved invoked-SMR.
		 * First thing: not all simple map table should have a pointer to xTR application.
		 * SimpleMapTables class in MapServer has no such a pointer.
		 * Thus, we first need to make sure whether the xTR smart pointer's value is 0.
		 * If yes. bypass all the following logics.
		 */
		NS_LOG_DEBUG("xTR application pointer of this map table is :"<<m_xTRApp);
		if (m_xTRApp != NULL){
			std::list<Ptr<MapRequestMsg>> m_mapReqMsg = m_xTRApp->GetMapRequestMsgList();
			Ipv4Address eidAddressIpv4 = Ipv4Address::ConvertFrom(eidAddress);
			NS_LOG_DEBUG("The newly received mapping has an EID:"<<eidAddressIpv4);
			for(std::list<Ptr<MapRequestMsg>>::const_iterator it = m_mapReqMsg.begin(); it!=m_mapReqMsg.end();++it){
				// it here is a pointer of pointer??
				Ptr<MapRequestMsg> requestMsg = (*it);
				Ipv4Address queriedEid = Ipv4Address::ConvertFrom(requestMsg->GetItrRlocAddrIp());
				Ipv4Address queriedEidPrefix = queriedEid.CombineMask(mask);
				NS_LOG_DEBUG("The buffered SMR contains an unknown RLOC (i.e. a Local RLOC, an EID):"<<Ipv4Address::ConvertFrom (queriedEid));
				// Should verify if queriedEid belongs to eidAddress (which is a prefix)
				if (eidAddressIpv4.IsEqual(queriedEidPrefix)){
					GetxTRApp()->SendInvokedSmrMsg(requestMsg);
					NS_LOG_DEBUG("Now a buffered invoked-SMR has been sent... ");
					//Don't forget to delete the sent invoked-SMR
//					m_mapReqMsg.remove(*it);
				}
			}
		}
		else{
			NS_LOG_DEBUG("This map table is not managed by any xTR application");
		}
	}
}

// Ipv6
void SimpleMapTables::SetEntry(const Address &eid, const Ipv6Prefix &prefix,
		Ptr<MapEntry> mapEntry, MapEntryLocation location) {
	// TODO SetEntry: do the same for ipv6
}

// Insert Locator
void SimpleMapTables::InsertLocator(const Address &eid, const Ipv4Mask &mask,
		const Ipv6Prefix &prefix, const Address &rlocAddress, uint8_t priority,
		uint8_t weight, MapEntryLocation location, bool reachable) {
	NS_LOG_FUNCTION(this<<&eid<<&mask);
	Ptr<RlocMetrics> rlocMetrics = Create<RlocMetrics>(priority, weight,
			reachable);
	Ptr<Locator> locator = Create<Locator>(rlocAddress);
	locator->SetRlocMetrics(rlocMetrics);
	if (location == IN_DATABASE) {
		Ptr<MapEntry> dbEntry = DatabaseLookup(eid);
		if (dbEntry) {
			dbEntry->InsertLocator(locator);
		} else {
			Ptr<MapEntry> mapEntry = Create<MapEntryImpl>();
			mapEntry->InsertLocator(locator);
			if (Ipv4Address::IsMatchingType(eid))
				SetEntry(eid, mask, mapEntry, IN_DATABASE);
			else if (Ipv6Address::IsMatchingType(eid))
				SetEntry(eid, prefix, mapEntry, IN_DATABASE);
		}
	} else {

		Ptr<MapEntry> dbEntry = CacheLookup(eid);
		if (dbEntry) {
			dbEntry->InsertLocator(locator);
		} else {
			Ptr<MapEntry> mapEntry = Create<MapEntryImpl>();
			mapEntry->InsertLocator(locator);
			if (Ipv4Address::IsMatchingType(eid)){
				SetEntry(eid, mask, mapEntry, IN_CACHE);
				NS_LOG_DEBUG("Insert a Mapping for "<<Ipv4Address::ConvertFrom(eid)<<" into Cache Database!");
			}
			else if (Ipv6Address::IsMatchingType(eid)){
				SetEntry(eid, prefix, mapEntry, IN_CACHE);
			}

		}
	}
}

// wrapper for ipv4 eid and ipv4 rlocAddress
void SimpleMapTables::InsertLocator(const Ipv4Address &eid,
		const Ipv4Mask &mask, const Ipv4Address &rlocAddress, uint8_t priority,
		uint8_t weight, MapEntryLocation location, bool reachable) {
	NS_LOG_FUNCTION(
			this << eid << mask << rlocAddress << uint32_t (priority) << uint32_t (weight) << location << reachable);
	InsertLocator(static_cast<Address>(eid), mask, Ipv6Prefix(),
			static_cast<Address>(rlocAddress), priority, weight, location,
			reachable);
}

void SimpleMapTables::InsertLocator(const Ipv4Address &eid,
		const Ipv4Mask &mask, const Ipv6Address &rlocAddress, uint8_t priority,
		uint8_t weight, MapEntryLocation location, bool reachable) {
	NS_LOG_FUNCTION(
			this << eid << mask << rlocAddress << uint32_t (priority) << uint32_t (weight) << location);
	InsertLocator(static_cast<Address>(eid), mask, Ipv6Prefix(),
			static_cast<Address>(rlocAddress), priority, weight, location,
			reachable);
}

void SimpleMapTables::InsertLocator(const Ipv6Address &eid,
		const Ipv6Prefix &prefix, const Ipv4Address &rlocAddress,
		uint8_t priority, uint8_t weight, MapEntryLocation location,
		bool reachable) {
	// TODO InsertLocator IPv6 eid IPv4 rloc
}

void SimpleMapTables::InsertLocator(const Ipv6Address &eid,
		const Ipv6Prefix &prefix, const Ipv6Address &rlocAddress,
		uint8_t priority, uint8_t weight, MapEntryLocation location,
		bool reachable) {
	// TODO InsertLocator IPv6 eid IPv6 rloc
}

Ptr<Locator> SimpleMapTables::DestinationRlocSelection(
		Ptr<const MapEntry> remoteMapEntry) {
	return remoteMapEntry->RlocSelection();
}

// srcEid est important pour recuperer la liste des locators (sources)
// associe a cet eid
Ptr<Locator> SimpleMapTables::SourceRlocSelection(Address const &srcEid,
		Ptr<const Locator> destLocator) {
	NS_LOG_FUNCTION(this<<"source EID: "<<srcEid<<" Destination RLOC Address: "<<destLocator->GetRlocAddress());
	NS_LOG_DEBUG("Selecting Source Locator for the source eid: " << Ipv4Address::ConvertFrom(srcEid) << " to dest locator " << Ipv4Address::ConvertFrom(destLocator->GetRlocAddress()));
	NS_ASSERT(destLocator);

	bool isDestIpv4 = false;
	Ptr<Node> node = (MapTables::GetLispOverIp())->GetNode();
	Socket::SocketErrno errno_;
	Address srcAddress;
	Ptr<Locator> srcLocator;
	// if the destination is Ipv4
	if (Ipv4Address::IsMatchingType(destLocator->GetRlocAddress())) {
		isDestIpv4 = true;
		Ptr<Ipv4Route> route = 0;
		Ptr<Ipv4RoutingProtocol> routingProtocol =
				node->GetObject<Ipv4>()->GetRoutingProtocol();

		if (routingProtocol != 0) {
			Ipv4Header header = Ipv4Header();
			header.SetDestination(
					Ipv4Address::ConvertFrom(destLocator->GetRlocAddress()));
			route = routingProtocol->RouteOutput(0, header, 0, errno_);
		} else {
			NS_LOG_ERROR(
					"SimpleMapTables::SelectSrcRloc: routingProtocol == 0");
		}

		if (route) {
			// we know the route and we get the output interface
			int32_t interface = node->GetObject<Ipv4>()->GetInterfaceForDevice(
					route->GetOutputDevice());
			// we get the interface address
			NS_LOG_DEBUG("The layer 3 interface corresponding to selected route is: "<<interface);
			Ipv4Address ipv4SrcAddress = (node->GetObject<Ipv4>()->GetAddress(
					interface, 0)).GetLocal();
			srcAddress = static_cast<Address>(ipv4SrcAddress);
			NS_LOG_DEBUG("With the given route, the selected source RLOC address "<<Ipv4Address::ConvertFrom(ipv4SrcAddress));
		} else {
			NS_LOG_WARN("No route to destination locator. Return.");
			return 0;
		}
	} else if (Ipv6Address::IsMatchingType(destLocator->GetRlocAddress())) {
		Ptr<Ipv6Route> route = 0;
		Ptr<Ipv6RoutingProtocol> routingProtocol =
				node->GetObject<Ipv6>()->GetRoutingProtocol();
		if (routingProtocol) {
			Ipv6Header header = Ipv6Header();
			header.SetDestinationAddress(
					Ipv6Address::ConvertFrom(destLocator->GetRlocAddress()));
			route = routingProtocol->RouteOutput(0, header, 0, errno_);
		} else {
			NS_LOG_ERROR(
					"SimpleMapTables::SelectSrcRloc: routingProtocol == 0");
		}

		if (route) {
			// we know the route and we get the output interface
			int32_t interface = node->GetObject<Ipv6>()->GetInterfaceForDevice(
					route->GetOutputDevice());
			// we get the inteface address
			Ipv6Address ipv6SrcAddress = (node->GetObject<Ipv6>()->GetAddress(
					interface, 0)).GetAddress();
			srcAddress = static_cast<Address>(ipv6SrcAddress);
		} else {
			NS_LOG_WARN("No route to destination locator. Return.");
			return 0;
		}
	} else
		return 0; // should not happen

	// Now we take the mapping entry and look for a Locator address that
	// corresponds to the outgoing interface address

	// Get the mapping entry corresponding to the EID in the db
	Ptr<MapEntry> srcEidMapEntry = DatabaseLookup(srcEid);

	if (srcEidMapEntry != 0) {
		Ptr<Locators> locs = srcEidMapEntry->GetLocators();
		NS_LOG_DEBUG("Pointer to the list of Locators: "<<locs);
		// we look for an RLOC address that corresponds to the output ifAddress
		srcLocator = locs->FindLocator(srcAddress);

		if (!srcLocator) {
			NS_LOG_ERROR(
					"No locator address for outgoing interface " << Ipv4Address::ConvertFrom (srcAddress));
			return 0;
		}
	}
	else{
	    //TODO: In fact, if the srcEidMapEntry is 0, the code afterwards will crash...
	    // Maybe here we need to add NS_ASSERT?
	    NS_LOG_ERROR("srcEidMapEntry = 0 -> No EID-RLOC mapping for "<<srcEid);
	    NS_ASSERT(srcEidMapEntry != 0);
	}
	if (isDestIpv4) {
		// should never be true
		if (!Ipv4Address::IsMatchingType(srcLocator->GetRlocAddress()))
			return 0;
	} else {
		if (!Ipv6Address::IsMatchingType(srcLocator->GetRlocAddress()))
			return 0;
	}
	// Doit on verifier si c'est une interface locale alors qu'on sait que l'adresse est celle de l'interface de sortie ?
	// TODO add it back in condition && srcLocator->GetRlocMetrics ()->IsLocalInterface ()
	if (srcLocator->GetRlocMetrics()->IsUp()
			&& srcLocator->GetRlocMetrics()->IsLocalInterface()
			&& srcLocator->GetRlocMetrics()->GetPriority()
					< LispOverIp::LISP_MAX_RLOC_PRIO)
	  {
	    NS_LOG_DEBUG("Selected Source RLOC Address: "<<srcLocator->GetRlocAddress());
	    return srcLocator;
	  }


	NS_LOG_LOGIC("No source locator found for source Address");
	return 0;
}

void SimpleMapTables::GetMapEntryList(MapTables::MapEntryLocation location,
		std::list<Ptr<MapEntry> > &entryList) {

	if (location == MapTables::IN_DATABASE) {
		for (std::map<Ptr<EndpointId>, Ptr<MapEntry> >::iterator it =
				m_mappingDatabase.begin(); it != m_mappingDatabase.end();
				++it) {
			it->second->SetEidPrefix(it->first);
			entryList.push_back(it->second);
		}
	} else if (location == MapTables::IN_CACHE) {
		for (std::map<Ptr<EndpointId>, Ptr<MapEntry> >::iterator it =
				m_mappingCache.begin(); it != m_mappingCache.end(); ++it) {
			it->second->SetEidPrefix(it->first);
			entryList.push_back(it->second);
		}
	}
}

bool SimpleMapTables::IsMapForReceivedPacket(Ptr<const Packet> p,
		const LispHeader &header, const Address &srcRloc,
		const Address &destRloc) {
	NS_LOG_FUNCTION(this);
	Ipv4Header innerHeader;
	Ptr<Locator> srcLocator;
	Ptr<Locator> destLocator;
	Ptr<LispOverIp> lispOverIp = MapTables::GetLispOverIp();
	p->PeekHeader(innerHeader);

	Address srcEidAddress = static_cast<Address>(innerHeader.GetSource());
	Address destEidAddress = static_cast<Address>(innerHeader.GetDestination());

	// The destination address should be in the Db
	/* TAKE CARE OF THE DESTRLOC */
	Ptr<MapEntry> localMapEntry = DatabaseLookup(destEidAddress);

	if (!localMapEntry) {
		/* we received a LISP packet addressed to the wrong ETR
		 * we assume that the local mapping is alway present
		 * TODO Error no mapping in DB DROP
		 * TODO I know why in Lionel's design, MR is not a xTR
		 * (a normal router). In LISP-MN <--> Normal CN communication case,
		 * map request message from LISP-MN is encapsulated into a lisp data
		 * plan packet. Since MR's RLOC is a gloablly routable ip address,
		 * this packet should be decapsulated at MR (if MR is not a xTR, how it
		 * could be done? place a xTR before MR? annouce a route to this MR? which
		 * solution is better?)
		 *
		 * Any it is a good solution to counting local mapping miss here?
		 */
		lispOverIp->GetLispStatisticsV4()->NoLocalMap();
		return false;
	}

	destLocator = localMapEntry->FindLocator(destRloc);
	if (!destLocator) {
		/*
		 * the mapping exists in the database but the RLOC address
		 *  doesn't.
		 *  this should not happen
		 *  TODO DROP packet
		 */
		lispOverIp->GetLispStatisticsV4()->NoLocalMap();
		return false;
	}
	NS_LOG_DEBUG(
			"Local Map Entry exists for destination eid " << Ipv4Address::ConvertFrom(destEidAddress) << ": " << localMapEntry->RlocSelection ());
	/* TAKE CARE OF THE SRCRLOC */
	Ptr<MapEntry> remoteMapEntry = CacheLookup(srcEidAddress);

	if (remoteMapEntry) {
		srcLocator = remoteMapEntry->FindLocator(srcRloc);
		if (!srcLocator) {
			/*
			 * The entry exists in the cache but the
			 * RLOC address of the remote ETR doesn't
			 * We should notify through socket or not
			 * TODO Drop Packet;
			 */
			lispOverIp->GetLispStatisticsV4()->NoLocalMap();
			return false;
		}
	} else {
		/*
		 * There is no entry in the cache...
		 * This should happens the first time we look for
		 * an eid address
		 * NB: there are 3 behavior possible,
		 * here we choose the default one --- Do Nothing.
		 * TODO Notify through socket
		 */
		/*Do nothing*/;
	}

	// check the LISP header
	if (!LispOverIp::CheckLispHeader(header, localMapEntry, remoteMapEntry,
			srcLocator, destLocator, lispOverIp)) {
		//lispOverIp->GetLispStatisticsV4 ()->;
		/*
		 * There is an error with the lisp header
		 * TODO DROP packet
		 */

		return false;
	}

	return true;
}

//////////////////
// MapEntryImpl

MapEntryImpl::MapEntryImpl() {
	m_locators = Create<LocatorsImpl>();
}

MapEntryImpl::MapEntryImpl(Ptr<Locator> locator) {
	m_locators = Create<LocatorsImpl>();
	m_locators->InsertLocator(locator);
}

MapEntryImpl::~MapEntryImpl() {

}


//TODO: If one
Ptr<Locator> MapEntryImpl::FindLocator(const Address &address) const {
	return m_locators->FindLocator(address);
}

Ptr<Locator> MapEntryImpl::RlocSelection(void) const {
	return m_locators->SelectFirsValidRloc();
}

std::string MapEntryImpl::Print(void) const {
	//std::string mapEntry = "EID prefix:";

	return m_locators->Print();
}

} /* namespace ns3 */
