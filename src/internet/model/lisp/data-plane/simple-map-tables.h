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
#ifndef SIMPLE_MAP_TABLES_H_
#define SIMPLE_MAP_TABLES_H_

//#include <ns3/address.h>
#include <ns3/packet.h>
//#include <ns3/ptr.h>
//#include <list> Do not need. Since "locators-impl.h" has alread included...
#include <map>
#include <ns3/log.h>
#include <ns3/system-mutex.h>

#include "ns3/lisp-protocol.h"
#include "ns3/map-tables.h"
#include "ns3/locators-impl.h"

#include "ns3/map-request-msg.h"
#include "ns3/lisp-etr-itr-application.h"


namespace ns3
{
  class TypeId;
//
} /* namespace ns3 */

namespace ns3
{
//	class LispEtrItrApplication;
  class Address;

  /**
   *
   */
  class SimpleMapTables : public MapTables
  {
  public:
    /**
     *
     * @return
     */
    static TypeId
    GetTypeId (void);
    SimpleMapTables ();
    virtual
    ~SimpleMapTables ();

    int
    GetNMapEntries (void);
    /**
     * \brief Get the number of mapping entries in LISP batabase
     * \return the number of mapping entries in LISP database
     */
    int
    GetNMapEntriesLispDataBase (void);
    /**
     * \brief Get the number of mapping entries in LISP Cache.
     * \return the number of mapping entries in LISP Cache.
     */
    int
    GetNMapEntriesLispCache (void);

    /**
     * \brief Print the Map Table content to the given output stream
     * \param The output stream to which this SimpleMapTables is printed
     */
    void Print (std::ostream &os) const;

    Ptr<MapEntry>
    DatabaseLookup (const Address &eidAddress);

    Ptr<MapEntry>
    CacheLookup (const Address &eidAddress);

    void
    SetEntry (const Address &eid, const Ipv4Mask &mask, Ptr<MapEntry> mapEntry,
	      MapEntryLocation location);
    void
    SetEntry (const Address &eid, const Ipv6Prefix &prefix,
	      Ptr<MapEntry> mapEntry, MapEntryLocation location);


    Ptr<LispEtrItrApplication> GetxTRApp();
    void SetxTRApp(Ptr<LispEtrItrApplication> xTRApp);

    // Insert Locator
    void
    InsertLocator (const Ipv4Address &eid, const Ipv4Mask &mask,
		   const Ipv4Address &rlocAddress, uint8_t priority,
		   uint8_t weight, MapEntryLocation location, bool reachable);
    void
    InsertLocator (const Ipv4Address &eid, const Ipv4Mask &mask,
		   const Ipv6Address &rlocAddress, uint8_t priority,
		   uint8_t weight, MapEntryLocation location, bool reachable);
    void
    InsertLocator (const Ipv6Address &eid, const Ipv6Prefix &prefix,
		   const Ipv4Address &rlocAddress, uint8_t priority,
		   uint8_t weight, MapEntryLocation location, bool reachable);
    void
    InsertLocator (const Ipv6Address &eid, const Ipv6Prefix &prefix,
		   const Ipv6Address &rlocAddress, uint8_t priority,
		   uint8_t weight, MapEntryLocation location, bool reachable);

    Ptr<Locator>
    DestinationRlocSelection (Ptr<const MapEntry> remoteMapEntry);

    Ptr<Locator>
    SourceRlocSelection (Address const &srcEid, Ptr<const Locator> destLocator);

    bool
    IsMapForReceivedPacket (Ptr<const Packet> p, const LispHeader &header,
			    const Address &srcRloc, const Address &destRloc);

    void
    GetMapEntryList (MapTables::MapEntryLocation location,
		     std::list<Ptr<MapEntry> > &entryList);


  private:
    void
    InsertLocator (const Address &eid, const Ipv4Mask &mask,
		   const Ipv6Prefix &prefix, const Address &rlocAddress,
		   uint8_t priority, uint8_t weight, MapEntryLocation location,
		   bool reachable);

    SystemMutex m_mutexCache;
    SystemMutex m_mutexDatabase;
    std::map<Ptr<EndpointId>, Ptr<MapEntry>, CompareEndpointId> m_mappingCache;
    std::map<Ptr<EndpointId>, Ptr<MapEntry>, CompareEndpointId> m_mappingDatabase;
    Ptr<LispEtrItrApplication> m_xTRApp;



  };

  /**
   *
   */

  /**
   *
   */
  class MapEntryImpl : public MapEntry
  {
  public:
    MapEntryImpl ();
    MapEntryImpl (Ptr<Locator> locator);
    virtual
    ~MapEntryImpl ();

    Ptr<Locator>
    FindLocator (const Address &address) const;
    Ptr<Locator>
    RlocSelection (void) const;
    std::string
    Print (void) const;
  };
  std::ostream& operator<< (std::ostream &os, SimpleMapTables const &simpleMapTable);
} /* namespace ns3 */

#endif /* SIMPLE_MAP_TABLES_H_ */
