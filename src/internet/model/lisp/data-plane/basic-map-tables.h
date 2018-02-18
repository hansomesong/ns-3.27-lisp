/*
 * basic-map-tables.h
 *
 *  Created on: Feb 17, 2018
 *      Author: qsong
 */

#ifndef SRC_INTERNET_MODEL_LISP_DATA_PLANE_BASIC_MAP_TABLES_H_
#define SRC_INTERNET_MODEL_LISP_DATA_PLANE_BASIC_MAP_TABLES_H_

#include "map-tables.h"

namespace ns3
{

	class BasicMapTables : public MapTables
	{
	public:

    BasicMapTables ();

		virtual
		~BasicMapTables ();

//    static TypeId
//		GetTypeId (void);
//
//    int
//    GetNMapEntries (void);
//
//    /**
//     * \brief Get the number of mapping entries in LISP batabase
//     * \return the number of mapping entries in LISP database
//     */
//    int
//    GetNMapEntriesLispDataBase (void);
//    /**
//     * \brief Get the number of mapping entries in LISP Cache.
//     * \return the number of mapping entries in LISP Cache.
//     */
//    int
//    GetNMapEntriesLispCache (void);
//
//    /**
//     * \brief Print the Map Table content to the given output stream
//     * \param The output stream to which this SimpleMapTables is printed
//     */
//    void Print (std::ostream &os) const;
//
//    Ptr<MapEntry>
//    DatabaseLookup (const Address &eidAddress);
//
//    Ptr<MapEntry>
//    CacheLookup (const Address &eidAddress);
//
//    void
//    SetEntry (const Address &eid, const Ipv4Mask &mask, Ptr<MapEntry> mapEntry,
//	      MapEntryLocation location);
//
//    void
//    SetEntry (const Address &eid, const Ipv6Prefix &prefix,
//	      Ptr<MapEntry> mapEntry, MapEntryLocation location);
//
//
//    Ptr<LispEtrItrApplication> GetxTRApp();
//    void SetxTRApp(Ptr<LispEtrItrApplication> xTRApp);
//
//    // Insert Locator
//    void
//    InsertLocator (const Ipv4Address &eid, const Ipv4Mask &mask,
//		   const Ipv4Address &rlocAddress, uint8_t priority,
//		   uint8_t weight, MapEntryLocation location, bool reachable);
//
//    void
//    InsertLocator (const Ipv4Address &eid, const Ipv4Mask &mask,
//		   const Ipv6Address &rlocAddress, uint8_t priority,
//		   uint8_t weight, MapEntryLocation location, bool reachable);
//
//    void
//    InsertLocator (const Ipv6Address &eid, const Ipv6Prefix &prefix,
//		   const Ipv4Address &rlocAddress, uint8_t priority,
//		   uint8_t weight, MapEntryLocation location, bool reachable);
//
//    void
//    InsertLocator (const Ipv6Address &eid, const Ipv6Prefix &prefix,
//		   const Ipv6Address &rlocAddress, uint8_t priority,
//		   uint8_t weight, MapEntryLocation location, bool reachable);
//
//    Ptr<Locator>
//    DestinationRlocSelection (Ptr<const MapEntry> remoteMapEntry);
//
//    Ptr<Locator>
//    SourceRlocSelection (Address const &srcEid, Ptr<const Locator> destLocator);
//
//    bool
//    IsMapForReceivedPacket (Ptr<const Packet> p, const LispHeader &header,
//			    const Address &srcRloc, const Address &destRloc);
//
//    void
//    GetMapEntryList (MapTables::MapEntryLocation location,
//		     std::list<Ptr<MapEntry> > &entryList);
//
//    void
//    InsertLocator (const Address &eid, const Ipv4Mask &mask,
//		   const Ipv6Prefix &prefix, const Address &rlocAddress,
//		   uint8_t priority, uint8_t weight, MapEntryLocation location,
//		   bool reachable);


  private:
    std::map<Ptr<EndpointId>, Ptr<MapEntry>, CompareEndpointId> m_mappingCache;
    std::map<Ptr<EndpointId>, Ptr<MapEntry>, CompareEndpointId> m_mappingDatabase;
	};

} /* namespace ns3 */

#endif /* SRC_INTERNET_MODEL_LISP_DATA_PLANE_BASIC_MAP_TABLES_H_ */
