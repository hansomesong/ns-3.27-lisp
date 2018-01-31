/*
 * MapTables.h
 *
 *  Created on: 28 janv. 2016
 *      Author: lionel
 */

#ifndef MAPTABLES_H_
#define MAPTABLES_H_

//#include "ns3/lisp-etr-itr-application.h"
#include "ns3/lisp-over-ip.h"
#include "ns3/packet.h"
#include "ns3/locators.h" //it included lisp-protcol.h
//#include "ns3/ptr.h"
#include "ns3/simple-ref-count.h"
#include "ns3/map-entry.h"



namespace ns3
{
/**
 * We encounter a cyclic reference situation between xTR application and map table here.
 * We use forward declaration to break the cycle.
 * Note that we have no include directive for xTR application header file.
 */
class LispEtrItrApplication;
class LispHeader;
class LispOverIp;

class MapTables : public Object
{
public:

  static TypeId GetTypeId (void);
  MapTables ();
  virtual ~MapTables ();

  virtual Ptr<MapEntry> DatabaseLookup (const Address &eid) = 0;

  virtual Ptr<MapEntry> CacheLookup (const Address &eid) = 0;

  virtual int GetNMapEntries (void)=0;
  /**
   * \brief Get the number of mapping entries in LISP batabase
   * \return the number of mapping entries in LISP database
   */
  virtual int GetNMapEntriesLispDataBase (void)=0;
  /**
   * \brief Get the number of mapping entries in LISP Cache.
   * \return the number of mapping entries in LISP Cache.
   */
  virtual int GetNMapEntriesLispCache (void)=0;

  /**
   * \brief Print the Map Table content to the given output stream
   * \param The output stream to which this SimpleMapTables is printed
   */
  virtual void Print (std::ostream &os) const=0;

  enum MapEntryLocation {
    IN_DATABASE = 0,
    IN_CACHE = 1
  };

  virtual void SetEntry (const Address &eid, const Ipv4Mask &mask, Ptr<MapEntry> mapEntry, MapEntryLocation location) = 0;
  virtual void SetEntry (const Address &eid, const Ipv6Prefix &prefix, Ptr<MapEntry> mapEntry, MapEntryLocation location) = 0;

  virtual void InsertLocator (const Ipv4Address &eid, const Ipv4Mask &mask, const Ipv4Address &rlocAddress, uint8_t priority, uint8_t weight, MapEntryLocation location, bool reachable) = 0;
  virtual void InsertLocator (const Ipv4Address &eid, const Ipv4Mask &mask, const Ipv6Address &rlocAddress, uint8_t priority, uint8_t weight, MapEntryLocation location, bool reachable) = 0;
  virtual void InsertLocator (const Ipv6Address &eid, const Ipv6Prefix &prefix, const Ipv4Address &rlocAddress, uint8_t priority, uint8_t weight, MapEntryLocation location, bool reachable) = 0;
  virtual void InsertLocator (const Ipv6Address &eid, const Ipv6Prefix &prefix, const Ipv6Address &rlocAddress, uint8_t priority, uint8_t weight, MapEntryLocation location, bool reachable) = 0;

  virtual Ptr<Locator> DestinationRlocSelection (Ptr<const MapEntry> remoteMapEntry) = 0;

  virtual Ptr<Locator> SourceRlocSelection (Address const &srcEid, Ptr<const Locator> destLocator) = 0;

  virtual bool IsMapForReceivedPacket (Ptr <const Packet> p, const LispHeader &header, const Address &srcRloc, const Address &destRloc) = 0;

  virtual void GetMapEntryList (MapEntryLocation location, std::list<Ptr<MapEntry> > &entryList) = 0;

  // TODO Add map_notify, map_check_lsbits ?,

  /**
   * Lisp Protocol associated to this MapTables
   * @return
   */
  Ptr<LispOverIp> GetLispOverIp (void);
  void SetLispOverIp (Ptr<LispOverIp> lispProtocol);

  /**
   * About how a derived class of the current class acesses an attribute of the base class:
   * http://www.cplusplus.com/forum/general/5323/
   * Another possibility is to use virtual function (i.e. polymorphism)
   * http://www.cplusplus.com/doc/tutorial/polymorphism/
   * In its dervied class, no need to declare the following two methods again.
   */
  virtual Ptr<LispEtrItrApplication> GetxTRApp()=0;
  virtual void SetxTRApp(Ptr<LispEtrItrApplication> xTRApp)=0;

  void DbMiss (void);
  void DbHit (void);
  void CacheHit (void);
  void CacheMiss (void);

  struct CompareEndpointId
   {
     //TODO: If want to support Ipv6, should modify...
     bool
     operator() (const Ptr<EndpointId> a, const Ptr<EndpointId> b) const
     {
	if (a->IsIpv4 ())
	  {
	    if (b->GetIpv4Mask ().IsEqual (Ipv4Mask ()))
	      {
		if (Ipv4Address::ConvertFrom (a->GetEidAddress ()).CombineMask (
		    a->GetIpv4Mask ()).Get ()
		    > Ipv4Address::ConvertFrom (b->GetEidAddress ()).CombineMask (
			a->GetIpv4Mask ()).Get ())
		  return true;
		else if (Ipv4Address::ConvertFrom (a->GetEidAddress ()).CombineMask (
		    a->GetIpv4Mask ()).Get ()
		    < Ipv4Address::ConvertFrom (b->GetEidAddress ()).CombineMask (
			a->GetIpv4Mask ()).Get ())
		  return false;
	      }
	    else if (a->GetIpv4Mask ().IsEqual (Ipv4Mask ()))
	      {
		return Ipv4Address::ConvertFrom (a->GetEidAddress ()).CombineMask (
		    b->GetIpv4Mask ()).Get ()
		    > Ipv4Address::ConvertFrom (b->GetEidAddress ()).CombineMask (
			b->GetIpv4Mask ()).Get ();
	      }
	    else
	      {
		if (Ipv4Address::ConvertFrom (a->GetEidAddress ()).CombineMask (
		    a->GetIpv4Mask ()).Get ()
		    > Ipv4Address::ConvertFrom (b->GetEidAddress ()).CombineMask (
			b->GetIpv4Mask ()).Get ())
		  return true;
		else if (Ipv4Address::ConvertFrom (a->GetEidAddress ()).CombineMask (
		    a->GetIpv4Mask ()).Get ()
		    < Ipv4Address::ConvertFrom (b->GetEidAddress ()).CombineMask (
			b->GetIpv4Mask ()).Get ())
		  return false;
		else
		  {
		    return a->GetIpv4Mask ().GetPrefixLength ()
			> b->GetIpv4Mask ().GetPrefixLength ();
		  }
	      }
	  }
	else
	  {
	    // TODO do the same for Ipv6
	    if (b->GetIpv6Prefix ().IsEqual (Ipv6Prefix ()))
	      {
		return !(Ipv6Address::ConvertFrom (a->GetEidAddress ()).CombinePrefix (
		    a->GetIpv6Prefix ())
		    < Ipv6Address::ConvertFrom (b->GetEidAddress ()).CombinePrefix (
			a->GetIpv6Prefix ()));
	      }
	    else if (a->GetIpv6Prefix ().IsEqual (Ipv6Prefix ()))
	      return !(Ipv6Address::ConvertFrom (a->GetEidAddress ()).CombinePrefix (
		  b->GetIpv6Prefix ())
		  < Ipv6Address::ConvertFrom (b->GetEidAddress ()).CombinePrefix (
		      b->GetIpv6Prefix ()));
	    else
	      {
		if (Ipv6Address::ConvertFrom (a->GetEidAddress ()).CombinePrefix (
		    a->GetIpv6Prefix ())
		    < Ipv6Address::ConvertFrom (b->GetEidAddress ()).CombinePrefix (
			b->GetIpv6Prefix ()))
		  {
		    return false;
		  }
		else if (Ipv6Address::ConvertFrom (b->GetEidAddress ()).CombinePrefix (
		    b->GetIpv6Prefix ())
		    < Ipv6Address::ConvertFrom (a->GetEidAddress ()).CombinePrefix (
			a->GetIpv6Prefix ()))
		  {
		    return true;
		  }
		else
		  {
		    return a->GetIpv6Prefix ().GetPrefixLength ()
			> b->GetIpv6Prefix ().GetPrefixLength ();
		  }
	      }
	  }
	return false;
     }
   };

//protected:
//  // Add this pointer so that when cache insertion event occurs, xTR application can trigger invoked-SMR send procedure
//  Ptr<LispEtrItrApplication> m_xTRApp;

private:
  Ptr<LispOverIp> m_lispProtocol;


  uint32_t m_dbMiss;    // # failed lookups in db
  uint32_t m_dbHit;     // # successful lookups in db
  uint32_t m_cacheMiss; // # failed lookups in cache
  uint32_t m_cacheHit;  // # successful lookups in cache
};

std::ostream& operator<< (std::ostream &os, MapTables const &mapTable);

} /* namespace ns3 */

#endif /* MAPTABLES_H_ */
