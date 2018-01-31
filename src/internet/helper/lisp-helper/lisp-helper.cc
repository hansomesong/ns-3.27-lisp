/*
 * internet-stack-with-lisp-helper.cpp
 *
 *  Created on: 30 janv. 2016
 *      Author: lionel
 */

#include "lisp-helper.h"

#include <iostream>
#include <string>
#include <fstream>

#include "ns3/log.h"
#include "ns3/assert.h"
#include "ns3/fatal-error.h"
#include "ns3/fatal-impl.h"
#include "ns3/ipv4.h"
#include "ns3/ipv6.h"
#include "ns3/lisp-over-ipv4-impl.h"
#include "ns3/node.h"
#include "ns3/node-container.h"
#include "ns3/object-factory.h"
#include "ns3/ptr.h"
#include "ns3/lisp-over-ipv6-impl.h"
#include "ns3/map-tables.h"
#include "ns3/simple-map-tables.h"
#include "ns3/lisp-protocol.h"

using std::cout;

namespace ns3
{



  NS_LOG_COMPONENT_DEFINE ("LispHelper");

  LispHelper::LispHelper (void)
  {
  }

  LispHelper::~LispHelper ()
  {
    // TODO Auto-generated destructor stub
  }

  void LispHelper::Install (Ptr<Node> node) const
  {
    NS_LOG_FUNCTION (this);

    // neither ipv4 nor ipv6
    if (node->GetObject<Ipv4> () == 0 && node->GetObject<Ipv6> () == 0) {
        NS_FATAL_ERROR ("LispHelper::Install (): Aggregating LISP"
                            "to a node with no existing ipv6 or ipv4 object");
        return;
    }
    if (node->GetObject<Ipv4> () == 0)
      {
        NS_LOG_WARN ("LispHelper::Install (): Aggregating LISPv4"
            "to a node with no existing ipv4 object");
      }
    else
      {
        NS_LOG_WARN ("LispHelper::Install: Aggregeting LISP ipv4");
        CreateAndAggregateLispVersion(node, LispOverIpv4Impl::GetTypeId ().GetName ());
      }

    if (node->GetObject<Ipv6> () == 0)
      {
        NS_LOG_WARN ("LispHelper::Install (): Aggregating LISPv6"
                    "to a node with no existing ipv6 object");
      }
    else
      {
        CreateAndAggregateLispVersion(node, LispOverIpv6Impl::GetTypeId ().GetName ());
      }
    Ptr<LispOverIp> lisp = node->GetObject<LispOverIp> ();
    lisp->SetRlocsList (m_rlocsList);
    // now we can open the mapping socket
    lisp->OpenLispMappingSocket ();
    /**
     * After trying and comparing, it is better to create mapTablesv4 and mapTablesv6
     * within lisp-helper. For normal xTRs, it makes no sense, because LispHelper::InstallMapTables()
     * iterates the NodeContainer and assign the corresponding map Tables to lispOverIpv4. The two
     * smart pointers will be replace.
     * However, for LISP-MN, the following two instructions are important: because program
     * 's configuration XML file has no entries for LISP-MN. Thus, LISP-MN has no map tables,
     * which will lead to segmentation fault for packet copy-forwarding.
     * We make sure once lispOverIpv4 is created, mapTables are always accessible.
     */
    Ptr<MapTables> mapTablesv4 = Create<SimpleMapTables>();
    Ptr<MapTables> mapTablesv6 = Create<SimpleMapTables>();
    lisp->SetMapTablesIpv4(mapTablesv4);
    lisp->SetMapTablesIpv6(mapTablesv6);
    /**
     * IMPORTANT: DO NOT FORGET TO CREATE LispStatistics for lispOverIpv4 object.
     * Otherwise the statistics work in LispOverIpv4Impl::LispOutput will encounter
     * segmentation fault (cause these two statistics-related object is not created!)
     *
     * At the first glance, you would say it is wrong to set a statistics object for ipv6,
     * for a ipv4 lisp. However, note that ipv4 and ipv4 can be used mixly. for example
     * inner header is ipv6, but the withdrew may be ipv4.
     */
    Ptr<LispStatistics> statisticsForV4 = Create<LispStatistics> ();
    Ptr<LispStatistics> statisticsForV6 = Create<LispStatistics> ();
    lisp->SetLispStatistics (statisticsForV4, statisticsForV6);
  }

  void LispHelper::Install (NodeContainer c) const
  {
    for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
      {
        Install (*i);
      }
  }

  void LispHelper::InstallAll (void) const
  {
    Install (NodeContainer::GetGlobal ());
  }

  void LispHelper::InstallMapTables(NodeContainer c)
  {
    NS_LOG_FUNCTION (this);
    int devices = 0;
    for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
      {
        Ptr<MapTables> mapTablesIpv4 = 0;
        Ptr<MapTables> mapTablesIpv6 = 0;
        Ptr<LispStatistics> statisticsForV4;
        Ptr<LispStatistics> statisticsForV6;
        bool ok = false;
        std::vector<Address> ifAddresses;
        Ptr<Node> node = *i;
        devices = node->GetNDevices ();
        Ptr<Ipv4> ipv4 = node->GetObject<Ipv4> ();
        Ptr<Ipv6> ipv6 = node->GetObject<Ipv6> ();
        Ptr<LispOverIpv4> lispv4;
        Ptr<LispOverIpv6> lispv6;

        if (ipv4)
          lispv4 = node->GetObject<LispOverIpv4> ();
        if (ipv6)
          lispv6 = node->GetObject<LispOverIpv6> ();

        for (int j = 0; j < devices; ++j)
          {

            // get address of the interface associated to device
            uint32_t interface;
            Address ifAddress;

            if (ipv4) {
                interface = ipv4->GetInterfaceForDevice(node->GetDevice (j));
                ifAddress = static_cast<Address> (ipv4->GetAddress(interface, 0).GetLocal ());
            }
            else if (ipv6) {
               interface = ipv6->GetInterfaceForDevice (node->GetDevice(j));
               ifAddress = static_cast<Address> (ipv6->GetAddress(interface, 0).GetAddress ());
            }

            ifAddresses.push_back (ifAddress);

            //NS_LOG_DEBUG ("If Address type: " << ifAddress.);
            //if (Ipv4Address::IsMatchingType (ifAddress))
            NS_LOG_DEBUG ("CHECKING DEVICES ADDRESSES FOR MAPTABLES" << Ipv4Address::ConvertFrom (ifAddress));
            if (!ok && m_mapTablesIpv4.find (ifAddress) != m_mapTablesIpv4.end ())
              {
                mapTablesIpv4 = m_mapTablesIpv4.at (ifAddress);
                // they are added at the same time so if mapTablesIpv4 exists, the stats also exist.
                statisticsForV4 = m_lispStatisticsMapForV4.at (ifAddress);
                statisticsForV6 = m_lispStatisticsMapForV6.at (ifAddress);
                lispv4->SetMapTablesIpv4 (mapTablesIpv4);
                lispv4->SetLispStatistics (statisticsForV4, statisticsForV6);
              }
            if (!ok && m_mapTablesIpv6.find (ifAddress) != m_mapTablesIpv6.end ())
              {
                ok = true;
                mapTablesIpv6 = m_mapTablesIpv6.at (ifAddress);
                lispv4->SetMapTablesIpv6 (m_mapTablesIpv6.at (ifAddress));
              }
            // TODO do the same for LispOverIpv6
          }

        if (ok)
          {
            NS_LOG_DEBUG ("Setting LOCAL INTEFACES");
            std::list<Ptr<MapEntry> > entriesv4;
            std::list<Ptr<MapEntry> > entriesv6;
            mapTablesIpv4->GetMapEntryList(MapTables::IN_DATABASE, entriesv4);
            mapTablesIpv6->GetMapEntryList(MapTables::IN_DATABASE, entriesv6);
            for (std::list<Ptr<MapEntry> >::iterator it =
                entriesv4.begin (); it != entriesv4.end (); ++it)
              {
                Ptr<MapEntry> entry = *it;
                for (std::vector<Address>::iterator itv = ifAddresses.begin (); itv != ifAddresses.end (); itv++)
                  {
                    uint32_t iface = -1;
                    Ptr<Locator> locator = entry->FindLocator (*itv);
                    if (locator)
                      {
                        // set rloc is local iface
                        locator->GetRlocMetrics ()->SetIsLocalIf (true);
                        if (Ipv4Address::IsMatchingType (*itv))
                          {
                            iface = ipv4->GetInterfaceForAddress (
                                Ipv4Address::ConvertFrom (*itv));
                            // set local iface mtu
                            locator->GetRlocMetrics ()->SetMtu (
                                ipv4->GetMtu (iface));
                            // set local iface down if it is manually set up while it is really down
                            // Note: on can manually set lisp down for that iface while it is really up.
                            if (locator->GetRlocMetrics ()->IsUp () && !ipv4->IsUp(iface))
                              {
                                locator->GetRlocMetrics ()->SetUp (false);
                              }
                          }
                        else if (Ipv6Address::IsMatchingType (*itv))
                          {
                            iface = ipv6->GetInterfaceForAddress (
                                Ipv6Address::ConvertFrom (*itv));
                            locator->GetRlocMetrics ()->SetMtu (
                                ipv6->GetMtu (iface));
                          }
                      }
                  }
              }

            // set local entry attributes and flags (v6 entries)
            for (std::list<Ptr<MapEntry> >::iterator it =
                entriesv6.begin (); it != entriesv6.end (); ++it)
              {
                Ptr<MapEntry> entry = *it;

                for (std::vector<Address>::iterator itv = ifAddresses.begin ();
                    itv != ifAddresses.end (); itv++)
                  {
                    Ptr<Locator> locator = entry->FindLocator (*itv);
                    if (locator)
                      {
                        locator->GetRlocMetrics ()->SetIsLocalIf (true);
                        if (locator->GetRlocMetrics ()->GetMtu () < 0)
                          {
                            if (Ipv4Address::IsMatchingType (*itv))
                              locator->GetRlocMetrics ()->SetMtu (
                                  ipv4->GetMtu (
                                      ipv4->GetInterfaceForAddress (
                                          Ipv4Address::ConvertFrom (*itv))));
                            else if (Ipv6Address::IsMatchingType (*itv))
                              locator->GetRlocMetrics ()->SetMtu (
                                  ipv6->GetMtu (
                                      ipv6->GetInterfaceForAddress (
                                          Ipv6Address::ConvertFrom (*itv))));
                          }
                      }
                  }
              }
          }
      }
  }

  void LispHelper::CreateAndAggregateLispVersion (Ptr<Node> node, const std::string typeId) const
    {
      NS_LOG_FUNCTION (this << typeId);
      NS_ASSERT (typeId == LispOverIpv6Impl::GetTypeId().GetName() || typeId == LispOverIpv4Impl::GetTypeId().GetName());
      ObjectFactory factory;
      factory.SetTypeId (typeId);
      Ptr<Object> lispProtocol = factory.Create<Object> ();
      node->AggregateObject (lispProtocol);
    }

  // N.B. if the file exists, we assume that it is well formatted
  void
  LispHelper::BuildMapTables (std::string localMapTablesConfigFilePath)
  {

    NS_LOG_INFO (localMapTablesConfigFilePath);
    std::ifstream configFile (localMapTablesConfigFilePath.c_str ());
    std::string str;
    int ipv4Rlocs = 0;
    int ipv6Rlocs = 0;

    // Ipv4 Rlocs first
    if (std::getline (configFile, str))
      ipv4Rlocs = std::atoi (str.c_str ());

    // split line elements into vector
    std::vector<std::string> oneLineElems;
    int ipv4Eids = 0;
    int ipv6Eids = 0;
    for (int j = 0; j < ipv4Rlocs; ++j, ipv4Eids = 0, ipv6Eids = 0)
      {
        Ipv4Address rlocAddress;
        // get the rloc address
        if (std::getline (configFile, str))
          rlocAddress = Ipv4Address (str.c_str ());

        // get ipv4 eids
        if (std::getline (configFile, str))
          ipv4Eids = std::atoi (str.c_str ());

        // Create MapTables
        Ptr<SimpleMapTables> ipv4MapTables = Create<SimpleMapTables> ();
        Ptr<SimpleMapTables> ipv6MapTables = Create<SimpleMapTables> ();
        int eidIpv4Rlocs = 0;
        int eidIpv6Rlocs = 0;
        // insert ipv4 eid map entries
        for (int k = 0; k < ipv4Eids; ++k, eidIpv4Rlocs = 0, eidIpv6Rlocs = 0)
          {
            // get eid ipv4 address and mask
            if (std::getline (configFile, str))
              oneLineElems = Split (str);

            Ipv4Address eidAddress = Ipv4Address (oneLineElems[0].c_str ());
            Ipv4Mask eidMask = Ipv4Mask (oneLineElems[1].c_str ());
            MapTables::MapEntryLocation location = GetLocation(std::atoi (oneLineElems[2].c_str ()));

            // get ipv4 rlocs number for one EID
            if (std::getline (configFile, str))
              eidIpv4Rlocs = std::atoi (str.c_str ());

            int priority = 0, weight = 0, reachability = 0;
            // get each rloc for that eid
            for (int l = 0; l < eidIpv4Rlocs;
                ++l, priority = 0, weight = 0, reachability = 0)
              {
                // get ipv4 rloc
                if (std::getline (configFile, str))
                  oneLineElems = Split (str);

                // get rlocs ipv4 address
                Ipv4Address mappingRlocAddress = Ipv4Address (
                    oneLineElems[0].c_str ());
                // get metrics
                priority = std::atoi (oneLineElems[1].c_str ());
                weight = std::atoi (oneLineElems[2].c_str ());
                reachability = std::atoi (oneLineElems[3].c_str ());

                ipv4MapTables->InsertLocator (eidAddress, eidMask,
                                              mappingRlocAddress, priority,
                                              weight, location, reachability);
              }

            // get number of ipv6 rlocs for one EID
            if (std::getline (configFile, str))
              eidIpv6Rlocs = std::atoi (str.c_str ());

            for (int l = 0; l < eidIpv6Rlocs;
                ++l, priority = 0, weight = 0/*, reachability = 0*/)
              {
                // get ipv6 rloc
                if (std::getline (configFile, str))
                  oneLineElems = Split (str);

                // get rlocs ipv6 address
                Ipv6Address mappingRlocAddress = Ipv6Address (
                    oneLineElems[0].c_str ());
                // get metrics
                priority = std::atoi (oneLineElems[1].c_str ());
                weight = std::atoi (oneLineElems[2].c_str ());
                reachability = std::atoi (oneLineElems[3].c_str ());
                ipv4MapTables->InsertLocator (eidAddress, eidMask,
                                              mappingRlocAddress, priority,
                                              weight, location, reachability);
              }
          }

        // get number of ipv6 eids for the rloc router table
        if (std::getline (configFile, str))
          ipv6Eids = std::atoi (str.c_str ());

        // insert ipv6 eid map entries
        for (int k = 0; k < ipv6Eids; ++k, eidIpv4Rlocs = 0, eidIpv6Rlocs = 0)
          {
            // get eid ipv4 address and mask
            if (std::getline (configFile, str))
              oneLineElems = Split (str);

            Ipv6Address eidAddress = Ipv6Address (oneLineElems[0].c_str ());
            Ipv6Prefix eidPrefix = Ipv6Prefix (oneLineElems[1].c_str ());
            MapTables::MapEntryLocation location = GetLocation(std::atoi (oneLineElems[2].c_str ()));

            // get ipv4 rlocs number
            if (std::getline (configFile, str))
              eidIpv4Rlocs = std::atoi (str.c_str ());

            int priority = 0, weight = 0, reachability = 0;
            // get ipv4 Rlocs
            for (int l = 0; l < eidIpv4Rlocs;
                ++l, priority = 0, weight = 0/*,reachability = 0*/)
              {
                // get ipv4 rloc
                if (std::getline (configFile, str))
                  oneLineElems = Split (str);

                // get rlocs ipv4 address
                Ipv4Address mappingRlocAddress = Ipv4Address (
                    oneLineElems[0].c_str ());
                // get metrics
                priority = std::atoi (oneLineElems[1].c_str ());
                weight = std::atoi (oneLineElems[2].c_str ());
                reachability = std::atoi (oneLineElems[3].c_str ());

                ipv6MapTables->InsertLocator (eidAddress, eidPrefix,
                                              mappingRlocAddress, priority,
                                              weight, location, reachability);
              }

            // get number of ipv6 rlocs
            if (std::getline (configFile, str))
              eidIpv6Rlocs = std::atoi (str.c_str ());

            for (int l = 0; l < eidIpv6Rlocs;
                ++l, priority = 0, weight = 0, reachability = 0)
              {
                // get ipv6 rloc
                if (std::getline (configFile, str))
                  oneLineElems = Split (str);

                // get rlocs ipv6 address
                Ipv6Address mappingRlocAddress = Ipv6Address (
                    oneLineElems[0].c_str ());
                // get metrics
                priority = std::atoi (oneLineElems[1].c_str ());
                weight = std::atoi (oneLineElems[2].c_str ());
                reachability = std::atoi (oneLineElems[3].c_str ());
                ipv6MapTables->InsertLocator (eidAddress, eidPrefix,
                                              mappingRlocAddress, priority,
                                              weight, location, reachability);
              }
          }
        m_mapTablesIpv4.insert (std::pair<Address, Ptr<MapTables> > (rlocAddress, ipv4MapTables));
        m_mapTablesIpv6.insert (std::pair<Address, Ptr<MapTables> > (rlocAddress, ipv6MapTables));
      }

    // Ipv6 Rlocs
    if (std::getline (configFile, str))
      ipv6Rlocs = std::atoi (str.c_str ());

    for (int j = 0; j < ipv6Rlocs; ++j, ipv4Eids = 0, ipv6Eids = 0)
      {
        // TODO Finish IPV6 RLOCS
      }
  }

  void LispHelper::BuildMapTables2 (std::string localMapTablesConfigFilePath)
  {
    // locators
    std::string LOCS_START = "<locators>";
    std::string LOCS_END = "</locators>";
    // locator
    std::string LOC_START = "<locator>";
    std::string LOC_END = "</locator>";
    // router if address
    std::string IF_ADDR4_START = "<if-address-v4>";
    std::string IF_ADDR4_END = "</if-address-v4>";
    std::string IF_ADDR6_START = "<if-address-v6>";
    std::string IF_ADDR6_END = "</if-address-v6>";
    // entry
    std::string ENTRY_START = "<entry>";
    std::string ENTRY_END = "</entry>";
    // eid
    std::string EID4_START = "<eid-v4>";
    std::string EID4_END = "</eid-v4>";
    std::string EID6_START = "<eid-v4>";
    std::string EID6_END = "</eid-v4>";
    // rloc
    std::string RLOC4_START = "<rloc-v4>";
    std::string RLOC4_END = "</rloc-v4>";
    std::string RLOC6_START = "<rloc-v6>";
    std::string RLOC6_END = "</rloc-v6>";

    NS_LOG_INFO(localMapTablesConfigFilePath);
    std::ifstream configFile (localMapTablesConfigFilePath.c_str ());
    std::string str;

    bool endFile = false;
    bool inLocator = false;
    if (!endFile && std::getline(configFile, str) && str == LOCS_START)
      {
        Address locatorIfAddress;
        while (!endFile && std::getline (configFile, str))
          {
              if (str == LOCS_END) {
                  NS_ASSERT(!endFile);
                endFile = true;
              }
              else if (!endFile && str == LOC_START) {
                  NS_ASSERT_MSG (!inLocator, "ERROR ON LINE :" << str);
                  inLocator = true;
              }
              else if (!endFile && str == LOC_END)
                {
                NS_ASSERT(inLocator);
                inLocator = false;
                }
              else if (inLocator && (str.find (IF_ADDR4_START) == 0 || str.find (IF_ADDR6_START) == 0)) // && str.find (IF_ADDR4_START) == 0
                {
                std::vector<std::string> vect = Split (str);
                NS_LOG_DEBUG (vect.size () << " " << str);
                NS_ASSERT(vect.size () == 3 && ((vect[0] == IF_ADDR4_START && vect[2] == IF_ADDR4_END) || (vect[0] == IF_ADDR6_START && vect[2] == IF_ADDR6_END)));
                if (vect[0] == IF_ADDR4_START)
                  locatorIfAddress = static_cast<Address> (Ipv4Address (
                    vect[1].c_str ()));
                else locatorIfAddress = static_cast<Address> (Ipv6Address (
                    vect[1].c_str ()));
                // creating mapTables
                Ptr<SimpleMapTables> ipv4MapTables = Create<SimpleMapTables> ();
                Ptr<SimpleMapTables> ipv6MapTables = Create<SimpleMapTables> ();
                // start encoding one ENTRY
                while (std::getline (configFile, str))
                  {
                    if (str == LOC_END)
                      {
                        inLocator = false;
                        break;
                      }

                    if (str != ENTRY_START)
                      break;
                    // we read the eid address and the rlocs further
                    if (std::getline (configFile, str))
                      {
                        vect = Split (str);
                        NS_ASSERT_MSG(vect.size () == 5,
                                      "Bad File Format --- see <eid-vX>");
                        if (vect[0] == EID4_START && vect[4] == EID4_END)
                          {
                            Ipv4Address eidAddress = Ipv4Address (
                                vect[1].c_str());
                            Ipv4Mask eidMask = Ipv4Mask (vect[2].c_str ());
                            MapTables::MapEntryLocation location = GetLocation (
                                std::atoi (vect[3].c_str ()));
                            // we read rlocs + prio +weight ... until we read </entry>
                            while (std::getline (configFile, str))
                              {
                                if (str == ENTRY_END)
                                  break;
                                vect = Split (str);
                                NS_ASSERT_MSG(
                                    vect.size () == 6,
                                    "Bad File Format --- see <rloc-vX> --- ERROR ON LINE: " << str);
                                if (vect[0] == RLOC4_START && vect[5] == RLOC4_END)
                                  {
                                    Ipv4Address rlocAddress = Ipv4Address (vect[1].c_str ());
                                    int priority = std::atoi (vect[2].c_str ());
                                    int weight = std::atoi (vect[3].c_str ());
                                    int reachability = std::atoi (vect[4].c_str ());
                                    ipv4MapTables->InsertLocator (eidAddress, eidMask,
                                                                                  rlocAddress, (uint8_t) priority,
                                                                                  (uint8_t) weight, location, (bool) reachability);
                                  }
                                else if (vect[0] == RLOC6_START && vect[5] == RLOC6_END)
                                  {
                                    // TODO V6 RLOC
                                    Ipv6Address rlocAddress = Ipv6Address (
                                        vect[1].c_str ());
                                    int priority = std::atoi (vect[2].c_str ());
                                    int weight = std::atoi (vect[3].c_str ());
                                    int reachability = std::atoi (
                                        vect[4].c_str ());
                                    ipv4MapTables->InsertLocator (eidAddress,
                                                                  eidMask,
                                                                  rlocAddress,
                                                                  (uint8_t) priority,
                                                                  (uint8_t) weight,
                                                                  location,
                                                                  (bool) reachability);
                                  }
                                else
                                  {
                                    // ERROR
                                    NS_LOG_ERROR("Bad File Format!");
                                    exit (-1);
                                  }
                              }
                          }
                        else if (vect[0] == EID6_START && vect[4] == EID6_END)
                          {
                            // TODO V6 EID
                          }
                        else
                          {
                            // ERROR
                            NS_LOG_ERROR("Bad File Format2");
                            exit (-1);
                          }
                      }
                  }
                m_mapTablesIpv4.insert (std::pair<Address, Ptr<MapTables> > (locatorIfAddress, ipv4MapTables));
                m_mapTablesIpv6.insert (std::pair<Address, Ptr<MapTables> > (locatorIfAddress, ipv6MapTables));
                m_lispStatisticsMapForV4.insert (std::pair<Address, Ptr<LispStatistics> > (locatorIfAddress, Create<LispStatistics> ()));
                m_lispStatisticsMapForV6.insert (std::pair<Address, Ptr<LispStatistics> > (locatorIfAddress, Create<LispStatistics> ()));
              }
              else
                {
                  // Error
                  NS_LOG_ERROR("ERROR " << str);
                  //exit (-1);
                }
          }
      }
    else
      {
        // ERROR
        NS_LOG_ERROR("Bad File Format --- File start with <locators> and ends with  </locators>");
        exit (-1);
      }
  }

  std::vector<std::string> LispHelper::Split(std::string str)
  {
    char delimiter = '\t';
    std::vector<std::string> internal;
    std::stringstream ss(str); // Turn the string into a stream.
    std::string tok;

    while(getline(ss, tok, delimiter)) {
      internal.push_back(tok);
    }
    return internal;
  }

  MapTables::MapEntryLocation LispHelper::GetLocation (int locationInt)
  {
    if (!locationInt)
      return MapTables::IN_DATABASE;
    else return MapTables::IN_CACHE;
  }

void LispHelper::BuildRlocsSet (std::string rlocsListFilePath)
{

  NS_LOG_INFO(rlocsListFilePath);
  std::ifstream rlocsFile (rlocsListFilePath.c_str ());
  std::string str;

  std::vector<std::string> vect;
  while (std::getline (rlocsFile, str))
    {
      vect = Split (str);
      if (std::atoi(vect[0].c_str ()) == 1)
        {
          m_rlocsList.insert (static_cast<Address> (Ipv4Address (vect[1].c_str ())));
        }
      else if (std::atoi(vect[0].c_str ()) == 2)
        {
          m_rlocsList.insert (static_cast<Address> (Ipv6Address (vect[1].c_str ())));
        }
    }
}


} /* namespace ns3 */
