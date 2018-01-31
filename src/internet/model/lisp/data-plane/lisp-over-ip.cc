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

#include <stdint.h>
#include "lisp-over-ip.h"
#include "ns3/node.h"
#include <ns3/log.h>
#include "ns3/object.h"
#include "ns3/object-vector.h"
#include "lisp-mapping-socket-factory.h"
#include "lisp-mapping-socket.h"
#include "simple-map-tables.h"

namespace ns3
{

  NS_LOG_COMPONENT_DEFINE("LispOverIp");

  NS_OBJECT_ENSURE_REGISTERED(LispOverIp);

  // LispProtocol
  const uint8_t LispOverIp::PROT_NUMBER = 143 ;// first non assigned (adhoc number)
  const uint16_t LispOverIp::LISP_DATA_PORT = 4341;
  const uint16_t LispOverIp::LISP_SIG_PORT = 4342;
  const uint16_t LispOverIp::LISP_MAX_RLOC_PRIO = 255;
  const uint16_t LispOverIp::MAX_VERSION_NUM = 4095;
  const uint16_t LispOverIp::WRAP_VERSION_NUM = 2047;
  const uint8_t LispOverIp::NULL_VERSION_NUM = 0;

  TypeId
  LispOverIp::GetTypeId ()
  {
    static TypeId tid =
	TypeId ("ns3::LispOverIp").SetParent<Object> ().SetGroupName ("Lisp").AddAttribute (
	    "SocketList",
	    "The list of sockets associated to this protocol (lisp).",
	    ObjectVectorValue (),
	    MakeObjectVectorAccessor (&LispOverIp::m_sockets),
	    MakeObjectVectorChecker<LispMappingSocket> ());
    return tid;
  }

  LispOverIp::LispOverIp (Ptr<LispStatistics> statisticsForIpv4,
			  Ptr<LispStatistics> statisticsForIpv6)
  {
    NS_LOG_FUNCTION(this);
    NS_ASSERT(statisticsForIpv4 && statisticsForIpv6);
    m_statisticsForIpv4 = statisticsForIpv4;
    m_statisticsForIpv6 = statisticsForIpv6;
  }

  void
  LispOverIp::SetLispStatistics (Ptr<LispStatistics> statisticsV4,
				 Ptr<LispStatistics> statisticsV6)
  {
    m_statisticsForIpv4 = statisticsV4;
    m_statisticsForIpv6 = statisticsV6;
  }

  LispOverIp::LispOverIp ()
  {
    /**
     * Qipeng gives some explanation:
     * 2017-04-27
     * When LispOverIp is instantialized, we note that CreateSocket method has been
     * invoked one time and m_lispSocket is returned.
     *
     * When LispEtrItrApplicaiton startApplication method is called, lispMappingSocket
     * factory will be called, in which CreateSocket method in this class will be
     * called.
     *
     * That's why m_sockets.size increases.
     *
     * 2017-04-28
     * When LispOverIP is instantized, the attribute m_mapTablesIpv6 is 0 until
     * lispHelper set map table for LispOverIp object. We rely this to judge if
     * in Ipv4L3Protocol, lisp-related code will be executed.
     */
    NS_LOG_FUNCTION(this<<"LispOverIp constructor is called, the m_sockets size is: "<<m_sockets.size());
    m_lispSocket = CreateSocket ();
    // Above instruction make m_lispSocket as the first element (index 0)
    // saved in vector m_sockets. Which place LispOverIp create the second one?

  }

  LispOverIp::~LispOverIp ()
  {
    NS_LOG_FUNCTION(this);
  }

  Ptr<Node>
  LispOverIp::GetNode ()
  {
    return m_node;
  }

  void
  LispOverIp::SetNode (Ptr<Node> node)
  {
    m_node = node;
  }

  void
  LispOverIp::NotifyNewAggregate ()
  {
    NS_LOG_FUNCTION(this);
    if (m_node == 0)
      {
	Ptr<Node> node = this->GetObject<Node> ();
	// verify that it's a valid node and that
	// the node has not been set before
	if (node != 0)
	  {
	    this->SetNode (node);
	    if (!node->GetObject<LispMappingSocketFactory> ())
	      {
		Ptr<LispMappingSocketFactory> mappingFactory = CreateObject<
		    LispMappingSocketFactory> ();
		mappingFactory->SetLisp (this);
		node->AggregateObject (mappingFactory);
	      }
	  }
      }
    Object::NotifyNewAggregate ();
  }

  void
  LispOverIp::DoDispose (void)
  {

  }

  Ptr<Socket>
  LispOverIp::CreateSocket (void)
  {
    NS_LOG_FUNCTION_NOARGS ();
    Ptr<LispMappingSocket> socket = CreateObject<LispMappingSocket> ();
    socket->SetNode (m_node);
    socket->SetLisp (this);
    socket->SetSockIndex (m_sockets.size ());
    m_sockets.push_back (socket);
    return socket;
  }

  Ptr<LispMappingSocket>
  LispOverIp::GetMappingSocket (uint8_t sockIndex)
  {
    return m_sockets.at (sockIndex);
  }

  void
  LispOverIp::SetMapTablesIpv4 (Ptr<MapTables> mapTablesIpv4)
  {
    NS_ASSERT(mapTablesIpv4 != 0);
    m_mapTablesIpv4 = mapTablesIpv4;
    m_mapTablesIpv4->SetLispOverIp (this);
  }
  void
  LispOverIp::SetMapTablesIpv6 (Ptr<MapTables> mapTablesIpv6)
  {
    NS_ASSERT(mapTablesIpv6 != 0);
    m_mapTablesIpv6 = mapTablesIpv6;
    m_mapTablesIpv6->SetLispOverIp (this);
  }

  void LispOverIp::Print (std::ostream &os) const
  {
    os<<"For Ipv4"<<std::endl;
    m_mapTablesIpv4->Print(os);
  }

  Ptr<MapEntry>
  LispOverIp::DatabaseLookup (Address const &eidAddress) const
  {
    if (Ipv4Address::IsMatchingType (eidAddress))
      {
	return m_mapTablesIpv4->DatabaseLookup (eidAddress);
      }
    else if (Ipv6Address::IsMatchingType (eidAddress))
      {
	return m_mapTablesIpv6->DatabaseLookup (eidAddress);
      }
    return 0;
  }

  Ptr<MapEntry>
  LispOverIp::CacheLookup (Address const &eidAddress) const
  {
    if (Ipv4Address::IsMatchingType (eidAddress))
      {
	return m_mapTablesIpv4->CacheLookup (eidAddress);
      }
    else if (Ipv6Address::IsMatchingType (eidAddress))
      {
	return m_mapTablesIpv6->CacheLookup (eidAddress);
      }
    return 0;
  }

  Ptr<Locator>
  LispOverIp::SelectDestinationRloc (Ptr<const MapEntry> mapEntry) const
  {
    return mapEntry->RlocSelection ();
  }

  Ptr<Locator>
  LispOverIp::SelectSourceRloc (Address const &srcEid,
				Ptr<const Locator> destLocator) const
  {
    NS_LOG_FUNCTION(this);
    //Default value is 0
    Ptr<Locator> srcRloc;
    if (Ipv4Address::IsMatchingType (srcEid))
      {
	NS_LOG_DEBUG(
	    "Select Source RLOC for EID: "<<Ipv4Address::ConvertFrom(srcEid)<<" Given that destination Locator is:"<<destLocator->GetRlocAddress());
	srcRloc = m_mapTablesIpv4->SourceRlocSelection (srcEid, destLocator);
      }
    else if (Ipv6Address::IsMatchingType (srcEid))
      {
	srcRloc = m_mapTablesIpv6->SourceRlocSelection (srcEid, destLocator);
      }
    if(srcRloc != 0)
      {
	  NS_LOG_DEBUG("The selected source RLOC address: "<<srcRloc->GetRlocAddress());
      }
    else
      {
	NS_LOG_ERROR("No valid source RLOC is found!");
	NS_LOG_DEBUG("Currently, the mapping database's content: "<<*m_mapTablesIpv4);
      }
    return srcRloc;
  }

  Ptr<LispStatistics>
  LispOverIp::GetLispStatisticsV4 (void)
  {
    return m_statisticsForIpv4;
  }

  Ptr<LispStatistics>
  LispOverIp::GetLispStatisticsV6 (void)
  {
    return m_statisticsForIpv6;
  }

  void
  LispOverIp::OpenLispMappingSocket (void)
  {
    NS_LOG_FUNCTION(this);
    // create mapping socket address
    m_lispAddress = static_cast<Address> (MappingSocketAddress (
	m_node->GetDevice (0)->GetAddress (), 0));
    NS_LOG_DEBUG(
	"LispOverIp open and bind a mapping socket at address: "<<m_lispAddress<<"whose length is: "<<unsigned(m_lispAddress.GetLength()));
    // bind
    m_lispSocket->Bind (m_lispAddress);
    m_lispSocket->SetRecvCallback (
	MakeCallback (&LispOverIp::HandleMapSockRead, this));
    NS_LOG_DEBUG("Bind to " << m_lispAddress);
  }

  void
  LispOverIp::HandleMapSockRead (Ptr<Socket> socket)
  {
    NS_LOG_FUNCTION(this);
    Ptr<Packet> packet;
    Address from;

    while ((packet = socket->RecvFrom (from)))
      {
	MappingSocketMsgHeader sockMsgHdr;
	packet->RemoveHeader (sockMsgHdr);
	uint8_t buf[packet->GetSize ()];
	packet->CopyData (buf, packet->GetSize ());
	Ptr<MappingSocketMsg> msg = MappingSocketMsg::Deserialize (buf);
	NS_LOG_DEBUG("MSG HEADER " << sockMsgHdr);
	if (sockMsgHdr.GetMapType ()
	    == static_cast<uint16_t> (LispMappingSocket::MAPM_ADD))
	  {
	    //TODO: extract the following code and write it as an function
	    // Both MAPM_ADD and MAPM_UPDATE will use the same code.
	    NS_LOG_DEBUG(
		"ADD Message received on lisp (" << msg->GetEndPointId()->Print () << ") \n" << msg->GetLocators ()->Print ());
	    Ptr<EndpointId> eid = msg->GetEndPointId ();
	    Ptr<MapEntry> mapEntry = Create<MapEntryImpl> ();
	    Ptr<Locators> locators;
	    mapEntry->SetEidPrefix (eid);
	    if ((int) sockMsgHdr.GetMapFlags ()
		& (int) LispMappingSocket::MAPF_NEGATIVE)
	      {
		NS_LOG_DEBUG("MAP ENTRY is negative!");
		mapEntry->setIsNegative (1);
	      }
	    else
	      {
		NS_LOG_DEBUG("Setting Is Local If to 0!");
		locators = msg->GetLocators ();
		for (int i = 0; i < locators->GetNLocators (); ++i)
		  {
		    Ptr<Locator> locator = locators->GetLocatorByIdx (i);
		    locator->GetRlocMetrics ()->SetIsLocalIf (0);
		  }
		mapEntry->setIsNegative (0);
		mapEntry->SetLocators (locators);
	      }
	    if (eid->IsIpv4 ())
	      {
		// In the case of cache update, we should first delete the previous one containing EID-prefix
		// This work is done by SetEntry method!
		m_mapTablesIpv4->SetEntry (eid->GetEidAddress (),
					   eid->GetIpv4Mask (), mapEntry,
					   MapTables::IN_CACHE);
		NS_LOG_DEBUG(
		    "Ipv4 Map Entry IPv4 (Extracted from Map Reply Message) has been saved in cache database by LispOverIp");
	      }
	    else
	      {
		m_mapTablesIpv6->SetEntry (eid->GetEidAddress (),
					   eid->GetIpv6Prefix (), mapEntry,
					   MapTables::IN_CACHE);
		NS_LOG_DEBUG(
		    "Ipv4 Map Entry IPv6 (Extracted from Map Reply Message) has been saved in cache database by LispOverIp");
	      }
	  }
	else if (sockMsgHdr.GetMapType ()
	    == static_cast<uint16_t> (LispMappingSocket::MAPM_DELETE))
	  {

	  }
	else if (sockMsgHdr.GetMapType ()
	    == static_cast<uint16_t> (LispMappingSocket::MAPM_GET))
	  {

	  }
	else if (sockMsgHdr.GetMapType ()== static_cast<uint16_t> (LispMappingSocket::MAPM_DATABASE_UPDATE))
	  {
	    /**
	     * TODO: Notify to xTR to send a new Map-Register message.
	     * In this case, first check whether the received EID is in the database.
	     * If yes => update the map entry
	     * If no => add the map entry.
	     * Maybe we can use another solution:
	     * 1) if DHCP client detect link state change
	     * (e.g. during mobility, the wifi link is temporarily lost), DHCP has no
	     * IP address (RLOC). Trigger a MAPM_DELETE message to flush LISP-MN database
	     * 10-07-2017: The answer is no. Since flush LISP-MN database cannot update map version number!
	     * (how to treat Cache?=> Never touch Cache! Since it is the only place we
	     * know to which it communicate!! if cache is not flushed. It is OK if we have
	     * cache but no database? or database is empty?)
	     */
	    NS_LOG_DEBUG(
		"Update or Add one mapping (" << msg->GetEndPointId()->Print () << ") \n" << msg->GetLocators ()->Print ());
	    Ptr<EndpointId> eid = msg->GetEndPointId ();
	    Ptr<MapEntry> mapEntry = Create<MapEntryImpl> ();
	    Ptr<Locators> locators = msg->GetLocators();
	    mapEntry->SetEidPrefix (eid);
	    if ((int) sockMsgHdr.GetMapFlags ()
		& (int) LispMappingSocket::MAPF_NEGATIVE)
	      {
		NS_LOG_DEBUG("MAP ENTRY is negative!");
		mapEntry->setIsNegative (1);
	      }
	    else
	      {
		mapEntry->setIsNegative (0);
		mapEntry->SetLocators (locators);
	      }
	    if (eid->IsIpv4 ())
	      {
		//TODO: to verify if map data structure supports add (or update) manipulation.
		// This is important cause DHCP will periodically received offered @IP.
		// If two different RLOCs, how to treat it?
	    // 07-10-2017: DHCP client should guarantee that the newly assigned @IP is different from previous one
 		// In the case of cache update, we should first delete the previous one containing EID-prefix
		// This work is done by SetEntry method!
		m_mapTablesIpv4->SetEntry (eid->GetEidAddress (),
					   eid->GetIpv4Mask (), mapEntry,
					   MapTables::IN_DATABASE);
		NS_LOG_DEBUG(
		    "Ipv4 Map Entry IPv4 (Received from DHCP client) has been saved in database by LispOverIp");
		NS_LOG_DEBUG("After message from DHCP to LISP, LISP Database now: \n"<<*(LispOverIp::GetMapTablesV4()));
	      }
	    else
	      {
		m_mapTablesIpv6->SetEntry (eid->GetEidAddress (),
					   eid->GetIpv6Prefix (), mapEntry,
					   MapTables::IN_DATABASE);
		NS_LOG_DEBUG(
		    "Ipv4 Map Entry IPv6 (Received from DHCP client) has been saved in database by LispOverIp");
	      }
	    /**
	     * IMPORTANT: To support LISP-MN.
	     * 1) Send a signal (MAPM_REGISTER) to xTR application so that xTR send map register again
	     * 2) Delete the previously assigned RLOC IP address (by DHCP server) in m_rlocsList attribute
	     * of lispOverIpv4 object. This attribute is previously set by lispHelper class.
	     * 3) Add the newly obtained RLOC IP address into m_rlocsList
	     *
	     * TODO: Current implementation does not support empty message content.
	     * Otherwise error occurs. Should consider this!
	     */
	    uint8_t buf[256];
	    Ptr<MappingSocketMsg> mapSockMsg = Create<MappingSocketMsg> ();
	    MappingSocketMsgHeader mapSockHeader;
	    mapSockMsg->SetEndPoint (eid);
	    mapSockMsg->SetLocators (msg->GetLocators());
	    mapSockMsg->Serialize (buf);
	    mapSockHeader.SetMapType (LispMappingSocket::MAPM_REGISTER);
	    Ptr<Packet> packet = Create<Packet> (buf, 100);
	    uint8_t messageType = static_cast<uint8_t> (LispMappingSocket::MAPM_REGISTER);


	    // Normally the received EID-RLOC mapping contains just one RLOC.
	    // If more than one RLOCs, it is anormal!! First delete the RLOC in
	    // m_rlocsList then add the newly assigned RLOC into it.
	    NS_ASSERT(msg->GetLocators()->GetNLocators()==1);

	    // Is necessary to delete the previously RLOC? What's the hurt?
	    // What if EID has two RLOCs assigned? which one to delete?
//	    Ptr<MapEntry> curEidMapEntry = LispOverIp::DatabaseLookup(eid->GetEidAddress());
//	    Address curRlocAddr = curEidMapEntry->GetLocators()->GetLocatorByIdx(0)->GetRlocAddress();
//	    m_rlocsList.erase(curRlocAddr);
	    Address rlocAddr = msg->GetLocators()->GetLocatorByIdx(0)->GetRlocAddress();
	    m_rlocsList.insert(rlocAddr);
	    for(std::set<Address>::const_iterator it=m_rlocsList.begin();it != m_rlocsList.end (); ++it)
	    {
		if(Ipv4Address::IsMatchingType(*it))
		  {
		    NS_LOG_DEBUG("RLOC list item: "<<Ipv4Address::ConvertFrom(*it));
		  }
		else if(Ipv6Address::IsMatchingType(*it))
		  {
		    NS_LOG_DEBUG("RLOC list item: "<<Ipv6Address::ConvertFrom(*it));
		  }
		else
		  {
		    NS_LOG_ERROR("Unknown Address Type...");
		  }
	    }

	    // First populate m_rlocList then send map register message. Otherwise
	    // Lisp data plan will try to find a RLOC for the real RLOC...
	    NS_LOG_DEBUG("Notify xTR to send map register message.");
	    LispOverIp::SendNotifyMessage (messageType, packet, mapSockHeader,0);
	  }

      }
  }

  Address
  LispOverIp::GetLispMapSockAddress (void)
  {
    return m_lispAddress;
  }

  void
  LispOverIp::SendNotifyMessage (uint8_t messageType, Ptr<Packet> packet,
				 MappingSocketMsgHeader mapSockMsgHeader,
				 int flags)
  {
    // Notify message is sent to every application
    NS_LOG_FUNCTION(this<<"Message Type: "<<unsigned(messageType));

    switch (messageType)
      {
      case (static_cast<uint8_t> (LispMappingSocket::MAPM_MISS)):
	switch (LispMappingSocket::m_lispMissMsgType)
	  {
	  case (LispMappingSocket::LISP_MISSMSG_EID):
	    break;
	  default:
	    break;
	  }
	break;
      case (static_cast<uint8_t> (LispMappingSocket::MAPM_DELETE)):
	break;
      default:
	break;
      }
    mapSockMsgHeader.SetMapFlags (flags | LispMappingSocket::MAPF_DONE);
    packet->AddHeader (mapSockMsgHeader);
    NS_LOG_DEBUG("Send Messages from data plan to control plan: " << m_sockets.size ()-1<<" sockets.");
    for (uint32_t i = 1; i < m_sockets.size (); ++i)
      {
	Address ad = static_cast<Address> (MappingSocketAddress ());
	m_sockets.at (i)->GetSockName (ad);
	m_lispSocket->Connect (ad);
	// Lionel initially use Socket's Send method, which always send packet
	// to m_lispSocket with index 1. Now this bug is fixed to support
	// communication between LispOverIp and DHCP server (surely DHCP client)
	m_lispSocket->SendTo (packet, flags, ad);
	NS_LOG_DEBUG(
	    "Send Notification: "<< MappingSocketAddress::ConvertFrom(m_lispAddress) <<"-->"<< MappingSocketAddress::ConvertFrom (ad));
	NS_LOG_DEBUG("Sent Notification content is: " << *packet);
      }
  }

  void
  LispOverIp::SetRlocsList (const std::set<Address> rlocsList)
  {
    m_rlocsList = rlocsList;
  }

  std::set<Address>
  LispOverIp::GetRlocsList (void) const
  {
    return m_rlocsList;
  }

  bool
  LispOverIp::IsLocatorInList (Address address) const
  {

    for (std::set<Address>::const_iterator it = m_rlocsList.begin ();
	it != m_rlocsList.end (); ++it)
      {
	if (Ipv4Address::IsMatchingType (address)
	    && Ipv4Address::IsMatchingType (*it)
	    && Ipv4Address::ConvertFrom (address).IsEqual (
		Ipv4Address::ConvertFrom (*it)))
	  return true;
	else if (Ipv6Address::IsMatchingType (address)
	    && Ipv6Address::IsMatchingType (*it)
	    && Ipv6Address::ConvertFrom (address).IsEqual (
		Ipv6Address::ConvertFrom (*it)))
	  return true;
	else
	  continue;
      }
    return false;
  }

  Ptr<MapTables>
  LispOverIp::GetMapTablesV4 (void) const
  {
    return m_mapTablesIpv4;
  }

  Ptr<MapTables>
  LispOverIp::GetMapTablesV6 (void) const
  {
    return m_mapTablesIpv6;
  }

  Ptr<Packet>
  LispOverIp::PrependLispHeader (Ptr<Packet> packet,
                                     Ptr<const MapEntry> localMapEntry,
                                     Ptr<const MapEntry> remoteMapEntry,
                                     Ptr<Locator> sourceRloc,
                                     Ptr<Locator> destRloc)
  {
    NS_ASSERT (packet);
    LispHeader lispHeader = LispHeader ();

    // Check if txNonce present
    if (destRloc->GetRlocMetrics ()->IsTxNoncePresent())
      {
        // if yes set N bit
        lispHeader.SetNBit(1);
        // copy nonce
        lispHeader.SetNonce(destRloc->GetRlocMetrics ()->GetTxNonce());
      }

    // check if local and remote mapping use versioning
    if (localMapEntry->IsUsingVersioning () && remoteMapEntry->IsUsingVersioning ())
      {
        lispHeader.SetVBit(1);
        lispHeader.SetDestMapVersion (remoteMapEntry->GetVersionNumber());
        lispHeader.SetSrcMapVersion (localMapEntry->GetVersionNumber());
      }
    else if (localMapEntry->IsUsingLocStatusBits ())
      {
        lispHeader.SetLBit (1);
        lispHeader.SetLSBs (localMapEntry->GetLocsStatusBits ());
      }

    packet->AddHeader (lispHeader);
    NS_LOG_DEBUG ("Lisp Header Added: " << lispHeader);
    return packet;
  }

  bool
  LispOverIp::CheckLispHeader (const LispHeader &header,
                                   Ptr<const MapEntry> localMapEntry,
                                   Ptr<const MapEntry> remoteMapEntry,
                                   Ptr<Locator> srcRloc, Ptr<Locator> destRloc,
                                   Ptr<LispOverIp> lispOverIp)
  {
    /*
     * TODO Make enum for message types (0, 1 = dest number older, 2 = src older,
     * 3 = the received status bits is different from the ones in the cache
     *
     *
     */
    Ptr<LispStatistics> stats;

    if (Ipv4Address::IsMatchingType (destRloc->GetRlocAddress ()))
      stats = lispOverIp->GetLispStatisticsV4 ();
    else
      stats = lispOverIp->GetLispStatisticsV6 ();

    int msgType = 0;
    bool retValue = 1;

    if (header.GetNBit ())
      {
        // the LISP header contains a nonce
        if (remoteMapEntry && destRloc)
          {
            /*
             * we copy the last received nonce in the locator
             */
            destRloc->GetRlocMetrics ()->SetRxNoncePresent (true);
            destRloc->GetRlocMetrics ()->SetRxNonce (header.GetNonce ());
          }
      }
    else if (header.GetVBit ()) // note: N bit and V bit cannot be set in header
      {
        // See RFC 6834 for more info about Map-version number
        if (localMapEntry->IsUsingVersioning () && header.GetDestMapVersion () != localMapEntry->GetVersionNumber ())
          {
            // TODO Question Why Does OpenLisp mask the version number
            if (IsMapVersionNumberNewer(localMapEntry->GetVersionNumber (), header.GetDestMapVersion ()))
              {
                /*
                 * The dest version num is newer than the one of
                 * the local map.
                 * This should not happen (because system is
                 * authoritative on the mapping) TODO Why
                 */
                stats->BadDestVersionNumber ();
                retValue = false;
              }
            else
              {
                /*
                 * The dest version number is older
                 * than the one in the local map.
                 * TODO Notify the control plane ! Why
                 *
                 */
                msgType = LispMappingSocket::MAPM_REMOTESTALE;
              }
          }
        else if (remoteMapEntry && remoteMapEntry->IsUsingVersioning () && header.GetSrcMapVersion () != remoteMapEntry->GetVersionNumber ())
          {
            if (IsMapVersionNumberNewer(remoteMapEntry->GetVersionNumber (), header.GetSrcMapVersion ()))
              {
                /*
                 * The src version number is newer than the one in the
                 * Cache
                 * Notify control plane! Why?
                 */

                msgType = LispMappingSocket::MAPM_LOCALSTALE;
              }
            else
              {
                /*
                 * The src version number is older than the one in the
                 * Cache
                 * Todo Erro why ?
                 */
                stats->BadSrcVersionNumber ();
                retValue = false;
              }
          }
      }
    if (!msgType)
      {
        if (header.GetLBit ())
          {
            /*
             * If the Loc Status Bit is set and
             * the LSB field is different from the one
             * in the Cache, notify the Control plane
             */

            if (remoteMapEntry && destRloc && header.GetLSBs () != remoteMapEntry->GetLocsStatusBits ())
              {
                msgType = LispMappingSocket::MAPM_LSBITS;
              }
          }
        else if (header.GetIBit ())
          {
            /*
             * Ignore Ibit.
             * Do nothing for now
             * TODO Consider Instance ID later
             */
          }
      }

    if (msgType)
      {
        // TODO notify control plane through socket
      }
    return retValue;
  }

  uint16_t LispOverIp::GetLispSrcPort (Ptr<const Packet> packet)
  {
    return LISP_DATA_PORT;
  }

  bool LispOverIp::IsMapVersionNumberNewer (uint16_t vnum2, uint16_t vnum1)
  {
    if ((vnum2 > vnum1 && (vnum2 - vnum1) < LispOverIp::WRAP_VERSION_NUM)
        || (vnum1 > vnum2 && (vnum1 - vnum2) > LispOverIp::WRAP_VERSION_NUM + 1))
      {
        return true;
      }
    else return false;
  }


} /* namespace ns3 */
