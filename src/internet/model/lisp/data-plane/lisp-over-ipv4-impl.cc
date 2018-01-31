/*
 * lisp-protocol.cc
 *
 *  Created on: 28 janv. 2016
 *      Author: lionel
 */

#include "lisp-over-ipv4-impl.h"
#include "lisp-over-ipv6-impl.h"
#include <ns3/packet.h>
#include "ns3/address.h"
#include "ns3/assert.h"
#include "ns3/ipv4-address.h"
#include "ns3/ipv4-header.h"
#include "ns3/log.h"
#include "ns3/udp-header.h"
#include "ns3/udp-l4-protocol.h"
#include "lisp-header.h"
#include "ns3/ptr.h"
#include "simple-map-tables.h"
#include <ns3/ipv4-l3-protocol.h>

#include "lisp-over-ip.h"
#include "lisp-mapping-socket.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE ("LispOverIpv4Impl");

NS_OBJECT_ENSURE_REGISTERED (LispOverIpv4Impl);

TypeId
LispOverIpv4Impl::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::LispOverIpv4Impl")
    .SetParent<LispOverIpv4> ()
    .SetGroupName ("Lisp")
    .AddConstructor<LispOverIpv4Impl> ()
  ;
  return tid;
}

LispOverIpv4Impl::LispOverIpv4Impl ()
{
  NS_LOG_FUNCTION (this);
  // Init MapTables
}

LispOverIpv4Impl::~LispOverIpv4Impl ()
{
  NS_LOG_FUNCTION (this);
}

// TODO Remember to manage error cases
// TODO add argument to say packet must be dropped
// Packets coming from upper layer or from an end-host
// If false, we drop packet in IP, if true OK.
void LispOverIpv4Impl::LispOutput (Ptr<Packet> packet, Ipv4Header const &innerHeader,
                                   Ptr<const MapEntry> localMapping,
                                   Ptr<const MapEntry> remoteMapping,
                                   Ptr<Ipv4Route> lispRoute)
{

  NS_LOG_FUNCTION (this);
  Ptr<Locator> destLocator = 0;
  Ptr<Locator> srcLocator = 0;
  uint16_t udpSrcPort;

  // headers
  Ipv4Header outerHeader;
  uint16_t udpLength = 0;

  NS_ASSERT (localMapping != 0);

  NS_LOG_LOGIC ("Check remote mapping existence");
  if (remoteMapping == 0)
    {
      // TODO drop packet
      m_statisticsForIpv4->IncCacheMissPackets ();
      m_statisticsForIpv4->IncOutputDropPackets ();
      m_statisticsForIpv4->IncOutputPackets();
      NS_LOG_WARN ("No remote mapping for destination EID. Drop");
      return;
    }

  // NB. ns3 does not compute checksum by default

  // Select a destination Rloc if no drop packet
  destLocator = SelectDestinationRloc (remoteMapping);
  NS_LOG_DEBUG("Destination RLOC address: "<<destLocator->GetRlocAddress());

  if (destLocator == 0)
    {
      // TODO add stats and drop packet
      m_statisticsForIpv4->IncNoValidRloc ();
      m_statisticsForIpv4->IncOutputDropPackets ();
      m_statisticsForIpv4->IncOutputPackets();
      NS_LOG_WARN ("No valid destination locator for eid " << innerHeader.GetDestination() << ". Drop!");
      return;
    }

  // Check size for destRloc MTU if needed
  if (destLocator->GetRlocMetrics ()->GetMtu () &&
      (packet->GetSize() + LispHeader ().GetSerializedSize () +
          Ipv4Header ().GetSerializedSize () * 2 +
          UdpHeader ().GetSerializedSize()) >
  destLocator->GetRlocMetrics ()->GetMtu ())
    {
      // drop packet
      // TODO send ICMP message
      int size = (packet->GetSize() + LispHeader ().GetSerializedSize () +
          Ipv4Header ().GetSerializedSize () +
          UdpHeader ().GetSerializedSize());
      m_statisticsForIpv4->IncNoValidMtuPackets ();
      m_statisticsForIpv4->IncOutputDropPackets ();
      m_statisticsForIpv4->IncOutputPackets();
      NS_LOG_DEBUG ("MTU DEST " << destLocator->GetRlocMetrics ()->GetMtu () << " Packet size " << size);
      NS_LOG_ERROR ("[LISP_OUTPUT] Drop! MTU check failed for destination RLOC.");
      return;
    }

    // Get Outgoing interface thanks to RouteOutput of m_routingProtocol
    // If the following 2 checks are not OK, drop
    // check if the 2 Rloc (DEST and SRC) are IPvX
    /*
     * We check the list of the Rlocs and select the one which
     * respects the following constraints:
     * - Has the same AF
     * - Is a local address
     * - Has a valid priority (i.e. less than 255)
     * - Is the address of the outgoing interface
     */
  srcLocator = SelectSourceRloc (static_cast<Address> (innerHeader.GetSource ()), destLocator);

  if (srcLocator == 0)
    {
      //drop packet
      m_statisticsForIpv4->IncNoValidRloc ();
      m_statisticsForIpv4->IncOutputDropPackets ();
      m_statisticsForIpv4->IncOutputPackets();
      NS_LOG_ERROR ("[LISP_OUTPUT] Drop! No valid source .");
      return;
    }

  // if MTU set, check MTU for the srcRloc too
  if (srcLocator->GetRlocMetrics ()->GetMtu () &&
      (packet->GetSize() + LispHeader ().GetSerializedSize () +
          Ipv4Header ().GetSerializedSize () * 2 +
          UdpHeader ().GetSerializedSize()) >
  srcLocator->GetRlocMetrics ()->GetMtu ())
    {
      // drop packet
      m_statisticsForIpv4->IncNoValidMtuPackets ();
      m_statisticsForIpv4->IncOutputDropPackets ();
      m_statisticsForIpv4->IncOutputPackets();
      NS_LOG_DEBUG ("MTU SRC " << srcLocator->GetRlocMetrics ()->GetMtu ());
      NS_LOG_ERROR ("[LISP_OUTPUT] Drop! MTU check failed for source RLOC.");
      return;
    }

  // add inner header before encapsulating whole packet
  packet->AddHeader (innerHeader);

  // compute src port based on inner header(Follow algo get_lisp_srcport)
  udpSrcPort = LispOverIp::GetLispSrcPort(packet);

  // add LISP header to the packet (with inner IP header)
  packet = PrependLispHeader (packet, localMapping, remoteMapping, srcLocator, destLocator);

  if (!packet)
    {
      m_statisticsForIpv4->IncNoEnoughSpace ();
      m_statisticsForIpv4->IncOutputDropPackets ();
      m_statisticsForIpv4->IncOutputPackets();
      NS_LOG_ERROR ("[LISP_OUTPUT] Drop! Not enough buffer space for packet.");
      return;
    }

  /*
   * We perform the encapsulation by adding the UDP then the outer IP header
   * according to the source locator Address Family (AF) -- If src ipv4
   * add ipv4 header, ipv6 if not.
   */
  if (Ipv4Address::IsMatchingType (srcLocator->GetRlocAddress ()))
    {
      // NB. In ns3 payloadSize is the size of payload without ip header.
      // /!\ problem with udp header payload size
      udpLength = innerHeader.GetPayloadSize ()+ innerHeader.GetSerializedSize () + UdpHeader ().GetSerializedSize () + LispHeader ().GetSerializedSize ();

      //NS_LOG_DEBUG ("Building outer header");
      // outer Ipv4 Header
      outerHeader.SetPayloadSize(udpLength + outerHeader.GetSerializedSize ());
      outerHeader.SetTtl (innerHeader.GetTtl ()); // copy inner TTL to outer header
      outerHeader.SetTos (0); // Default TOS
      outerHeader.SetDontFragment (); // set don't fragment bit
      outerHeader.SetSource (Ipv4Address::ConvertFrom (srcLocator->GetRlocAddress ()));
      outerHeader.SetDestination (Ipv4Address::ConvertFrom (destLocator->GetRlocAddress ()));
      outerHeader.SetProtocol (UdpL4Protocol::PROT_NUMBER); // set udp protocol

      // we just add the Lisp header and the UDP header
      NS_LOG_LOGIC ("Encapsulating packet");
      packet = LispEncapsulate (packet, udpLength, udpSrcPort);
      NS_ASSERT(m_statisticsForIpv4 != 0);
      m_statisticsForIpv4->IncOutputPackets ();
      // finally we have a packet that we can re-inject in IP
      Ptr<Ipv4> ipv4 = GetNode ()->GetObject<Ipv4> ();
      NS_LOG_LOGIC ("Re-injecting packet in IPV4");
      ipv4->SendWithHeader(packet, outerHeader, lispRoute);
    }
  else if (Ipv6Address::IsMatchingType (srcLocator->GetRlocAddress ()))
    {
      // TODO Implement IPv6
      LispOverIpv6Impl ().LispEncapsulate (packet, udpLength, udpSrcPort);
      // TODO Use Ipv6->Send()
      m_statisticsForIpv6->IncOutputDifAfPackets ();
    }
  else
    {
      // error
      // should never happen --- drop packet
      return;
    }
}

// Packets coming from a possible Rloc to enter the AS
void LispOverIpv4Impl::LispInput (Ptr<Packet> packet, Ipv4Header const &outerHeader)
{
  UdpHeader udpHeader;
  LispHeader lispHeader;
  Ipv4Header innerIpv4Header;
  Ipv6Header innerIpv6Header;
  bool isMappingForPacket;

  m_statisticsForIpv4->IncInputPacket ();

  // make basic check on the size
  if (packet->GetSize () < (udpHeader.GetSerializedSize () + outerHeader.GetSerializedSize ()))
    {
      NS_LOG_ERROR ("[LISP_INPUT] Drop! Packet size smaller that headers size.");
      m_statisticsForIpv4->IncBadSizePackets ();
      return;
    }

  // Remove the UDP headers
  // NB: the IP header has already been removed and checked
  packet->RemoveHeader (udpHeader);
  NS_LOG_DEBUG ("UDP header removed: " << udpHeader);

  // get the LISP header
  packet->RemoveHeader(lispHeader);
  NS_LOG_LOGIC ("Lisp header removed: " << lispHeader);

  packet->EnableChecking ();
  PacketMetadata::Item item;
  PacketMetadata::ItemIterator metadataIterator = packet->BeginItem ();

  /*
   * We know that the first header is an ip header
   * check the mappings for the src and dest EIDs
   * check in the case of Ipv6 and Ipv4 addresses (lisp_check_ip_mappings)
   *
   * IMPORTANT: But it is also possible that the first header is an ip header
   * of maq request. Lionel's implementation has not considered that the inner header
   * is a lisp data plan message?
   */
  while (metadataIterator.HasNext ())
    {
      item = metadataIterator.Next ();
      if (!(item.tid.GetName ().compare(Ipv4Header::GetTypeId().GetName ())))
        {
          isMappingForPacket = LispOverIp::m_mapTablesIpv4->
              IsMapForReceivedPacket(packet, lispHeader, static_cast<Address>
          (outerHeader.GetSource ()),
          static_cast<Address> (outerHeader.GetDestination ()));

          bool isControlPlanMsg = false;
          // if inner UDP of innerIpv4header is at port 4342. we should
          // use ipv4->Receive() to send this packet to itself.
          // that's why whether isMappingForPacket is true, we also retrieve @ip and port
	  Ptr<Node> node = GetNode();
	  packet->RemoveHeader(innerIpv4Header);
	  if (innerIpv4Header.GetProtocol () == UdpL4Protocol::PROT_NUMBER){
	      NS_LOG_DEBUG ("Next Header of Inner IP is still UDP, let's see the port");
	      packet->RemoveHeader(udpHeader);
	      if (udpHeader.GetDestinationPort () == LispOverIp::LISP_SIG_PORT)
	        {
		  NS_LOG_DEBUG ("UDP on port:"<<unsigned(udpHeader.GetDestinationPort ())<<"->LISP Control Plan Message!");
		  isControlPlanMsg = true;
	        }
	  }
	  innerIpv4Header.SetTtl(outerHeader.GetTtl());
	  Address from = static_cast<Address> (innerIpv4Header.GetSource ());
	  Address to = static_cast<Address> (innerIpv4Header.GetDestination ());
	  innerIpv4Header.EnableChecksum ();
	  // Add UDP&IP header to the packet
	  packet->AddHeader (udpHeader);
	  packet->AddHeader (innerIpv4Header);

	  // Either find mapping for inner ip header or find the inner message is control message
	  // use ip receive procedure to forward packet
          if (isMappingForPacket or isControlPlanMsg)
            {
              Ptr<Ipv4L3Protocol> ipv4 = (node->GetObject<Ipv4L3Protocol> ());
              // put it back in ip Receive ()
              //Attention: it's the method LispOverIpv4::RecordReceiveParams that
              //retrieve m_currentDevice values!!! This method is called in
              //Ipv4L3Protocol::Received() method!
              ipv4->Receive (m_currentDevice, packet, m_ipProtocol, from, to, m_currentPacketType);
              NS_LOG_DEBUG ("Re-inject the packet in receive to forward it to: " << innerIpv4Header.GetDestination ());
            }
          else
            {
              // TODO drop and log
              NS_LOG_ERROR("Mapping check failed during local deliver! Attention if this is cause by double encapsulation!");
            }
          return;
        }
      // if inner header is ipv6
      else if (!(item.tid.GetName ().compare (Ipv6Header::GetTypeId ().GetName ())))
        {
          m_statisticsForIpv6->IncInputDifAfPackets ();
          isMappingForPacket = LispOverIp::m_mapTablesIpv6->IsMapForReceivedPacket(packet, lispHeader, static_cast<Address> (outerHeader.GetSource ()), static_cast<Address> (outerHeader.GetDestination ()));
          if (isMappingForPacket)
            {
              // remove inner ipheader
              // TODO do the same for Ipv6
            }
          else
            {
              // TODO drop and log
            }
          return;
        }
      else
        {
          // should not happen -- report error
          NS_LOG_ERROR ("[LISP_INPUT] Drop! Unrecognized inner AF");
          m_statisticsForIpv4->IncBadSizePackets ();
          return;
        }
    }
}

LispOverIpv4::MapStatus LispOverIpv4Impl::IsMapForEncapsulation (Ipv4Header const &innerHeader , Ptr<MapEntry> &srcMapEntry, Ptr<MapEntry> &destMapEntry, Ipv4Mask mask)
{
  // mask == mask of the output iface
  NS_LOG_FUNCTION (this << innerHeader << mask);

  // Check if the source address is already defined
  if (innerHeader.GetSource ().IsEqual(Ipv4Address::GetAny()))
    {
      return LispOverIpv4::No_Mapping;
    }

  if (srcMapEntry == 0)
    {
      // Check if the prefix of the source address is in the db
      srcMapEntry = LispOverIp::DatabaseLookup (static_cast<Address> (innerHeader.GetSource ()));
      NS_LOG_DEBUG ("found srcMapEntry: \n"<<srcMapEntry->GetLocators()->Print());
    }

  // also check if it is the same address range (mask)
  if (srcMapEntry == 0)
    {
      NS_LOG_DEBUG ("[MapForEncap] Source map entry does not exist");
      Ptr<MapTables> mapTable = LispOverIp::GetMapTablesV4();
      //NS_LOG_DEBUG ("Current Database content: \n"<<*mapTable);
      return LispOverIpv4::No_Mapping;
    }

  if (srcMapEntry->IsNegative ())
    {
      NS_LOG_DEBUG ("[MapForEncap] Source map entry is a Negative mapping");
      return LispOverIpv4::No_Mapping;
    }

  if (mask.IsMatch (innerHeader.GetSource (), innerHeader.GetDestination ()))
    {
      NS_LOG_DEBUG ("[MapForEncap] No encap needed. Addresses matches in their mask!");
      return LispOverIpv4::No_Need_Encap;
    }

  if (destMapEntry == 0)
    {
      destMapEntry = LispOverIp::CacheLookup (static_cast<Address> (innerHeader.GetDestination ()));
    }

  // Cache miss !
  if (destMapEntry == 0)
    {
      NS_LOG_DEBUG ("[MapForEncap] SOURCE map entry exists but DEST map entry does not !");
      Ptr<LispOverIp> lisp = GetNode()->GetObject<LispOverIp>();
      //NS_LOG_INFO("XXX:"<<*lisp->GetMapTablesV4());
      MappingSocketMsgHeader sockMsgHdr;
      sockMsgHdr.SetMapAddresses ((int) sockMsgHdr.GetMapAddresses() | static_cast<int> (LispMappingSocket::MAPA_EID));
      sockMsgHdr.SetMapType (static_cast<uint8_t> (LispMappingSocket::MAPM_MISS));
      sockMsgHdr.SetMapVersion (LispMappingSocket::MAPM_VERSION);

      Ptr<MappingSocketMsg> mapSockMsg = Create<MappingSocketMsg> ();
      mapSockMsg->SetEndPoint (Create<EndpointId> (static_cast<Address> (innerHeader.GetDestination ())));
      mapSockMsg->SetLocators (0);
      NS_LOG_DEBUG ("[MapForEncap] EID not found " << innerHeader.GetDestination ());
      uint8_t buf[64];
      mapSockMsg->Serialize (buf);
      Ptr<Packet> packet = Create<Packet> (buf, 100);
      NS_LOG_DEBUG ("Send Notification to all LISP apps");

      SendNotifyMessage (static_cast<uint8_t> (LispMappingSocket::MAPM_MISS), packet, sockMsgHdr, 0);
      return LispOverIpv4::No_Mapping;
    }
  else if (destMapEntry->IsNegative ())
  {
      NS_LOG_DEBUG ("Negative Map Entry" << innerHeader.GetDestination ());
     /*
      * A mapping exists but it is a negative one. We treat it as
      * if no mapping was found.
      */
      return LispOverIpv4::No_Mapping;
  }

  return LispOverIpv4::Mapping_Exist;
}

/*
 * Check if there exists a local mapping
 * for the source EID.
 */
bool LispOverIpv4Impl::NeedEncapsulation (Ipv4Header const &ipHeader, Ipv4Mask mask)
{
  NS_LOG_FUNCTION (this << " outer header: " << ipHeader);

  // check if the src address is defined
  if (ipHeader.GetSource () == Ipv4Address::GetAny ())
    {
      return false;
    }

  // Use dblookup to find the list of rlocs (if mapping exists)
  Ptr<MapEntry> eidMapEntry = LispOverIp::DatabaseLookup (static_cast<Address> (ipHeader.GetSource ()));
  // check if it has the same mask as the dest EID, if yes no need
  // to encap, if no encap
  if (eidMapEntry)
    {
      // Use check if addresses match in their mask
      if (mask.IsMatch(ipHeader.GetSource (), ipHeader.GetDestination()))
        {
	  NS_LOG_DEBUG("Why no mapping entry found for EID:"<<eidMapEntry->GetEidPrefix()->GetEidAddress());
          return false;
        }
      NS_LOG_DEBUG("The found EID-RLOC mapping is: EID("<<eidMapEntry->GetEidPrefix()->GetEidAddress()<<") --> RLOC\n("<<eidMapEntry->GetLocators()->Print()<<")");
      return true;
    }

  return false;
}

/*
 * Check if the received packet
 * OK !!
 */
bool LispOverIpv4Impl::NeedDecapsulation (Ptr<const Packet> packet, Ipv4Header const &ipHeader)
{
  NS_LOG_FUNCTION(this << " outer header: " << ipHeader);
  NS_ASSERT (packet != 0);

  Ptr<Packet> p = packet->Copy ();
  UdpHeader udpHeader;
  LispHeader lispHeader;

  if (packet->GetSize() < (udpHeader.GetSerializedSize () + ipHeader.GetSerializedSize () + lispHeader.GetSerializedSize ()))
    {
      NS_LOG_DEBUG ("Packet size not good");
      return false;
    }

  if (ipHeader.GetProtocol () == UdpL4Protocol::PROT_NUMBER)
    {
      NS_LOG_DEBUG ("Protocol beneath IP is really UDP");
      p->RemoveHeader (udpHeader);
      if (udpHeader.GetDestinationPort () == LispOverIp::LISP_DATA_PORT)
        {
          return true;
        }
    }
  return false;
}

// called at the end of lisp_output
Ptr<Packet> LispOverIpv4Impl::LispEncapsulate (Ptr<Packet> packet, uint16_t udpLength, uint16_t udpSrcPort)
{
  // add UDP header with src ports + dest port (4341 for Data or 4342 for Control)
  UdpHeader udpHeader;
  // UDP header
  udpHeader.SetDestinationPort(LispOverIp::LISP_DATA_PORT);
  udpHeader.SetSourcePort(udpSrcPort);
  udpHeader.ForceChecksum(0); // set checksum as 0 (RFC 6830)
  // the ip payload size include data + header sizes
  udpHeader.ForcePayloadSize(udpLength);

  // Finally put the headers (UDP the Ipv4)
  packet->AddHeader(udpHeader);

  return packet;
}
} /* namespace ns3 */
