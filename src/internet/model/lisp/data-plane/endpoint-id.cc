/*
 * endpoint-id.cc
 *
 *  Created on: Apr 1, 2017
 *      Author: qsong
 */

#include "endpoint-id.h"
#include "ns3/assert.h"

namespace ns3 {

// EID
EndpointId::EndpointId ()
{
  m_eidAddress = static_cast<Address> (Ipv4Address ()); // ipv4 by default
  m_mask = Ipv4Mask ();
  m_prefix = Ipv6Prefix ();
}

EndpointId::EndpointId (const Address &eidAddress)
{

  m_eidAddress = eidAddress;
  m_mask = Ipv4Mask ();
  m_prefix = Ipv6Prefix ();
}

EndpointId::EndpointId (const Address &eidAddress, const Ipv4Mask &mask)
{
  NS_ASSERT (Ipv4Address::IsMatchingType (eidAddress));
  m_eidAddress = Ipv4Address::ConvertFrom(eidAddress);
  m_mask = mask;
  m_prefix = Ipv6Prefix ();
}
EndpointId::EndpointId (const Address &eidAddress, const Ipv6Prefix &prefix)
{
  NS_ASSERT (Ipv6Address::IsMatchingType (eidAddress));
  m_eidAddress = eidAddress;
  m_mask = Ipv4Mask ();
  m_prefix = prefix;
}

EndpointId::~EndpointId () {

}

void EndpointId::SetEidAddress (const Address &eidAddress)
{
  NS_ASSERT (Ipv4Address::IsMatchingType (eidAddress) || Ipv6Address::IsMatchingType (eidAddress));
  m_eidAddress = eidAddress;
}

Address
EndpointId::GetEidAddress (void) const
{
  return m_eidAddress;
}

void EndpointId::SetIpv4Mask (const Ipv4Mask &mask)
{
  m_mask = mask;
}

Ipv4Mask
EndpointId::GetIpv4Mask (void) const
{
  return m_mask;
}

Ipv6Prefix
EndpointId::GetIpv6Prefix(void) const
{
  return m_prefix;
}

void
EndpointId::SetIpv6Prefix(const Ipv6Prefix &prefix)
{
  m_prefix = prefix;
}

bool EndpointId::IsIpv4 (void) const
{
  return Ipv4Address::IsMatchingType (m_eidAddress);
}

std::string EndpointId::Print (void) const
{
  std::string eid = "EID prefix: ";
  if (IsIpv4 ())
    {
      std::stringstream str;
      Ipv4Address::ConvertFrom (m_eidAddress).Print(str);
      eid += str.str ();
      str.str (std::string ());
      m_mask.Print (str);
      eid += " with ipv4 mask: " + str.str ();
    }
  else
    {
      std::stringstream str;
      Ipv6Address::ConvertFrom (m_eidAddress).Print(str);
      eid += str.str () + "\n";
      str.str (std::string ());
      m_prefix.Print (str);
      eid += "ipv6 prefix: " + str.str () + "\n\n";
    }

  return eid;
}

uint8_t EndpointId::Serialize (uint8_t buf[33]) const
{
  uint8_t size = 0;
  if (IsIpv4 ())
    {
      buf[0] = 1;
      Ipv4Address::ConvertFrom (m_eidAddress).Serialize (buf+1);
      buf[5] = (m_mask.Get () >> 24) & 0xff;
      buf[6] = (m_mask.Get () >> 16) & 0xff;
      buf[7] = (m_mask.Get () >> 8) & 0xff;
      buf[8] = (m_mask.Get () >> 0) & 0xff;
      size = 9;
    }
  else
    {
      buf[0] = 0;
      Ipv6Address::ConvertFrom (m_eidAddress).Serialize (buf+1);
      m_prefix.GetBytes (buf+17);
      size = 33;
    }

  return size;
}

uint8_t EndpointId::GetSerializedSize (void) const
{
  if (IsIpv4())
    return 9;
  else return 33;
}

Ptr<EndpointId> EndpointId::Deserialize (const uint8_t *buf)
{
  NS_ASSERT (buf);

  Ptr<EndpointId> endpointId = 0;
  if (buf[0])
    {
      Address eidAddress = static_cast<Address> (Ipv4Address::Deserialize (buf+1));
      uint32_t mask = 0;
      mask |= buf[5];
      mask <<= 8;
      mask |= buf[6];
      mask <<= 8;
      mask |= buf[7];
      mask <<= 8;
      mask |= buf[8];

      Ipv4Mask ipv4Mask = Ipv4Mask (mask);
       endpointId = Create<EndpointId> (eidAddress, ipv4Mask);
    }
  else
    {
      Address eidAddress = static_cast<Address> (Ipv6Address::Deserialize (buf+1));
      Ipv6Prefix prefix = Ipv6Prefix ((uint8_t *) buf+17);
      endpointId = Create<EndpointId> (eidAddress, prefix);
    }
  return endpointId;
}

} /* namespace ns3 */
