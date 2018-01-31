/*
 * endpoint-id.h
 *
 *  Created on: Apr 1, 2017
 *      Author: qsong
 */

#ifndef SRC_INTERNET_MODEL_LISP_DATA_PLANE_ENDPOINT_ID_H_
#define SRC_INTERNET_MODEL_LISP_DATA_PLANE_ENDPOINT_ID_H_

#include "ns3/address.h"
#include "ns3/ipv4-address.h"
#include "ns3/ipv6-address.h"
#include "ns3/ptr.h"
#include "ns3/simple-ref-count.h"

namespace ns3 {

/**
 * \brief Implementation of an Endpoint ID (EID).
 * This class represents an EID prefix. It is characterized
 * by an address (IPv4 or v6) and by a IPv4 mask or IPv6 prefix.
 */
class EndpointId : public SimpleRefCount<EndpointId> {
public:
  /**
   * \brief Constructor
   */
  EndpointId ();
  /**
   * \brief Constructor
   */
  EndpointId (const Address &eidAddress);

  /**
   * \brief Constructor
   *
   * \param eidAddress an address (type IPv4)
   * \param mask The IPv4 mask associated to the prefix.
   */
  EndpointId (const Address &eidAddress, const Ipv4Mask &mask);

  /**
   * \brief Constructor
   *
   * \param eidAddress An address (type IPv6)
   * \param prefix The IPv6 prefix associated to this address.
   */
  EndpointId (const Address &eidAddress, const Ipv6Prefix &prefix);

  /**
   * \brief Destructor
   */
  ~EndpointId ();

  /**
   * \brief Get the EID address.
   * \return The EID address
   */
  Address GetEidAddress (void) const;
  /**
   * \brief Set the EID address.
   * \param eidAddress The EID address
   */
  void SetEidAddress (const Address &eidAddress);


  /**
   * \brief Get the IPv4 mask.
   * \return The IPv4 mask.
   */
  Ipv4Mask GetIpv4Mask () const;

  /**
   * \brief Set the IPv4 mask.
   * \param mask The IPv4 mask.
   */
  void SetIpv4Mask (const Ipv4Mask &mask);

  /**
   * \brief Set the IPv6 prefix
   * \param prefix The IPv6 prefix.
   */
  void SetIpv6Prefix (const Ipv6Prefix &prefix);

  /**
   * \brief Get the IPv6 prefix.
   * \return The IPv6 prefix of this EID prefix.
   */
  Ipv6Prefix GetIpv6Prefix (void) const;


  /**
   * This methods checks if the EID prefix is an IPv4 one.
   *
   * \returns True if the prefix is an IPv4 prefix, false otherwise (if
   * it is IPv6 prefix).
   */
  bool IsIpv4 (void) const;

  /**
   * \brief This method returns a character string representation of this EID prefix.
   *
   * \return A character string representing the EID
   */
  std::string Print (void) const;

  /**
   * This method serializes the EID prefix.
   *
   * \param buf The buffer that must contain the serialized EID prefix.
   *
   * \return The size of the buffer after the serialization of the EID prefix.
   */
  uint8_t Serialize (uint8_t buf[33]) const;

  /**
   * This method return the size of the buffer in which
   * the EID prefix is serialized to.
   *
   * \return The size of the buffer in which the EID prefix is serialized to.
   */
  uint8_t GetSerializedSize (void) const;

  /**
   * This method deserializes the EID from the buffer given as an
   * argument.
   *
   * \param buf The uint8_t array that contains the serialized representation
   * of the EID prefix.
   * \return A pointer to an EndpointId object. It points to the
   *  deserialized object
   */
  static Ptr<EndpointId> Deserialize (const uint8_t *buf);
private:
  Address m_eidAddress;
  Ipv4Mask m_mask;
  Ipv6Prefix m_prefix;
};

} /* namespace ns3 */

#endif /* SRC_INTERNET_MODEL_LISP_DATA_PLANE_ENDPOINT_ID_H_ */
