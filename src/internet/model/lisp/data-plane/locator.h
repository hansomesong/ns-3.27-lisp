/*
 * locator.h
 *
 *  Created on: Apr 1, 2017
 *      Author: qsong
 */

#ifndef SRC_INTERNET_MODEL_LISP_DATA_PLANE_LOCATOR_H_
#define SRC_INTERNET_MODEL_LISP_DATA_PLANE_LOCATOR_H_

#include "rloc-metrics.h"
#include "ns3/log.h"
#include "ns3/packet.h"
#include "ns3/ptr.h"
#include "ns3/address.h"

namespace ns3 {

/**
 * \brief Implementation of a Routing LOCator.
 *
 * The goal of this class is to hold the information related to an
 * RLOC (the RLOC address and its metrics).
 *
 */
class Locator : public SimpleRefCount<Locator>
{
public:

  /**
   * \brief Constructor
   * \param rlocAddress The RLOC address
   */
  Locator (Address rlocAddress);
  /**
   * \brief Constructor
   */
  Locator ();

  /**
   * \brief Destructor
   */
  ~Locator ();

  /**
   * \brief Get the address of the RLOC.
   * \return The RLOC address.
   */
  Address GetRlocAddress (void) const;

  /**
   * \brief Set the RLOC address.
   * \param rlocAddress The RLOC address
   */
  void SetRlocAddress (const Address &rlocAddress);

  /**
   * \brief Get the metrics associated to the RLOC.
   * \return A pointer to the metrics associated to the RLOC.
   */
  Ptr<RlocMetrics> GetRlocMetrics (void) const;

  /**
   * \brief Set the metrics associated to this RLOC.
   * \param rlocMetrics The metrics associated to the RLOC.
   */
  void SetRlocMetrics (Ptr<RlocMetrics> rlocMetrics);

  /**
   * \brief Print some information about the RLOC.
   * \return The information about the RLOC.
   */
  std::string Print (void);
  void Print (std::ostream &os) const;

  /**
   * \brief Serialize the RLOC.
   * \param buf Buffer in which the RLOC will be serialized.
   * \return The size of the buffer after serialization.
   */
  uint8_t Serialize (uint8_t *buf);

  /**
   * \brief Deserialize the RLOC.
   * \param buf Buffer that contains the serialized RLOC.
   * \return
   */
  static Ptr<Locator> Deserialized (const uint8_t *buf);
  static Ptr<Locator> DeserializedInMapReplyRecord(const uint8_t *buf);

private:
  Address m_rlocAddress;
  Ptr<RlocMetrics> m_metrics;
};

} /* namespace ns3 */

#endif /* SRC_INTERNET_MODEL_LISP_DATA_PLANE_LOCATOR_H_ */
