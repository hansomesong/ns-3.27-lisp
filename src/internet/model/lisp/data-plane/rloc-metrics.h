/*
 * rloc-metrics.h
 *
 *  Created on: Apr 1, 2017
 *      Author: qsong
 */

#ifndef SRC_INTERNET_MODEL_LISP_DATA_PLANE_RLOC_METRICS_H_
#define SRC_INTERNET_MODEL_LISP_DATA_PLANE_RLOC_METRICS_H_

#include "ns3/assert.h"
#include "ns3/simple-ref-count.h"
#include "ns3/ptr.h"

namespace ns3 {

/**
 * \brief Implementation of the RLOC metrics.
 *
 * This class exists to holds the metrics associated to an RLOC.
 */
class RlocMetrics : public SimpleRefCount<RlocMetrics>
{
public:

  //static TypeId GetTypeId (void);
  /**
   * \brief Constructor
   */
  RlocMetrics ();
  /**
   * \brief Constructor
   * \param priority The RLOC Priority
   * \param weight The RLOC Weight
   */
  RlocMetrics (uint8_t priority, uint8_t weight);

  /**
   * \brief Constructor
   * \param priority The RLOC Priority
   * \param weight The RLOC Weight
   * \param reachable State of the RLOC (reachable or not)
   */

  RlocMetrics(uint8_t priority, uint8_t mpriority, uint8_t weight, uint8_t mweight);

  RlocMetrics (uint8_t priority, uint8_t weight, bool reachable);
  /**
   * \brief Destructor
   */
  ~RlocMetrics ();

  /**
   * The RLOC flags
   */
  enum RlocMetricsFlags {
    RLOCF_UP = 0x01,     //!< Set if RLOC is up
    RLOCF_LIF = 0x02,    //!< Set if the RLOC address is the one of the local interface
    RLOC_TXNONCE = 0x04, //!< Set if the transmission nonce is present
    RLOCF_RXNONCE = 0x08,//!< Set if the reception nonce is up.
  };

  enum RlocFlags{
	  RLOCF_L = 0x04,
	  RLOCF_p = 0x02,
	  RLOCF_R = 0x01,
  };

  enum RlocAfi{
	    IPv4 = 1,
	    IPv6 = 2,
  };
  /**
   * This method returns the priority of the RLOC.
   *
   * \return The priority of the RLOC
   */
  uint8_t GetPriority (void) const;

  /**
   * This method set the priority of the RLOC.
   *
   * \param priority The priority of the RLOC
   */
  void SetPriority (uint8_t priority);

  /**
   * Get the weight of the RLOC.
   *
   * \return the weight of the RLOC.
   */
  uint8_t GetWeight (void) const;
  /**
   * Set the weight of the RLOC.
   * \param weight the new weight of the RLOC.
   */
  void SetWeight (uint8_t weight);

  /**
   * Get the MTU of the RLOC.
   * \return the MTU.
   */
  uint32_t GetMtu (void) const;

  /**
   * Set the MTU of the RLOC.
   * \param mtu The MTU of the RLOC.
   */
  void SetMtu (uint32_t mtu);

  /**
   * Get the state of the RLOC (usable or not)
   * \return true if the RLOC is usable, false otherwise
   */
  bool IsUp (void);
  /**
   * Set the state of the RLOC.
   * \param status The new state of the RLOC.
   */
  void SetUp (bool status);

  /**
   * Get if the RLOC address is the one of a local interface or not.
   *
   * \return True if the RLOC address is the one of a local interface. False
   * otherwise.
   */
  bool IsLocalInterface (void);

  /**
   * Set if the RLOC is the one of a local interface or not.
   * \param isLocal The state of the RLOC address (local or not).
   */
  void SetIsLocalIf (bool isLocal);

  void SetFlagL(bool flag);

  bool GetFlagL(void);

  void SetFlagP(bool flag);

  bool GetFlagP(void);

  void SetFlagR(bool flag);

  bool GetFlagR(void);

  RlocAfi GetLocAfi(void);

  void SetLocAfi(RlocAfi afi);

  /**
   * Get if a transmission nonce is present or not.
   * \return True if a transmission nonce is present.
   */
  bool IsTxNoncePresent (void);
  /**
   * Set if the transmission nonce is present or not.
   * \param txNoncePresent The state of the transmission nonce (present or not)
   */
  void SetTxNoncePresent (bool txNoncePresent);

  /**
   * Get if the reception nonce is present or not.
   * \return True if the reception nonce is present.
   */
  bool IsRxNoncePresent (void);

  /**
   * Set the transmission nonce.
   * \param rxNoncePresent The state of the transmission nonce (present or not)
   */
  void SetRxNoncePresent (bool rxNoncePresent);

  /**
   * Get the transmission nonce.
   * \return The transmission nonce.
   */
  uint32_t GetTxNonce (void);
  /**
   * Set the transmission nonce.
   * \param txNonce The transmission nonce.
   */
  void SetTxNonce (uint32_t txNonce);

  /**
   * Get the reception nonce.
   * \return The reception nonce.
   */
  uint32_t GetRxNonce (void);

  /**
   * Set the reception nonce
   * \param rxNonce The reception nonce
   */
  void SetRxNonce (uint32_t rxNonce);

  /**
   * Print the RlocMetrics object.
   * \return The string representation of the RlocMetrics object.
   */
  std::string Print ();

  /**
   * Serialize the RlocMetrics object in the buffer given as
   * an argument.
   * \param buf The buffer in which the RlocMetrics object will be serialized.
   * \return The size of the buffer given as an argument after serialization.
   */
  uint8_t Serialize (uint8_t *buf);

  /**
   * Deserialize the RlocMetrics object from the buffer given as an argument
   * \param buf The buffer containing the serialized RlocMetrics object.
   * \return A pointer to the deserialized RlocMetrics object.
   */
  static Ptr<RlocMetrics> Deserialized (const uint8_t *buf);
  static Ptr<RlocMetrics> DeserializedInMapReplyRecord(const uint8_t *buf);

private:
  uint8_t m_priority; // Rloc priority
  uint8_t m_mpriority; // Multicast RLOC Priority
  /*
   * locator weight: used when several Rlocs have the same
   * priority. The sum of the weight fields of the Rlocs having the
   * same priority must always be 100 or 0.
   */
  uint8_t m_weight;
  uint8_t m_mweight; // Multicast RLOC weight

  bool m_flagp;
  bool m_flagL;
  bool m_flagR;

  RlocAfi m_locAfi;

  bool m_rlocIsUp; // Rloc Status Bit
  bool m_rlocIsLocalInterface; // Rloc is a local interface
  bool m_txNoncePresent; // Rloc Tx Nonce is present
  bool m_rxNoncePresent; // Rloc Rx Nonce is present
  uint32_t m_txNonce; // used when sending a LISP encapsulated packet
  uint32_t m_rxNonce; // used when receiving LISP encapsulated packet
  /*
   * This is useful for local mapping for which flag 'i' is
   * set.
   * Initialized at creation by copying the mtu of the corresponding
   * interface here. This value is not updated if changed. (TODO OK ?)
   */
  uint32_t m_mtu;
};

} /* namespace ns3 */

#endif /* SRC_INTERNET_MODEL_LISP_DATA_PLANE_RLOC_METRICS_H_ */
