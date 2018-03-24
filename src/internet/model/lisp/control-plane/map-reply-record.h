/*
 * MapReplyRecordNew.h
 *
 *  Created on: Mar 31, 2017
 *      Author: qsong
 */

#ifndef SRC_INTERNET_MODEL_LISP_CONTROL_PLANE_MAP_REPLY_RECORD_H_
#define SRC_INTERNET_MODEL_LISP_CONTROL_PLANE_MAP_REPLY_RECORD_H_



#include "ns3/address.h"
#include "ns3/lisp-control-msg.h"
#include "ns3/locators.h"
#include "ns3/ptr.h"
#include "ns3/simple-ref-count.h"

namespace ns3 {

class MapReplyRecord : public SimpleRefCount<MapReplyRecord>
{
public:

	enum ACT {
    NoAction = 0,
    NativelyForward,
    SendMapRequest,
    Drop,
  };

  MapReplyRecord ();
  virtual
  ~MapReplyRecord ();

  void SetRecordTtl (uint32_t recordTtl);
  uint32_t GetRecordTtl (void);

  void SetLocatorCount (uint8_t locCount);
  uint8_t GetLocatorCount (void);

  void SetEidMaskLength (uint8_t eidMaskLength);
  uint8_t GetEidMaskLength (void);

  void SetAct (ACT act);
  ACT GetAct (void);

  void SetA (uint8_t a);
  uint8_t GetA (void);

  void SetMapVersionNumber (uint16_t versionNumber);
  uint16_t GetMapVersionNumber (void);

  LispControlMsg::AddressFamily GetEidAfi (void);
  void SetEidAfi (LispControlMsg::AddressFamily afi);

  void SetLocators (Ptr<Locators> locators);
  Ptr<Locators> GetLocators (void);

  void SetEidPrefix (Address eidPrefix);
  Address GetEidPrefix (void);

  void Serialize (uint8_t *buf);
  static Ptr<MapReplyRecord> Deserialize (uint8_t *buf);

  void Print (std::ostream& os);

  static const uint32_t m_defaultRecordTtl;
  /**
   * Yue: I decide to declare the length (in number of bytes) as static and constant
   * variable in class header file.
   * I'm not sure if it is the best solution.
   */
  static const uint8_t RECORD_TTL_LEN = 4;
  static const uint8_t EID_PREFIX_AFI_LEN = 2;


private:
  uint32_t m_recordTtl;
  uint8_t m_locatorCount;
  uint8_t m_eidMaskLength;
  ACT m_act;
  uint8_t m_A:1;
  uint8_t m_reserved:7;
  uint16_t m_mapVersionNumber;
  LispControlMsg::AddressFamily m_eidPrefixAfi;
  Address m_eidPrefix;
  Ptr<Locators> m_locators;
};

} /* namespace ns3 */

#endif /* SRC_INTERNET_MODEL_LISP_CONTROL_PLANE_MAP_REPLY_RECORD_H_ */
