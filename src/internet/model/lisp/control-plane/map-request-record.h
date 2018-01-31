/*
 * map-request-record.h
 *
 *  Created on: Apr 4, 2017
 *      Author: qsong
 */

#ifndef SRC_INTERNET_MODEL_LISP_CONTROL_PLANE_MAP_REQUEST_RECORD_H_
#define SRC_INTERNET_MODEL_LISP_CONTROL_PLANE_MAP_REQUEST_RECORD_H_

#include "ns3/address.h"
#include "ns3/lisp-control-msg.h"
#include "ns3/ptr.h"
#include "ns3/simple-ref-count.h"



namespace ns3 {

class MapRequestRecord : public SimpleRefCount<MapRequestRecord>
{
public:
  MapRequestRecord ();
  MapRequestRecord (Address eidPrefix, uint8_t eidMaskLength);
  virtual
  ~MapRequestRecord ();

  LispControlMsg::AddressFamily GetAfi (void);
  void SetAfi (LispControlMsg::AddressFamily afi);

  void SetMaskLenght (uint8_t maskLenght);
  uint8_t GetMaskLength (void);

  void SetEidPrefix (Address prefix);
  Address GetEidPrefix (void);

  void Serialize (uint8_t *buf) const;
  void SerializeOld(uint8_t *buf);
  static Ptr<MapRequestRecord> Deserialize (uint8_t *buf);
  static Ptr<MapRequestRecord> DeserializeOld (uint8_t *buf);

  void Print (std::ostream& os);
private:
  LispControlMsg::AddressFamily m_afi;
  uint8_t m_eidMaskLenght;
  Address m_eidPrefix;
};

} /* namespace ns3 */

#endif /* SRC_INTERNET_MODEL_LISP_CONTROL_PLANE_MAP_REQUEST_RECORD_H_ */
