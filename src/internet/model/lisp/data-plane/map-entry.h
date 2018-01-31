/*
 * map-entry.h
 *
 *  Created on: Jul 9, 2017
 *      Author: qsong
 */

#ifndef SRC_INTERNET_MODEL_LISP_DATA_PLANE_MAP_ENTRY_H_
#define SRC_INTERNET_MODEL_LISP_DATA_PLANE_MAP_ENTRY_H_

#include "locator.h"
#include "locators.h"
#include "ns3/ptr.h"
#include "ns3/simple-ref-count.h"
#include "ns3/endpoint-id.h"

namespace ns3 {
/**
 *
 */
class MapEntry : public SimpleRefCount<MapEntry>
{

  static const uint8_t MAX_RLOCS;

public:
  MapEntry ();
  virtual ~MapEntry ();

  virtual Ptr<Locator> FindLocator (const Address &address) const = 0;
  virtual Ptr<Locator> RlocSelection (void) const = 0;
  void InsertLocator (Ptr<Locator> locator);
  Ptr<Locators> GetLocators (void);
  void SetLocators (Ptr<Locators> locators);
  bool IsUsingVersioning (void) const;
  void SetIsUsingVersioning (bool is);
  bool IsUsingLocStatusBits (void) const;
  void SetIsUsingLocStatusBits (bool is);
  bool IsNegative (void) const;
  void setIsNegative (bool isNegative);
  uint16_t GetVersionNumber (void) const;
  void SetVersionNumber (uint16_t versionNb);
  uint32_t GetLocsStatusBits (void) const;
  virtual std::string Print (void) const = 0;
  void SetEidPrefix (Ptr<EndpointId> prefix);
  Ptr<EndpointId> GetEidPrefix (void);

protected:
  // TODO add a possible lock to the entry
  Ptr<EndpointId> m_eidPrefix;
  Ptr<Locators> m_locators;

private:
  // Mapping flags
  bool m_inDatabase;
  bool m_isStatic; // Is manually added
  bool m_isUsable;
  bool m_isExpired;
  bool m_isNegative;
  bool m_useVersioning;
  bool m_useLocatorStatusBits;

  uint16_t m_mappingVersionNumber; // Version number of the mapping
  uint32_t m_rlocsStatusBits;
  // TODO investigate last time it has been used (to expunge cache)
};

} /* namespace ns3 */

#endif /* SRC_INTERNET_MODEL_LISP_DATA_PLANE_MAP_ENTRY_H_ */
