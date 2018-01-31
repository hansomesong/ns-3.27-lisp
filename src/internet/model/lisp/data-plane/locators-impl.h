/*
 * locators-impl.h
 *
 *  Created on: Mar 31, 2017
 *      Author: qsong
 */

#ifndef SRC_INTERNET_MODEL_LISP_DATA_PLANE_LOCATORS_IMPL_H_
#define SRC_INTERNET_MODEL_LISP_DATA_PLANE_LOCATORS_IMPL_H_

#include <list>
#include <ns3/ptr.h>
#include <ns3/address.h>
#include "ns3/lisp-over-ip.h"
#include "ns3/locators.h" //it includes locator.h

namespace ns3 {

class  LocatorsImpl : public Locators
{
public:
  LocatorsImpl ();
  virtual ~LocatorsImpl ();
  Ptr<Locator> FindLocator (const Address &address) const;
  Ptr<Locator> GetLocatorByIdx (uint8_t locIndex);
  void InsertLocator (Ptr<Locator> locator);
  uint8_t GetNLocators (void) const;
  Ptr<Locator> SelectFirsValidRloc (void) const;
  std::string Print (void) const;

  int Serialize (uint8_t *buf);

  static Ptr<LocatorsImpl> Deserialize (const uint8_t *buf);
private:
  static bool compare_rloc (Ptr<const Locator> a, Ptr<const Locator> b);

  std::list<Ptr<Locator> > m_locatorsChain;
};

} /* namespace ns3 */

#endif /* SRC_INTERNET_MODEL_LISP_DATA_PLANE_LOCATORS_IMPL_H_ */
