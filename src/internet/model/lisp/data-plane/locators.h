/*
 * locators.h
 *
 *  Created on: Apr 1, 2017
 *      Author: qsong
 */

#ifndef SRC_INTERNET_MODEL_LISP_DATA_PLANE_LOCATORS_H_
#define SRC_INTERNET_MODEL_LISP_DATA_PLANE_LOCATORS_H_

#include "locator.h"
#include "ns3/ptr.h"
#include "ns3/simple-ref-count.h"
#include "ns3/address.h"

namespace ns3 {

/**
 *
 */
class Locators : public SimpleRefCount<Locators>
{

public:
  Locators ();
  virtual ~Locators ();

  virtual Ptr<Locator> FindLocator (const Address &address) const = 0;
  virtual void InsertLocator (Ptr<Locator> locator) = 0;
  virtual Ptr<Locator> GetLocatorByIdx (uint8_t locIndex) = 0;
  virtual uint8_t GetNLocators (void) const = 0;
  virtual Ptr<Locator> SelectFirsValidRloc (void) const = 0;
  virtual std::string Print (void) const = 0;

  virtual int Serialize (uint8_t *buf) = 0;

};

} /* namespace ns3 */

#endif /* SRC_INTERNET_MODEL_LISP_DATA_PLANE_LOCATORS_H_ */
