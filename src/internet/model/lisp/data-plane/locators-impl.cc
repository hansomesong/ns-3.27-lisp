/*
 * locators-impl.cc
 *
 *  Created on: Mar 31, 2017
 *      Author: qsong
 */

#include "locators-impl.h"

#include <sstream>

#include <ns3/log.h>

namespace ns3 {

////////////////
// LocatorsImpl
LocatorsImpl::LocatorsImpl ()
{

}

LocatorsImpl::~LocatorsImpl ()
{

}

Ptr<Locator> LocatorsImpl::GetLocatorByIdx (uint8_t locIndex)
{
  NS_ASSERT (locIndex < m_locatorsChain.size () && locIndex >= 0);

  int i = 0;
  for (std::list<Ptr<Locator> >::iterator it = m_locatorsChain.begin ();
      it != m_locatorsChain.end (); ++it, i++)
    {
      if (i == locIndex)
        return *it;
    }
  return 0;
}

Ptr<Locator>
LocatorsImpl::FindLocator (const Address &address) const
{
  for (std::list<Ptr<Locator> >::const_iterator it = m_locatorsChain.begin ();
    it != m_locatorsChain.end (); ++it)
  {
    if ((*it)->GetRlocAddress () == address)
      return *it;
  }
  return 0;
}

uint8_t
LocatorsImpl::GetNLocators (void) const
{
  return m_locatorsChain.size ();
}

Ptr<Locator>
LocatorsImpl::SelectFirsValidRloc (void) const
{
  /*
   * the first valid rloc of the linked list
   * is the one which is up and whose priority is < than 255
   */
  for (std::list<Ptr<Locator> >::const_iterator it = m_locatorsChain.begin ();
      it != m_locatorsChain.end (); ++it)
    {
      if ((*it)->GetRlocMetrics ()->IsUp () && (*it)->GetRlocMetrics()->GetPriority() < LispOverIp::LISP_MAX_RLOC_PRIO)
        return *it;
    }
    return 0;
}

void
LocatorsImpl::InsertLocator (Ptr<Locator> locator)
{
  m_locatorsChain.push_back(locator);
  m_locatorsChain.sort (compare_rloc);
}


std::string LocatorsImpl::Print (void) const
{
  std::string locators = std::string ();

  int i = 1;
  std::stringstream str;
  for (std::list<Ptr<Locator> >::const_iterator it = m_locatorsChain.begin ();
        it != m_locatorsChain.end (); ++it)
    {
      str << i;
      locators += "RLOC" + str.str () + "\n";
      locators += (*it)->Print ();
      str.str(std::string());
      i++;
    }

  return locators;
}

int LocatorsImpl::Serialize (uint8_t *buf)
{
  buf[0] = GetNLocators ();
  int position = 1;
  uint8_t size = 0;

  for (std::list<Ptr<Locator> >::const_iterator it = m_locatorsChain.begin (); it != m_locatorsChain.end (); ++it)
    {
     position++;
     size = (*it)->Serialize (buf + position);
     buf[position - 1] = size;
     position += size;
    }
  return position;
}

Ptr<LocatorsImpl> LocatorsImpl::Deserialize (const uint8_t *buf)
{
  uint8_t locCount = buf[0];
  int position = 1;
  uint8_t size = 0;

  Ptr<LocatorsImpl> locators = Create<LocatorsImpl> ();

  for (int i = 0; i < locCount; i++)
    {
      size = buf[position];
      locators->InsertLocator (Locator::Deserialized (buf + position + 1));
      position += size + 1;
    }
  return locators;
}

bool LocatorsImpl::compare_rloc (Ptr<const Locator> a, Ptr<const Locator> b)
  {
    if (a->GetRlocMetrics ()->IsUp () && a->GetRlocMetrics ()->GetPriority () < b->GetRlocMetrics ()->GetPriority ())
      {
        return true;
      }
    else if (b->GetRlocMetrics ()->IsUp () && b->GetRlocMetrics ()->GetPriority () < a->GetRlocMetrics ()->GetPriority ())
      {
        return false;
      }
    else
      {
        return a->GetRlocMetrics ()->GetWeight () < b->GetRlocMetrics ()->GetWeight ();
      }
  }


} /* namespace ns3 */
