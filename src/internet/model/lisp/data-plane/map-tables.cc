/*
 * MapTables.cc
 *
 *  Created on: 28 janv. 2016
 *      Author: lionel
 */

#include "ns3/map-tables.h"
#include "ns3/log.h"
#include "ns3/locators-impl.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE ("MapTables");

NS_OBJECT_ENSURE_REGISTERED (MapTables);

// MapTables
TypeId
MapTables::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::MapTables")
    .SetParent<Object> ()
    .SetGroupName("Lisp")
  ;
  return tid;
}

MapTables::MapTables (void)
  : m_dbMiss (0),
    m_dbHit (0),
    m_cacheMiss (0),
    m_cacheHit (0)
{
  NS_LOG_FUNCTION (this);
}

MapTables::~MapTables (void)
{
  NS_LOG_FUNCTION (this);
}

Ptr<LispOverIp> MapTables::GetLispOverIp (void)
{
  return m_lispProtocol;
}

void MapTables::SetLispOverIp(Ptr<LispOverIp> lispProtocol)
{
  m_lispProtocol = lispProtocol;
}


void MapTables::DbMiss (void)
{
  m_dbMiss++;
}

void MapTables::DbHit (void)
{
  m_dbHit++;
}

void MapTables::CacheHit (void)
{
  m_cacheHit++;
}
void MapTables::CacheMiss (void)
{
  m_cacheMiss++;
}

std::ostream& operator<< (std::ostream &os, MapTables const &mapTable)
{
  mapTable.Print(os);
  return os;
}

//////////////////
// MapEntryImpl

MapEntryImpl::MapEntryImpl() {
	m_locators = Create<LocatorsImpl>();
}

MapEntryImpl::MapEntryImpl(Ptr<Locator> locator) {
	m_locators = Create<LocatorsImpl>();
	m_locators->InsertLocator(locator);
}

MapEntryImpl::~MapEntryImpl() {

}

//TODO: If one
Ptr<Locator> MapEntryImpl::FindLocator(const Address &address) const {
	return m_locators->FindLocator(address);
}

Ptr<Locator> MapEntryImpl::RlocSelection(void) const {
	return m_locators->SelectFirsValidRloc();
}

std::string MapEntryImpl::Print(void) const {
	//std::string mapEntry = "EID prefix:";

	return m_locators->Print();
}

} /* namespace ns3 */
