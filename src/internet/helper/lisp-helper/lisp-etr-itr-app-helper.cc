/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 University of Liege
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Lionel Agbodjan <lionel.agbodjan@gmail.com>
 */
#include "ns3/lisp-etr-itr-app-helper.h"
#include "ns3/uinteger.h"
#include "ns3/names.h"


namespace ns3
{

NS_LOG_COMPONENT_DEFINE("LispEtrItrAppHelper");


LispEtrItrAppHelper::LispEtrItrAppHelper ()
{
  m_factory.SetTypeId (LispEtrItrApplication::GetTypeId ());
}

LispEtrItrAppHelper::~LispEtrItrAppHelper ()
{
    // TODO Auto-generated destructor stub
}

void
LispEtrItrAppHelper::SetAttribute (std::string name, const AttributeValue &value)
{
  m_factory.Set (name, value);
}

ApplicationContainer
LispEtrItrAppHelper::Install (Ptr<Node> node) const
{
    return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
LispEtrItrAppHelper::Install (std::string nodeName) const
{
  Ptr<Node> node = Names::Find<Node> (nodeName);
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
LispEtrItrAppHelper::Install (NodeContainer c) const
{
  ApplicationContainer apps;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      apps.Add (InstallPriv (*i));
    }

  return apps;
}

Ptr<Application> LispEtrItrAppHelper::InstallPriv (Ptr<Node> node) const
{
  Ptr<LispEtrItrApplication> app = m_factory.Create<LispEtrItrApplication> ();

  Ptr<LispOverIp> lisp = node->GetObject<LispOverIp> ();
  /*
   * we are only interested in map entry that are statically configured
   * belonging to the db.
   */
  for (std::list<Ptr<Locator> >::const_iterator it = m_mapResolverRlocs.begin (); it != m_mapResolverRlocs.end (); it++)
    {
      app->AddMapResolverLoc (*it);
    }
  app->SetMapTables (lisp->GetMapTablesV4(), lisp->GetMapTablesV6());
  /**
   * Update:2018-01-27
   * MapTable should also hold a pointer to the xTR application
   */
  lisp->GetMapTablesV4()->SetxTRApp(app);
  NS_LOG_DEBUG("xTR address set to simple map table:"<<lisp->GetMapTablesV4()->GetxTRApp()<<" It should be:"<<app);
  lisp->GetMapTablesV6()->SetxTRApp(app);
  app->SetMapServerAddresses (m_mapServerAddresses);
  node->AddApplication (app);
  return app;
}

void LispEtrItrAppHelper::AddMapServerAddress (Address mapServerAddress)
{
  m_mapServerAddresses.push_front (mapServerAddress);
}

void LispEtrItrAppHelper::AddMapResolverRlocs (Ptr<Locator> locator)
{
  m_mapResolverRlocs.push_back (locator);
}

} /* namespace ns3 */
