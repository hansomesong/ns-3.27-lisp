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
#include "map-resolver-helper.h"
//#include "ns3/map-resolver-client.h"
#include "ns3/map-resolver-ddt.h"
#include "ns3/uinteger.h"
#include "ns3/names.h"
#include "ns3/lisp-protocol.h"
#include "ns3/lisp-over-ip.h"

namespace ns3
{

MapResolverDdtHelper::MapResolverDdtHelper ()
{
  m_factory.SetTypeId (MapResolverDdt::GetTypeId ());
}

MapResolverDdtHelper::~MapResolverDdtHelper ()
{
}

void MapResolverDdtHelper::SetAttribute (std::string name, const AttributeValue &value)
{
  m_factory.Set (name, value);
}

ApplicationContainer
MapResolverDdtHelper::Install (Ptr<Node> node) const
{
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
MapResolverDdtHelper::Install (std::string nodeName) const
{
  Ptr<Node> node = Names::Find<Node> (nodeName);
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
MapResolverDdtHelper::Install (NodeContainer c) const
{
  ApplicationContainer apps;
    for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
      {
        apps.Add (InstallPriv (*i));
      }

    return apps;
}

Ptr<Application> MapResolverDdtHelper::InstallPriv (Ptr<Node> node) const
{
  Ptr<LispOverIp> lisp = node->GetObject<LispOverIp>();
  NS_ASSERT_MSG(lisp!=0, "a MR must have one LispOverIp object! It is a lisp-speaking device!");
  Ptr<MapResolverDdt> app = m_factory.Create<MapResolverDdt> ();
  app->SetMapServerAddress (m_mapServerAddress);
  node->AddApplication (app);
  return app;
}

void MapResolverDdtHelper::AddDdtRootRloc (Ptr<Locator> locator)
{
  m_ddtRootRlocs.push_back (locator);
}

void MapResolverDdtHelper::SetDdtRootRlocs (std::list<Ptr<Locator> > locators)
{
  m_ddtRootRlocs = locators;
}

void MapResolverDdtHelper::SetMapServerAddress (Address mapServerAddress)
{
  m_mapServerAddress = mapServerAddress;
}

} /* namespace ns3 */
