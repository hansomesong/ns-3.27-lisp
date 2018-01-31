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
#include "map-server-helper.h"
#include "ns3/map-server-ddt.h"
#include "ns3/uinteger.h"
#include "ns3/names.h"
#include "ns3/lisp-protocol.h"
#include "ns3/lisp-over-ip.h"

namespace ns3
{

MapServerDdtHelper::MapServerDdtHelper ()
{
  m_factory.SetTypeId (MapServerDdt::GetTypeId ());
}

  MapServerDdtHelper::~MapServerDdtHelper ()
{

}

void
MapServerDdtHelper::SetAttribute (std::string name, const AttributeValue &value)
{
  m_factory.Set (name, value);
}

ApplicationContainer
MapServerDdtHelper::Install (Ptr<Node> node) const
{
  /**
   * We should make sure that map server always has a lispoveripv4 or lispoveripv6 object
   * Because MapServerDdt should be a lisp-speaking equipment. For example, in LISP-MN
   * map server need to send encapsulated control message.
   */
  NS_ASSERT(node->GetObject<LispOverIp>() != 0);
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
MapServerDdtHelper::Install (std::string nodeName) const
{
  Ptr<Node> node = Names::Find<Node> (nodeName);
  NS_ASSERT(node->GetObject<LispOverIp>() != 0);
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
MapServerDdtHelper::Install (NodeContainer c) const
{
  ApplicationContainer apps;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      apps.Add (InstallPriv (*i));
    }

  return apps;
}

Ptr<Application> MapServerDdtHelper::InstallPriv (Ptr<Node> node) const
{
  Ptr<LispOverIp> lisp = node->GetObject<LispOverIp>();
  NS_ASSERT_MSG(lisp!=0, "a MR must have one LispOverIp object! It is a lisp-speaking device!");
  Ptr<MapServerDdt> app = m_factory.Create<MapServerDdt> ();
  node->AddApplication (app);
  // We tell MapResolverDdt the pointer to MapTables saved in lispOverIp
  app->SetMapTables(lisp->GetMapTablesV4(), lisp->GetMapTablesV6());
  return app;
}

} /* namespace ns3 */
