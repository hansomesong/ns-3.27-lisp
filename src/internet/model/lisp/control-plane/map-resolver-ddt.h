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
#ifndef SRC_INTERNET_MODEL_LISP_CONTROL_PLANE_MAP_RESOLVER_DDT_H_
#define SRC_INTERNET_MODEL_LISP_CONTROL_PLANE_MAP_RESOLVER_DDT_H_

#include "map-resolver.h"
#include "ns3/map-tables.h"
#include "ns3/lisp-control-msg.h"
#include "ns3/lisp-over-ip.h"
#include "ns3/map-referral-msg.h"
#include "ns3/map-request-msg.h"


namespace ns3
{

class ExtendedEidPrefix;
class MapReferralCache;
class PendingRequestList;

class MapResolverDdt : public MapResolver
{
public:
  MapResolverDdt ();
  virtual
  ~MapResolverDdt ();

  static TypeId
  GetTypeId (void);

  void SetMapServerAddress (Address mapServer);

private:

  virtual void StartApplication (void);

  virtual void StopApplication (void);

  virtual void SendMapRequest (Ptr<MapRequestMsg> mapRequestMsg);

  virtual void HandleRead (Ptr<Socket> socket);

  virtual void HandleReadFromClient (Ptr<Socket> socket);

  // when ddt is used
  Ptr<Locators> rootDdtNodeRlocs;
  //
  Address m_mapServerAddress;
  Ptr<MapReferralCache> m_mapRefCache;
  Ptr<PendingRequestList> m_pendingReqList;
};

class MapRefCacheEntry : public SimpleRefCount<MapRefCacheEntry>
{
public:

private:
  Ptr<Locators> locators;
};

class MapReferralCache : public SimpleRefCount<MapReferralCache>
{
public:

private:
  std::map<Ptr<ExtendedEidPrefix>, Ptr<MapRefCacheEntry> > m_mapRefCache;
};


class ExtendedEidPrefix : public SimpleRefCount<ExtendedEidPrefix>
{
public:

private:
  Ptr<EndpointId> m_eidPrefix;
  uint16_t m_lispDdtDbid;
  LispControlMsg::AddressFamily m_afi;
  uint32_t m_iid; //!< instance id
};

class PendingRequestEntry : public SimpleRefCount<PendingRequestEntry>
{
public:

private:
  Ptr<MapRequestMsg> m_mapRequest;
  Ptr<MapReferralMsg> m_lastMapReferral;
  Ptr<ExtendedEidPrefix> m_requestedXeid;
};

class PendingRequestList : public SimpleRefCount<PendingRequestList>
{

public:

private:
  std::map<Ptr<ExtendedEidPrefix>, Ptr<PendingRequestEntry> > m_pendingReqList;
};

} /* namespace ns3 */

#endif /* SRC_INTERNET_MODEL_LISP_CONTROL_PLANE_MAP_RESOLVER_DDT_H_ */
