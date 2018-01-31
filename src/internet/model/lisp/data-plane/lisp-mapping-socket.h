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
#ifndef LISP_MAPPING_SOCKET_H_
#define LISP_MAPPING_SOCKET_H_

#include <stdint.h>
#include <queue>
#include "ns3/socket.h"
#include "ns3/ptr.h"
#include "lisp-over-ip.h"


namespace ns3
{
class Node;
class Packet;
/**
 * \ingroup lisp
 *
 * \brief A sockets interface to LISP.
 *
 * This class provides a socket interface
 * to ns3's implementation of LISP.
 */
class LispMappingSocket : public Socket
{
public:

  /**
   * \brief Get the type ID
   * \return the object TypeID
   */
  static TypeId GetTypeId (void);

  /**
   * \brief Constructor
   */
  LispMappingSocket ();

  /**
   * \brief Destructor
   */
  virtual
  ~LispMappingSocket ();

  static const uint8_t MAPM_VERSION;
  /**
   * \enum MapSockMsgType
   *
   * \brief The LISP mapping socket message types
   */
  enum MapSockMsgType {
    MAPM_ADD = 1,     //!< Add Mapping to Cache
    MAPM_DELETE,      //!< Delete Mapping
    MAPM_GET,         //!< Get Mapping
    MAPM_MISS,        //!< Lookup failed
    MAPM_MISS_EID,    //!< Lookup failed and EID returned.
    MAPM_MISS_HEADER, //!< Lookup failed and IP header returned
    MAPM_MISS_PACKET, //!< Lookup failed and packet returned
    MAPM_LSBITS,      //!< Locator Status Bits changed
    MAPM_LOCALSTALE,  //!< Local map version is stale
    MAPM_REMOTESTALE, //!< Remote version is stale
    MAPM_NONCEMISMATCH, //!< Received a mismatching nonce
    MAPM_DATABASE_UPDATE, //!< Update a EID-RLOC mapping entry in Database, if the EID is not a key. Add this mapping entry.
    MAPM_REGISTER //!< For LISP-MN, if DHCP send mapping to database, send map register message again.
  };

  /**
   * \enum MappingFlags
   *
   * \brief Flags describing a Mapping.
   */
  enum MappingFlags {
    MAPF_DB = 0x001,            //!< Mapping is part of Database
    MAPF_VERSIONING = 0x002,    //!< Mapping uses versioning
    MAPF_LOCBITS = 0x004,       //!< Mapping uses LocStatus bits
    MAPF_STATIC = 0x008,        //!< Mapping has been added manually
    MAPF_UP = 0x010,            //!< mapping is usable
    MAPF_ALL = 0x020,           //!< Operation concerns DB and Cache
    MAPF_EXPIRED = 0x040,       //!< Mapping has not been used for a long time (see expunge time)
    MAPF_NEGATIVE = 0x080,      //!< Negative Mapping (no Rlocs, forward natively)
    MAPF_DONE = 0x100,          //!< message confirmed (send back by the kernel to notify)
  };

  /**
   * \enum MissMsgType
   *
   * \brief
   */
  enum MissMsgType {
    LISP_MISSMSG_EID = 1,//!< LISP_MISSMSG_EID
    LISP_MISSMSG_HEADER, //!< LISP_MISSMSG_HEADER
    LISP_MISSMSG_PACKET, //!< LISP_MISSMSG_PACKET
  };
  static const MissMsgType m_lispMissMsgType;

  /**
   * \enum PresentMapAddrs
   *
   * \brief
   */
  enum PresentMapAddrs {
    MAPA_EID = 0x1,    //!< MAPA_EID
    MAPA_EIDMASK = 0x2,//!< MAPA_EIDMASK
    MAPA_RLOC = 0x4    //!< MAPA_RLOC
  };

  /**
   * \brief Set the node associated with this socket.
   * \param node node to set
   */
  void SetNode (Ptr<Node> node);

  void SetLisp (Ptr<LispOverIp> lisp);

  virtual enum Socket::SocketErrno GetErrno () const;

  /**
   * \brief Get socket type (NS3_SOCK_RAW)
   * \return socket type
   */
  virtual enum Socket::SocketType GetSocketType (void) const;

  virtual Ptr<Node> GetNode (void) const;
  virtual int Bind (const Address &address);
  virtual int Bind (void);
  virtual int Bind6 (void);
  virtual int GetSockName (Address &address) const;
  virtual int GetPeerName (Address &address) const;
  virtual int Close (void);
  virtual int ShutdownSend (void);
  virtual int ShutdownRecv (void);
  virtual int Connect (const Address &address);
  virtual int Listen (void);
  virtual uint32_t GetTxAvailable (void) const;
  virtual int Send (Ptr<Packet> p, uint32_t flags);
  virtual int SendTo (Ptr<Packet> p, uint32_t flags,
                      const Address &toAddress);
  virtual uint32_t GetRxAvailable (void) const;
  virtual Ptr<Packet> Recv (uint32_t maxSize, uint32_t flags);
  virtual Ptr<Packet> RecvFrom (uint32_t maxSize, uint32_t flags,
                                Address &fromAddress);
  virtual bool SetAllowBroadcast (bool allowBroadcast);
  virtual bool GetAllowBroadcast (void) const;

  void SetSockIndex (uint8_t sockIndex);

  uint32_t GetRcvBufSize (void) const;
  void SetRcvBufSize (uint32_t rcvBufSize);

private:
  std::queue<Ptr<Packet> >GetDeliveryQueue (void);
  void Forward (Ptr<const Packet> packet, const Address &from);
  Ptr<LispOverIp> m_lisp;       //!< The associated LISP protocol
  Address m_destAddres;         //!< The destination address
  uint8_t m_lispSockIndex;      //!< The index assigned to this socket
  enum Socket::SocketErrno m_errno;   //!< Last error number.
  Ptr<Node> m_node;                 //!< Node
  bool m_shutdownSend;              //!< Flag to shutdown send capability.
  bool m_shutdownRecv;              //!< Flag to shutdown receive capability.
  bool m_connected;     //!< state of the socket (connected or not)
  std::queue<Ptr<Packet> > m_deliveryQueue;     //!< Packet waiting to be processed.
  uint32_t m_rxAvailable;       //!< Number of available bytes to be received
  uint32_t m_rcvBufSize;        //!< receive buffer size


};

} /* namespace ns3 */

#endif /* LISP_MAPPING_SOCKET_H_ */
