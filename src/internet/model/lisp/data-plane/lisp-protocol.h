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
#ifndef SRC_INTERNET_LISP_PROTOCOL_H_
#define SRC_INTERNET_LISP_PROTOCOL_H_

#include "ns3/address.h"
#include "ns3/log.h"
#include "ns3/packet.h"
#include "ns3/ptr.h"
#include "ns3/simple-ref-count.h"
#include "lisp-header.h"
#include "locator.h"
#include "rloc-metrics.h"
#include "endpoint-id.h"

namespace ns3 {


class MapEntry;
class LispOverIp;


/**
 * \brief Implementation of the LISP protocol utilities and declaration
 * of LISP constants.
 *
 * The main goal of this class is to implement static check
 * function for the LISP protocol.
 */
class LispProtocol
{

public:

  // TODO set number of RLOC in the mapping database/cache



  /**
   * \brief Get the type ID of this class.
   * \return type ID
   */
  static TypeId GetTypeId (void);

  /**
   * \brief Constructor
   */
  LispProtocol ();

  /**
   * \brief Destructor
   */
  virtual ~LispProtocol ();


};

/**
 * \brief Implementation of the statistics related to LISP.
 */
class LispStatistics : public Object
{
public:

  // TODO Add attributes
  static TypeId GetTypeId (void);
  LispStatistics ();
  ~LispStatistics ();

  /**
   * This method increases the m_noLocMapPresent by one unit.
   */
  void NoLocalMap (void);
  /**
   *
   */
  void BadDestVersionNumber (void);
  /**
   *
   */
  void BadSrcVersionNumber (void);
  /**
   *
   */
  void IncInputPacket (void);
  /**
   *
   */
  void IncBadSizePackets (void);
  /**
   *
   */
  void IncInputDifAfPackets (void);
  /**
   *
   */
  void IncCacheMissPackets (void);
  /**
   *
   */
  void IncNoValidRloc (void);
  /**
   *
   */
  void IncOutputDropPackets (void);
  /**
   *
   */
  void IncNoValidMtuPackets (void);
  /**
   *
   */
  void IncNoEnoughSpace (void);
  /**
   *
   */
  void IncOutputPackets (void);
  /**
   *
   */
  void IncOutputDifAfPackets (void);

  /**
   *
   * \param os
   */
  void Print (std::ostream &os) const;

private:
  // input statistics
  /*
   * total input packets
   */
  uint32_t m_inputPackets;
  /*
   * total input packets with a different
   * address family in the outer header packet
   */
  uint32_t m_inputDifAfPackets;
  /*
   * packets dropped because they are shorter
   * that their header.
   */
  uint32_t m_badSizePackets; // TODO may be remove
  /*
   * packets for which there is no
   * local mapping
   */
  uint32_t m_noLocMapPresent;
  /*
   * packets that are dropped because
   * the data length is larger that the packets
   * themselves
   */
  uint32_t m_badDataLength; // TODO may be remove
  /*
   * Packets with bad source version number
   */
  uint32_t m_badSourceVersionNumber;
  /*
   * packets with a bad destination version number
   */
  uint32_t m_badDestVesionNumber;

  // output statistics
  /*
   * total number of output lisp packets
   */
  uint32_t m_outputPackets;
  /*
   * total number of input packets with a different
   * address family in the inner header
   */
  uint32_t m_outputDifAfPackets;
  /*
   * packets that are dropped because of a cache miss
   */
  uint32_t m_cacheMissPackets;
  /*
   * packets dropped because there is no valid RLOC
   */
  uint32_t m_noValidRlocPackets;
  /*
   * packets dropped because they
   * don't pass the MTU check
   */
  uint32_t m_noValidMtuPackets;
  /*
   * packets dropped because the there is no buffer space
   */
  uint32_t m_noEnoughBufferPacket;
  /*
   * total output packets that are dropped
   */
  uint32_t m_outputDropPackets;
};

} /* namespace ns3 */

#endif /* LISP_H_ */
