/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2015-2016 University of Liege
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
 * Author:
 */

#ifndef LISP_HEADER_H
#define LISP_HEADER_H

#include "ns3/header.h"
#include "ns3/object.h"

namespace ns3 {

/**
 * \class LispHeader
 * \brief Packet header for LISP
 */
class LispHeader : public Header
{
public:

  /**
   * \brief Get the type identifier.
   * \return type identifier
   */
  static TypeId GetTypeId (void);

  /**
   * \brief Return the instance type identifier.
   * \return instance type ID
   */
  virtual TypeId GetInstanceTypeId (void) const;

  /**
   * \brief Constructor.
   */
  LispHeader (void);


  /**
   * \brief Set the N bit of the header.
   * \param N The N bit
   */
  void SetNBit (uint8_t N);

  /**
   * \brief Get the N bit
   * \return The N bit
   */
  uint8_t GetNBit (void) const;

  /**
   * \brief Set the L bit
   * \param L The L bit
   */
  void SetLBit (uint8_t L);

  /**
   *  \brief Get the L bit.
   *  \return the L bit.
   */
  uint8_t GetLBit (void) const;

  /**
   * \brief Set the E bit.
   * \param E The E bit
   */
  void SetEBit (uint8_t E);

  /**
   * \brief Get the E bit.
   * \return the E bit
   */
  uint8_t GetEBit (void) const;

  /**
   * \brief Set the V bit
   * \param V The V bit
   */
  void SetVBit (uint8_t V);

  /**
     * \brief Set the L bit
     * \param N The N bit
     */
  uint8_t GetVBit (void) const;

  /**
   * \brief Set the I bit
   * \param I The I bit
   */
  void SetIBit (uint8_t I);

  /**
   * \brief Get the I bit.
   * \return The I bit.
   */
  uint8_t GetIBit (void) const;

  /**
   * \brief Set the flags
   * \param flags The flags
   */
  void SetFlagsBits (uint8_t flags);

  /**
   * \brief Get the reserved flags.
   * \return The reserved flags.
   */
  uint8_t GetFlagsBits (void) const;

  /**
   * \brief Set the nonce
   * \param nonce The nonce.
   */
  void SetNonce (uint32_t nonce);

  /**
   * \brief Get the nonce.
   * \return The nonce.
   */
  uint32_t GetNonce (void) const;

  /**
   * \brief Set the Locator Status Bits
   * \param lsbs The Locator Status Bits.
   */
  void SetLSBs (uint32_t lsbs);

  /**
   * \brief Get the Locator Status Bits.
   * \return The Locator Status Bits.
   */
  uint32_t GetLSBs (void) const;

  /**
   * \brief Set the source Map Version number.
   * \param versionNumber The source map version number.
   */
  void SetSrcMapVersion (uint32_t versionNumber);

  /**
   * \brief Get the source map version number.
   * \return The source map version number.
   */
  uint32_t GetSrcMapVersion (void) const;

  /**
     * \brief Set the destination map version number.
     * \param versionNumber The destination map version number
     */
  void SetDestMapVersion (uint32_t versionNumber);

  /**
   * \brief Get the destination map version number.
   * \return The destination map version number.
   */
  uint32_t GetDestMapVersion (void) const;

  /**
   * \brief Set the instance ID.
   * \param instanceId The instance ID.
   */
  void SetInstanceID (uint32_t instanceId);

  /**
   * \brief Get the Instance ID.
   * \return The Instance ID.
   */
  uint32_t GetInstanceID (void) const;
  /**
   * \brief Print some informations about the LISP header.
   * \param os output stream
   * \return info about this LISP header
   */
  virtual void Print (std::ostream& os) const;

  /**
   * \brief Get the serialized size of the header.
   * \return size
   */
  virtual uint32_t GetSerializedSize (void) const;

  /**
   * \brief Serialize the header.
   * \param start Buffer iterator
   */
  virtual void Serialize (Buffer::Iterator start) const;

  /**
   * \brief Deserialize the header.
   * \param start Buffer iterator
   * \return size of the header
   */
  virtual uint32_t Deserialize (Buffer::Iterator start);

private:
  /**
   * \brief The N-bit (Nonce-present bit).
   */
  uint8_t m_N : 1;

  /**
   * \brief The L-bit (the 'Locator-Status-Bits' field enabled bit)
   */
  uint8_t  m_L : 1;

  /**
   * \brief The E-bit (echo-Nonce-request bit)
   */
   uint8_t m_E : 1;

   /**
    * \brief The V-bit (Map-Version present bit)
    * Note: The N-Bit and the V-Bit cannot be set
    * simultaneously.
    */
   uint8_t m_V : 1;

   /**
    * \brief The I-bit (Instance ID bit)
    */
   uint8_t m_I : 1;

   /**
    * Reserved.
    */
   uint8_t m_flags : 3;

  /**
   *
   */
  uint32_t m_nonce;

  /**
   * \brief The LISP Locator-Status-Bits (LSBs).
   * \note 32 bits when I == 0 and 8 otherwise
   */
  uint32_t m_lsbs;

  /**
   * \brief
   */
  uint16_t m_sourceMapVersion; // 12 bist
  uint16_t m_destMapVersion; // 12 bits

  /**
   * \brief 24
   */
  uint32_t m_instanceId; // 24 bits

};

} /* namespace ns3 */

#endif /* LISP_HEADER_H */

