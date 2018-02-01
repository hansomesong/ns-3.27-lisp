/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005,2006 INRIA
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
 * Authors: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 *          Ghada Badawy <gbadawy@gmail.com>
 *          Sébastien Deronne <sebastien.deronne@gmail.com>
 */

#ifndef YANS_WIFI_PHY_H
#define YANS_WIFI_PHY_H

#include "wifi-phy.h"
#include "ns3/ptr.h"

namespace ns3 {

class WifiChannel;
class YansWifiChannel;
class MobilityModel;
class Channel;
class Packet;
class WifiMode;

/**
 * \brief 802.11 PHY layer model
 * \ingroup wifi
 *
 * This PHY implements a model of 802.11a. The model
 * implemented here is based on the model described
 * in "Yet Another Network Simulator",
 * (http://cutebugs.net/files/wns2-yans.pdf).
 *
 *
 * This PHY model depends on a channel loss and delay
 * model as provided by the ns3::PropagationLossModel
 * and ns3::PropagationDelayModel classes, both of which are
 * members of the ns3::YansWifiChannel class.
 */
class YansWifiPhy : public WifiPhy
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  YansWifiPhy ();
  virtual ~YansWifiPhy ();

  /**
   * Set the YansWifiChannel this YansWifiPhy is to be connected to.
   *
   * \param channel the YansWifiChannel this YansWifiPhy is to be connected to
   */
  void SetChannel (const Ptr<YansWifiChannel> channel);

  /**
   * \param packet the packet to send
   * \param txVector the TXVECTOR that has tx parameters such as mode, the transmission mode to use to send
   *        this packet, and txPowerLevel, a power level to use to send this packet. The real transmission
   *        power is calculated as txPowerMin + txPowerLevel * (txPowerMax - txPowerMin) / nTxLevels
   * \param txDuration duration of the transmission.
   */
  void StartTx (Ptr<Packet> packet, WifiTxVector txVector, Time txDuration);

  virtual Ptr<Channel> GetChannel (void) const;

  void DoInitialize ();

  void ConfigureStandard (enum WifiPhyStandard standard);

  void SetRxNoiseFigure (double noiseFigureDb);

  void SetTxPowerStart (double start);

  void SetTxPowerEnd (double end);

  void SetNTxPower (uint32_t n);

  void SetTxGain (double gain);

  void SetRxGain (double gain);

  void SetEdThreshold (double threshold);

  void SetCcaMode1Threshold (double threshold);

  void SetErrorRateModel (Ptr<ErrorRateModel> rate);

  void SetDevice (Ptr<NetDevice> device);

  void SetMobility (Ptr<MobilityModel> mobility);

  double GetRxNoiseFigure (void) const;

  double GetTxPowerStart (void) const;

  double GetTxPowerEnd (void) const;

  double GetTxGain (void) const;

  double GetRxGain (void) const;

  double GetEdThreshold (void) const;

  double GetCcaMode1Threshold (void) const;

  Ptr<ErrorRateModel> GetErrorRateModel (void) const;

  Ptr<NetDevice> GetDevice (void) const;

  Ptr<MobilityModel> GetMobility (void);

  double CalculateSnr (WifiTxVector txVector, double ber) const;

//  Ptr<WifiChannel> GetChannel (void) const;

  void SetChannelNumber (uint16_t nch);

  uint16_t GetChannelNumber (void) const;

  Time GetChannelSwitchDelay (void) const;

  double GetChannelFrequencyMhz () const;

  void AddOperationalChannel (uint16_t channelNumber);


  std::vector<uint16_t> GetOperationalChannelList () const;

  void ClearOperationalChannelList ();

  void SetSleepMode (void);

  void ResumeFromSleep (void);

  void SetReceiveOkCallback (RxOkCallback callback);

  void SetReceiveErrorCallback (RxErrorCallback callback);

  void StartReceivePreambleAndHeader (Ptr<Packet> packet,
                                              double rxPowerDbm,
                                              WifiTxVector txVector,
                                              enum WifiPreamble preamble,
                                              enum MpduType mpdutype,
                                              Time rxDuration);

  void StartReceivePacket (Ptr<Packet> packet,
                                   WifiTxVector txVector,
                                   enum WifiPreamble preamble,
                                   enum MpduType mpdutype,
                                   Ptr<InterferenceHelper::Event> event);

  void SendPacket (Ptr<const Packet> packet, WifiTxVector txVector, WifiPreamble preamble);
  void SendPacket (Ptr<const Packet> packet, WifiTxVector txVector, MpduType mpdutype = NORMAL_MPDU);
  void SendPacket (Ptr<const Packet> packet, WifiTxVector txVector, WifiPreamble preamble, enum MpduType mpdutype);

  uint32_t GetNModes (void) const;

  WifiMode GetMode (uint32_t mode) const;

  bool IsModeSupported (WifiMode mode) const;

  bool IsMcsSupported (WifiMode mcs);

  uint32_t GetNTxPower (void) const;

  void Configure80211a (void);

  void Configure80211b (void);

  void Configure80211g (void);

  void Configure80211_10Mhz (void);

  void Configure80211_5Mhz (void);

  void ConfigureHolland (void);


  void ConfigureHtDeviceMcsSet (void);


  void Configure80211n (void);

  void Configure80211ac (void);

  void RegisterListener (WifiPhyListener *listener);

  void UnregisterListener (WifiPhyListener *listener);

  bool IsStateCcaBusy (void);

  bool IsStateIdle (void);

  bool IsStateBusy (void);

  bool IsStateRx (void);

  bool IsStateTx (void);

  bool IsStateSwitching (void);

  bool IsStateSleep (void);

  Time GetStateDuration (void);

  Time GetDelayUntilIdle (void);

  Time GetLastRxStartTime (void) const;

  double GetEdThresholdW (void) const;


  double GetPowerDbm (uint8_t power) const;

  void EndReceive (Ptr<Packet> packet, enum WifiPreamble preamble, enum MpduType mpdutype, Ptr<InterferenceHelper::Event> event);

  int64_t AssignStreams (int64_t stream);

  void SetFrequency (uint32_t freq);

  void SetNumberOfTransmitAntennas (uint32_t tx);

  void SetNumberOfReceiveAntennas (uint32_t rx);

  void SetLdpc (bool Ldpc);

  void SetStbc (bool stbc);

  void SetGreenfield (bool greenfield);

  bool GetGuardInterval (void) const;

  void SetGuardInterval (bool guardInterval);

  uint32_t GetFrequency (void) const;

  uint32_t GetNumberOfTransmitAntennas (void) const;

  uint32_t GetNumberOfReceiveAntennas (void) const;

  bool GetLdpc (void) const;

  bool GetStbc (void) const;

  bool GetGreenfield (void) const;

  bool GetShortPlcpPreambleSupported (void) const;

  void SetShortPlcpPreambleSupported (bool enable);

  void SetChannelWidth (uint32_t channelwidth);

  uint32_t GetChannelWidth (void) const;

  uint8_t GetSupportedRxSpatialStreams (void) const;

  uint8_t GetSupportedTxSpatialStreams (void) const;

  void AddSupportedChannelWidth (uint32_t width);

  std::vector<uint32_t> GetSupportedChannelWidthSet (void) const;

  uint32_t GetNBssMembershipSelectors (void) const ;

  uint32_t GetBssMembershipSelector (uint32_t selector) const;

  WifiModeList GetMembershipSelectorModes (uint32_t selector);

  uint8_t GetNMcs (void) const;

  WifiMode GetMcs (uint8_t mcs) const;


protected:
  // Inherited
  virtual void DoDispose (void);


private:
  Ptr<YansWifiChannel> m_channel; //!< YansWifiChannel that this YansWifiPhy is connected to

  double m_channelStartingFrequency;

};

} //namespace ns3

#endif /* YANS_WIFI_PHY_H */
