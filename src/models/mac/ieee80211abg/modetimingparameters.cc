/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2008-2009 - DRS CenGen, LLC, Columbia, Maryland
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 * * Neither the name of DRS CenGen, LLC nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "modetimingparameters.h"
#include "macconfig.h"
#include "macstatistics.h"
#include "maclayer.h"

#include "emane/packetinfo.h"
#include "emane/constants.h"

#include "utils.h"

namespace {
 const std::uint8_t RTS_BIT_LENGTH_DEFAULT {160};
 const std::uint8_t RTS_BIT_LENGTH_80211A  {160};
 const std::uint8_t RTS_BIT_LENGTH_80211B  {160};
 const std::uint8_t RTS_BIT_LENGTH_80211BG {160};

 const std::uint8_t CTS_BIT_LENGTH_DEFAULT {112};
 const std::uint8_t CTS_BIT_LENGTH_80211A  {112};
 const std::uint8_t CTS_BIT_LENGTH_80211B  {112};
 const std::uint8_t CTS_BIT_LENGTH_80211BG {112};

 const std::uint8_t ACK_BIT_LENGTH_DEFAULT {112};
 const std::uint8_t ACK_BIT_LENGTH_80211A  {112};
 const std::uint8_t ACK_BIT_LENGTH_80211B  {112};
 const std::uint8_t ACK_BIT_LENGTH_80211BG {112};

 const EMANE::Microseconds SLOT_TIME_USEC_DEFAULT {9};
 const EMANE::Microseconds SLOT_TIME_USEC_80211A  {9};
 const EMANE::Microseconds SLOT_TIME_USEC_80211B  {20};
 const EMANE::Microseconds SLOT_TIME_USEC_80211BG {20};

 const EMANE::Microseconds SIFS_TIME_USEC_DEFAULT {10};
 const EMANE::Microseconds SIFS_TIME_USEC_80211A  {16};
 const EMANE::Microseconds SIFS_TIME_USEC_80211B  {10};
 const EMANE::Microseconds SIFS_TIME_USEC_80211BG {16};

 const EMANE::Microseconds PREAMBLE_TIME_USEC_DEFAULT {192};
 const EMANE::Microseconds PREAMBLE_TIME_USEC_80211A  {20};
 const EMANE::Microseconds PREAMBLE_TIME_USEC_80211B  {192};
 const EMANE::Microseconds PREAMBLE_TIME_USEC_80211BG {192};


 const std::uint16_t IEEE_80211MAC_DATAHEADER_BITLEN {272};
 const std::uint16_t IEEE_80211MAC_CTRLHEADER_BITLEN {160};

 const float SLOT_TIME_DISTANCE_DIVISOR_SEC_F {299792458.0f};
}

/**
*
* @brief constructor
*
*/
EMANE::Models::IEEE80211ABG::ModeTimingParameters::ModeTimingParameters(const MACConfig & macConfig):
 macConfig_(macConfig)
{
  // default
  timingParams_[MODULATION_TYPE_DEFAULT].u16RtsBitLength_      = RTS_BIT_LENGTH_DEFAULT;
  timingParams_[MODULATION_TYPE_DEFAULT].u16CtsBitLength_      = CTS_BIT_LENGTH_DEFAULT;
  timingParams_[MODULATION_TYPE_DEFAULT].u16AckBitLength_      = ACK_BIT_LENGTH_DEFAULT;
  timingParams_[MODULATION_TYPE_DEFAULT].slotMicroseconds_     = SLOT_TIME_USEC_DEFAULT;
  timingParams_[MODULATION_TYPE_DEFAULT].sifsMicroseconds_     = SIFS_TIME_USEC_DEFAULT;
  timingParams_[MODULATION_TYPE_DEFAULT].preambleMicroseconds_ = PREAMBLE_TIME_USEC_DEFAULT;

  // 802.11a (OFDM) 
  timingParams_[MODULATION_TYPE_80211A].u16RtsBitLength_       = RTS_BIT_LENGTH_80211A;
  timingParams_[MODULATION_TYPE_80211A].u16CtsBitLength_       = CTS_BIT_LENGTH_80211A;
  timingParams_[MODULATION_TYPE_80211A].u16AckBitLength_       = ACK_BIT_LENGTH_80211A;
  timingParams_[MODULATION_TYPE_80211A].slotMicroseconds_      = SLOT_TIME_USEC_80211A;
  timingParams_[MODULATION_TYPE_80211A].sifsMicroseconds_      = SIFS_TIME_USEC_80211A;
  timingParams_[MODULATION_TYPE_80211A].preambleMicroseconds_  = PREAMBLE_TIME_USEC_80211A;

  // 802.11b (DSS) 
  timingParams_[MODULATION_TYPE_80211B].u16RtsBitLength_       = RTS_BIT_LENGTH_80211B;
  timingParams_[MODULATION_TYPE_80211B].u16CtsBitLength_       = CTS_BIT_LENGTH_80211B;
  timingParams_[MODULATION_TYPE_80211B].u16AckBitLength_       = ACK_BIT_LENGTH_80211B;
  timingParams_[MODULATION_TYPE_80211B].slotMicroseconds_      = SLOT_TIME_USEC_80211B;
  timingParams_[MODULATION_TYPE_80211B].sifsMicroseconds_      = SIFS_TIME_USEC_80211B;
  timingParams_[MODULATION_TYPE_80211B].preambleMicroseconds_  = PREAMBLE_TIME_USEC_80211B;

  // 802.11b/g (MIXED) 
  timingParams_[MODULATION_TYPE_80211BG].u16RtsBitLength_      = RTS_BIT_LENGTH_80211BG;
  timingParams_[MODULATION_TYPE_80211BG].u16CtsBitLength_      = CTS_BIT_LENGTH_80211BG;
  timingParams_[MODULATION_TYPE_80211BG].u16AckBitLength_      = ACK_BIT_LENGTH_80211BG;
  timingParams_[MODULATION_TYPE_80211BG].slotMicroseconds_     = SLOT_TIME_USEC_80211BG;
  timingParams_[MODULATION_TYPE_80211BG].sifsMicroseconds_     = SIFS_TIME_USEC_80211BG;
  timingParams_[MODULATION_TYPE_80211BG].preambleMicroseconds_ = PREAMBLE_TIME_USEC_80211BG;
}


/**
*
* @brief destructor
*
*/
EMANE::Models::IEEE80211ABG::ModeTimingParameters::~ModeTimingParameters()
{ }


/**
*
* @brief get the rts length for a given mode
*
* @param mode modulation type
*
* @retval rts length in bits
*
*/
std::uint16_t EMANE::Models::IEEE80211ABG::ModeTimingParameters::getRtsBitLength(MODULATION_TYPE mode) const
{
  switch(mode)
    {
      case MODULATION_TYPE_80211A:  // 80211.a (OFDM)
        return timingParams_[MODULATION_TYPE_80211A].u16RtsBitLength_;

      case MODULATION_TYPE_80211B:  // 802.11b (DSS)
        return timingParams_[MODULATION_TYPE_80211B].u16RtsBitLength_;

      case MODULATION_TYPE_80211BG: // 802.11b/g (MIXED)
        return timingParams_[MODULATION_TYPE_80211BG].u16RtsBitLength_; 

      default:             // default
        return timingParams_[MODULATION_TYPE_DEFAULT].u16RtsBitLength_;
    }
}


/**
*
* @brief get the cts length for a given mode
*
* @param mode modulation type
*
* @retval cts length in bits
*
*/
std::uint16_t
EMANE::Models::IEEE80211ABG::ModeTimingParameters::getCtsBitLength(MODULATION_TYPE mode) const 
{
  switch(mode)
    {
      case MODULATION_TYPE_80211A:  // 80211.a (OFDM)
        return timingParams_[MODULATION_TYPE_80211A].u16CtsBitLength_;

      case MODULATION_TYPE_80211B:  // 802.11b (DSS)
        return timingParams_[MODULATION_TYPE_80211B].u16CtsBitLength_;

      case MODULATION_TYPE_80211BG: // 802.11b/g (MIXED)
        return timingParams_[MODULATION_TYPE_80211BG].u16CtsBitLength_;

      default:             // default
        return timingParams_[MODULATION_TYPE_DEFAULT].u16CtsBitLength_;
    }
}


/**
*
* @brief get the ack length for a given mode
*
* @param mode modulation type
*
* @retval ack length in bits
*
*/
std::uint16_t
EMANE::Models::IEEE80211ABG::ModeTimingParameters::getAckBitLength(MODULATION_TYPE mode) const 
{
  switch(mode)
    {
      case MODULATION_TYPE_80211A:  // 80211.a (OFDM)
        return timingParams_[MODULATION_TYPE_80211A].u16AckBitLength_;

      case MODULATION_TYPE_80211B:  // 802.11b (DSS)
        return timingParams_[MODULATION_TYPE_80211B].u16AckBitLength_;

      case MODULATION_TYPE_80211BG: // 802.11b/g (MIXED)
        return timingParams_[MODULATION_TYPE_80211BG].u16AckBitLength_;

      default:             // default
        return timingParams_[MODULATION_TYPE_DEFAULT].u16AckBitLength_;
    }
}


/**
*
* @brief get the slot duration for a given mode and distance
*
* @retval slot time length in Microseconds
*
*/
EMANE::Microseconds
EMANE::Models::IEEE80211ABG::ModeTimingParameters::getSlotSizeMicroseconds() const 
{
  const MODULATION_TYPE mode{macConfig_.getModulationType()};

  const std::uint32_t dist{macConfig_.getMaxP2pDistance()};

  switch(mode)
    {
      case MODULATION_TYPE_80211A:  // 80211.a (OFDM)
        return timingParams_[MODULATION_TYPE_80211A].slotMicroseconds_ + getPropagationMicroseconds(dist);

      case MODULATION_TYPE_80211B:  // 802.11b (DSS)
        return timingParams_[MODULATION_TYPE_80211B].slotMicroseconds_ + getPropagationMicroseconds(dist);

      case MODULATION_TYPE_80211BG: // 802.11b/g (MIXED)
        return timingParams_[MODULATION_TYPE_80211BG].slotMicroseconds_ + getPropagationMicroseconds(dist);

      default:             // default
        return timingParams_[MODULATION_TYPE_DEFAULT].slotMicroseconds_ + getPropagationMicroseconds(dist);
    }
}


/**
*
* @brief get the sifs time for a given mode
*
* @param mode modulation type
*
* @retval sifs length in Microseconds
*
*/ 
EMANE::Microseconds
EMANE::Models::IEEE80211ABG::ModeTimingParameters::getSifsMicroseconds(MODULATION_TYPE mode) const
{
  switch(mode)
    {
      case MODULATION_TYPE_80211A:  // 80211.a (OFDM)
        return timingParams_[MODULATION_TYPE_80211A].sifsMicroseconds_;

      case MODULATION_TYPE_80211B:  // 802.11b (DSS)
        return timingParams_[MODULATION_TYPE_80211B].sifsMicroseconds_;

      case MODULATION_TYPE_80211BG: // 802.11b/g (MIXED)
        return timingParams_[MODULATION_TYPE_80211BG].sifsMicroseconds_;
  
      default:             // default
        return timingParams_[MODULATION_TYPE_DEFAULT].sifsMicroseconds_;
    }
}


/**
*
* @brief get the preamble time for a given mode
*
* @param mode modulation type
*
* @retval sifs length in Microseconds
*
*/ 
EMANE::Microseconds
EMANE::Models::IEEE80211ABG::ModeTimingParameters::getPreambleMicroseconds(MODULATION_TYPE mode) const
{
  switch(mode)
    {
      case MODULATION_TYPE_80211A:  // 80211.a (OFDM)
        return timingParams_[MODULATION_TYPE_80211A].preambleMicroseconds_;
 
      case MODULATION_TYPE_80211B:  // 802.11b (DSS)
        return timingParams_[MODULATION_TYPE_80211B].preambleMicroseconds_;

      case MODULATION_TYPE_80211BG: // 802.11b/g (MIXED)
        return timingParams_[MODULATION_TYPE_80211BG].preambleMicroseconds_;

      default:             // default
        return timingParams_[MODULATION_TYPE_DEFAULT].preambleMicroseconds_;
    }
}


/**
*
* @brief get the propagation time for a given distance
*
* @param dist distance in meters
*
* @retval propagation time in Microseconds
*
*/ 
EMANE::Microseconds
EMANE::Models::IEEE80211ABG::ModeTimingParameters::getPropagationMicroseconds(std::uint32_t dist) const
{
  return std::chrono::duration_cast<Microseconds>(DoubleSeconds{dist / SLOT_TIME_DISTANCE_DIVISOR_SEC_F});
}


/**
*
* @brief get the defer time
*
* @param u8Category queue index
*
* @retval the defer interval in Microseconds
*
*/ 
EMANE::Microseconds
EMANE::Models::IEEE80211ABG::ModeTimingParameters::getDeferIntervalMicroseconds(std::uint8_t u8Category) const
{
  const MODULATION_TYPE mode {macConfig_.getModulationType()};

  const Microseconds m{macConfig_.getAifsMicroseconds(u8Category).count() * 
                         getSlotSizeMicroseconds().count() + 
                           getSifsMicroseconds(mode).count()};

  return m;
}

/**
*
* @brief get the overhead time
*
* @param u8Category queue index
*
* @retval the overhead interval in Microseconds
*
*/ 
EMANE::Microseconds
EMANE::Models::IEEE80211ABG::ModeTimingParameters::getOverheadMicroseconds(std::uint8_t u8Category) const
{
  const MODULATION_TYPE mode {macConfig_.getModulationType()};

  DoubleSeconds doubleSeconds{(getSlotSizeMicroseconds().count() * 
                        (macConfig_.getAifsMicroseconds(u8Category).count() + (macConfig_.getCWMin(u8Category) / 2.0f)) + 
                           getSifsMicroseconds(mode).count()) / USEC_PER_SEC_F};

  return std::chrono::duration_cast<Microseconds>(doubleSeconds);
}





/**
*
* @brief check if a packet has timed out
*
* @param txOpMicroseconds packet txop time
* @param tvRxTime   packet rx time
*
* @retval return true if packet has timed out, else return false.
*
*/
bool
EMANE::Models::IEEE80211ABG::ModeTimingParameters::packetTimedOut(const Microseconds & txOpMicroseconds, 
                                                                  const TimePoint & rxTime) const
{
  // txop is enabled when non zero
  if(txOpMicroseconds.count() != 0)
    {
      // timed out if sent time plus txop is in the past
      return(rxTime + txOpMicroseconds) < Clock::now();
    }

  return false;
}




/**
*
* @brief get the message duration
*
* @param type packet type (rts, cts, ack, data)
* @param numBytes packet size in bytes
*
* @retval the transmission duration in Microseconds
*
*/
EMANE::Microseconds
EMANE::Models::IEEE80211ABG::ModeTimingParameters::getMessageDurationMicroseconds(std::uint8_t type, size_t numBytes) const
{
  switch(type)
    {
      // cts
      case MSG_TYPE_UNICAST_CTS_CTRL:
         return getCtsMessageDurationMicroseconds();

      // unicast data
      case MSG_TYPE_UNICAST_DATA:
         return getUnicastMessageDurationMicroseconds(numBytes);

      // unicast data rst-cts
      case MSG_TYPE_UNICAST_RTS_CTS_DATA:
           return getUnicastMessageDurationMicroseconds(numBytes) + 
                  getRtsMessageDurationMicroseconds() + 
                  getCtsMessageDurationMicroseconds();

      // broadcast data
      case MSG_TYPE_BROADCAST_DATA:
         return getBroadcastMessageDurationMicroseconds(numBytes);

      default:
        return Microseconds::zero();
    }
}


EMANE::Microseconds
EMANE::Models::IEEE80211ABG::ModeTimingParameters::getBroadcastMessageDurationMicroseconds(size_t numBytes) const
{
  const MODULATION_TYPE mode {macConfig_.getModulationType()};

  DoubleSeconds doubleSeconds{(getPreambleMicroseconds(mode).count() + 
                         ((IEEE_80211MAC_DATAHEADER_BITLEN + (numBytes * 8)) / 
                           (macConfig_.getBroadcastDataRateKbps() / 1000.0f))) / USEC_PER_SEC_F};

  return std::chrono::duration_cast<Microseconds>(doubleSeconds);
}



EMANE::Microseconds
EMANE::Models::IEEE80211ABG::ModeTimingParameters::getUnicastMessageDurationMicroseconds(size_t numBytes) const
{
  const MODULATION_TYPE mode {macConfig_.getModulationType()};

  DoubleSeconds doubleSeconds{(getSifsMicroseconds(mode).count() + (2 * getPreambleMicroseconds(mode).count()) +
                        ((IEEE_80211MAC_DATAHEADER_BITLEN + getAckBitLength(mode) + (numBytes * 8)) / 
                          (macConfig_.getUnicastDataRateKbps() / 1000.0f))) / USEC_PER_SEC_F};

  return std::chrono::duration_cast<Microseconds>(doubleSeconds);
}



EMANE::Microseconds
EMANE::Models::IEEE80211ABG::ModeTimingParameters::getCtsMessageDurationMicroseconds() const
{
  const MODULATION_TYPE mode {macConfig_.getModulationType()};

  DoubleSeconds doubleSeconds{(getPreambleMicroseconds(mode).count() + 
                       (IEEE_80211MAC_CTRLHEADER_BITLEN / (macConfig_.getUnicastDataRateKbps() / 1000.0f))) / USEC_PER_SEC_F};

  return std::chrono::duration_cast<Microseconds>(doubleSeconds);
}



EMANE::Microseconds
EMANE::Models::IEEE80211ABG::ModeTimingParameters::getRtsMessageDurationMicroseconds() const
{
  const MODULATION_TYPE mode {macConfig_.getModulationType()};

  DoubleSeconds doubleSeconds{(getPreambleMicroseconds(mode).count() + 
                       (IEEE_80211MAC_CTRLHEADER_BITLEN / (macConfig_.getUnicastDataRateKbps() / 1000.0f))) / USEC_PER_SEC_F};

  return std::chrono::duration_cast<Microseconds>(doubleSeconds);
}




/**
*
* @brief get the contention window
*
* @param u8Category      queue index
* @param tries    number of retires
*
* @retval cw the contention interval
*
*/ 
int
EMANE::Models::IEEE80211ABG::ModeTimingParameters::getContentionWindow(std::uint8_t u8Category, std::uint8_t tries) const
{
  const int min{macConfig_.getCWMin(u8Category)};
  const int max{macConfig_.getCWMax(u8Category)};

  int cw{min * static_cast<int>(powf(2.0f, tries))};

  if(cw > max)
    {
      cw = max;
    }
  else if(cw < min)
    {
      cw = min;
    }

  return cw;
}
