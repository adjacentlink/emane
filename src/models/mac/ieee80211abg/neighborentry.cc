/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2010-2012 - DRS CenGen, LLC, Columbia, Maryland
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


#include "neighborentry.h"

EMANE::Models::IEEE80211ABG::NeighborEntry::NeighborEntry():
 lastActivityTime_{},
 fEstimatedNumCommonNeighbors_{},
 fHiddenChannelActivity_{},
 fAverageHiddenRxPowerMilliWatts_{},
 fAverageCommonRxPowerMilliWatts_{}
{
  prevUtilizationTypeMap_.insert(std::make_pair(MSG_TYPE_BROADCAST_DATA,        NeighborEntry::Utilization()));
  prevUtilizationTypeMap_.insert(std::make_pair(MSG_TYPE_UNICAST_DATA,          NeighborEntry::Utilization()));
  prevUtilizationTypeMap_.insert(std::make_pair(MSG_TYPE_UNICAST_RTS_CTS_DATA,  NeighborEntry::Utilization()));
  prevUtilizationTypeMap_.insert(std::make_pair(MSG_TYPE_UNICAST_CTS_CTRL,      NeighborEntry::Utilization()));

  currUtilizationTypeMap_.insert(std::make_pair(MSG_TYPE_BROADCAST_DATA,        NeighborEntry::Utilization()));
  currUtilizationTypeMap_.insert(std::make_pair(MSG_TYPE_UNICAST_DATA,          NeighborEntry::Utilization()));
  currUtilizationTypeMap_.insert(std::make_pair(MSG_TYPE_UNICAST_RTS_CTS_DATA,  NeighborEntry::Utilization()));
  currUtilizationTypeMap_.insert(std::make_pair(MSG_TYPE_UNICAST_CTS_CTRL,      NeighborEntry::Utilization()));
}



EMANE::TimePoint 
EMANE::Models::IEEE80211ABG::NeighborEntry::getLastActivityTime() const
{
  return lastActivityTime_;
}



EMANE::Microseconds 
EMANE::Models::IEEE80211ABG::NeighborEntry::getUtilizationMicroseconds(std::uint8_t msgTypeMask) const
{
  EMANE::Microseconds result{};

  // all utilization types
  for(auto & iter : prevUtilizationTypeMap_)
   {
     // check mask
     if(iter.first & msgTypeMask)
      {
        result += iter.second.totalUtilizationMicroseconds_;
      }
   }

  return result;
}



float 
EMANE::Models::IEEE80211ABG::NeighborEntry::getHiddenChannelActivity() const
{
  return fHiddenChannelActivity_;
}



void
EMANE::Models::IEEE80211ABG::NeighborEntry::setHiddenChannelActivity(float fActivity)
{
  fHiddenChannelActivity_ = fActivity;
}



size_t 
EMANE::Models::IEEE80211ABG::NeighborEntry::getNumberOfPackets(std::uint8_t msgTypeMask) const
{
  size_t result = 0;

  // all utilization types
  for(auto & iter : prevUtilizationTypeMap_)
   {
     // check mask
     if(iter.first & msgTypeMask)
      {
        result += iter.second.totalNumPackets_;
      }
   }
 
  return result;
}



float 
EMANE::Models::IEEE80211ABG::NeighborEntry::getEstimatedNumCommonNeighbors() const
{
  return fEstimatedNumCommonNeighbors_;
}




void 
EMANE::Models::IEEE80211ABG::NeighborEntry::setEstimatedNumCommonNeighbors(float num)
{
  fEstimatedNumCommonNeighbors_ = num;
}



float
EMANE::Models::IEEE80211ABG::NeighborEntry::getAverageHiddenRxPowerMilliWatts() const
{
  return fAverageHiddenRxPowerMilliWatts_;
}



void
EMANE::Models::IEEE80211ABG::NeighborEntry::setAverageHiddenRxPowerMilliWatts(float fAverageHiddenRxPowerMilliWatts)
{
  fAverageHiddenRxPowerMilliWatts_ = fAverageHiddenRxPowerMilliWatts;
}


float
EMANE::Models::IEEE80211ABG::NeighborEntry::getAverageCommonRxPowerMilliWatts() const
{
  return fAverageCommonRxPowerMilliWatts_;
}


void
EMANE::Models::IEEE80211ABG::NeighborEntry::setAverageCommonRxPowerMilliWatts(float dAverageCommonRxPowerMilliWatts)
{
  fAverageCommonRxPowerMilliWatts_ = dAverageCommonRxPowerMilliWatts;
}


float
EMANE::Models::IEEE80211ABG::NeighborEntry::getRxPowerMilliWatts(std::uint8_t msgTypeMask) const
{
  float result{};

  for(auto & iter : prevUtilizationTypeMap_)
   {
     // check mask
     if(iter.first & msgTypeMask)
      {
        result += iter.second.fTotalRxPowerMilliWatts_;
      }
   }
 
  return result;
}



void 
EMANE::Models::IEEE80211ABG::NeighborEntry::storeUtilization()
{
  prevUtilizationTypeMap_ = currUtilizationTypeMap_;

  // all utilization types
  for(auto & iter : currUtilizationTypeMap_)
    {
      // reset entry
      iter.second.reset();
    }
}


void
EMANE::Models::IEEE80211ABG::NeighborEntry::updateChannelActivity(const EMANE::Microseconds & bandWidthMicroseconds, 
                                                                  std::uint8_t type,
                                                                  const EMANE::TimePoint & activityTime, 
                                                                  float fRxPowerMilliWatts, 
                                                                  size_t numPackets)
{
  currUtilizationTypeMap_[type].update(numPackets, bandWidthMicroseconds, fRxPowerMilliWatts);

  lastActivityTime_ = activityTime;
}



void 
EMANE::Models::IEEE80211ABG::NeighborEntry::setOneHopNeighbors(const NbrSet & nbrs)
{
  oneHopNbrSet_ = nbrs;
}



void 
EMANE::Models::IEEE80211ABG::NeighborEntry::setHiddenNeighbors(const NbrSet & nbrs)
{
  hiddenNbrSet_ = nbrs;
}



void 
EMANE::Models::IEEE80211ABG::NeighborEntry::setCommonNeighbors(const NbrSet & nbrs)
{
  commonNbrSet_ = nbrs;
}



const EMANE::Models::IEEE80211ABG::NbrSet & 
EMANE::Models::IEEE80211ABG::NeighborEntry::getOneHopNeighbors() const
{
  return oneHopNbrSet_;
}


const EMANE::Models::IEEE80211ABG::NbrSet & 
EMANE::Models::IEEE80211ABG::NeighborEntry::getCommonNeighbors() const
{
  return commonNbrSet_;
}


const EMANE::Models::IEEE80211ABG::NbrSet & 
EMANE::Models::IEEE80211ABG::NeighborEntry::getHiddenNeighbors() const
{
  return hiddenNbrSet_;
}


bool 
EMANE::Models::IEEE80211ABG::NeighborEntry::isOneHopNbr(EMANE::NEMId id) const
{
   return oneHopNbrSet_.find(id) != oneHopNbrSet_.end();
}


bool 
EMANE::Models::IEEE80211ABG::NeighborEntry::isCommonNbr(EMANE::NEMId id) const
{
   return commonNbrSet_.find(id) != commonNbrSet_.end();
}


bool 
EMANE::Models::IEEE80211ABG::NeighborEntry::isHiddenNbr(EMANE::NEMId id) const
{
   return hiddenNbrSet_.find(id) != hiddenNbrSet_.end();
}
