/*
 * Copyright (c) 2013,2016 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "neighbormanager.h"
#include "maclayer.h"
#include "utils.h"
#include "msgtypes.h"

#include "emane/constants.h"

#include <sstream>


namespace {
 const char * pzLayerName {"NeighborManager"};
}


EMANE::Models::IEEE80211ABG::NeighborManager::NeighborManager(NEMId id,
                                                PlatformServiceProvider * pPlatformService,
                                                MACLayer *pMACLayer):
  id_{id},
  pPlatformService_{pPlatformService},
  pMACLayer_{pMACLayer},
  wmmManager_{id, pPlatformService, pMACLayer},
  nbrTimeOutMicroseconds_{},
  lastOneHopNbrListTxTime_{},
  RNDZeroToOne_{0.0f, 1.0f},
  lastResetTime_{}
{
   // reset counters
   resetCounters_i();

   utilizationRatioVector_ = wmmManager_.getUtilizationRatios(Microseconds::zero());
}



EMANE::Models::IEEE80211ABG::NeighborManager::~NeighborManager()
{ }


void
EMANE::Models::IEEE80211ABG::NeighborManager::setCategories(std::uint8_t u8NumCategories)
{
   wmmManager_.setNumCategories(u8NumCategories);
}


float
EMANE::Models::IEEE80211ABG::NeighborManager::getHiddenChannelActivity(NEMId src) const
{
  // find the src in the one hop nbr(s)
  auto nbrEntry = oneHopNbrMap_.find(src);

  float fResult{};

  if(nbrEntry != oneHopNbrMap_.end())
   {
     fResult = nbrEntry->second.getHiddenChannelActivity();
   }

  return fResult;
}



float
EMANE::Models::IEEE80211ABG::NeighborManager::getNumberOfEstimatedCommonNeighbors(NEMId src) const
{
  // find the src
  auto nbrEntry = oneHopNbrMap_.find(src);

  float fResult{};

  if(nbrEntry != oneHopNbrMap_.end())
   {
     fResult = nbrEntry->second.getEstimatedNumCommonNeighbors();
   }

  return fResult;
}


float
EMANE::Models::IEEE80211ABG::NeighborManager::getNumberOfEstimatedHiddenNeighbors(NEMId src) const
{
  // find the src
  auto nbrEntry = oneHopNbrMap_.find(src);

  float fResult{};

  if(nbrEntry != oneHopNbrMap_.end())
   {
     // set result
     fResult = fEstimatedNumOneHopNeighbors_ - nbrEntry->second.getEstimatedNumCommonNeighbors();
   }

  // min allowed is 0
  if(fResult < 0.0f)
   {
     fResult = 0.0f;
   }

  return fResult;
}



EMANE::Microseconds
EMANE::Models::IEEE80211ABG::NeighborManager::getAllUtilizationMicroseconds(NEMId src) const
{
  // find the src
  auto nbrEntry = oneHopNbrMap_.find(src);

  if(nbrEntry != oneHopNbrMap_.end())
   {
     return nbrEntry->second.getUtilizationMicroseconds(MSG_TYPE_MASK_ALL_DATA);
   }
  else
   {
     return Microseconds::zero();
   }
}



float
EMANE::Models::IEEE80211ABG::NeighborManager::getNumberOfEstimatedOneHopNeighbors() const
{
  return fEstimatedNumOneHopNeighbors_;
}



float
EMANE::Models::IEEE80211ABG::NeighborManager::getNumberOfEstimatedTwoHopNeighbors() const
{
  return fEstimatedNumTwoHopNeighbors_;
}



EMANE::Microseconds
EMANE::Models::IEEE80211ABG::NeighborManager::getTotalOneHopUtilizationMicroseconds() const
{
  return totalOneHopUtilizationMicroseconds_;
}



EMANE::Microseconds
EMANE::Models::IEEE80211ABG::NeighborManager::getTotalTwoHopUtilizationMicroseconds() const
{
  return totalTwoHopUtilizationMicroseconds_;
}



EMANE::Microseconds
EMANE::Models::IEEE80211ABG::NeighborManager::getAverageMessageDurationMicroseconds() const
{
  return averageMessageDurationMicroseconds_;
}




float
EMANE::Models::IEEE80211ABG::NeighborManager::getLocalNodeTx() const
{
   return fLocalNodeTx_;
}



size_t
EMANE::Models::IEEE80211ABG::NeighborManager::getTotalActiveOneHopNeighbors() const
{
   return numTotalActiveOneHopNeighbors_;
}



float
EMANE::Models::IEEE80211ABG::NeighborManager::getAverageRxPowerPerMessageMilliWatts() const
{
  return fAverageRxPowerPerMessageMilliWatts_;
}



float
EMANE::Models::IEEE80211ABG::NeighborManager::getAverageRxPowerPerMessageHiddenNodesMilliWatts() const
{
  if(sumHiddenPackets_ > 0)
   {
     return fHiddenRxPowerMilliWatts_ / sumHiddenPackets_;
   }
  else
   {
     return 0.0;
   }
}



float
EMANE::Models::IEEE80211ABG::NeighborManager::getAverageRxPowerPerMessageCommonNodesMilliWatts() const
{
  if(sumCommonPackets_ > 0)
   {
     return fCommonRxPowerMilliWatts_ / sumCommonPackets_;
   }
  else
   {
     return 0.0;
   }
}



float
EMANE::Models::IEEE80211ABG::NeighborManager::getRandomRxPowerCommonNodesMilliWatts(NEMId src)
{
  // get random probability
  const float fRandom{RNDZeroToOne_()};

  // get result using common info
  const float fResult{getRandomRxPowerMilliWatts_i(src, fRandom, commonProbabilityMapMap_, commonNbrAvgRxPowerMwMap_)};

  LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                         DEBUG_LEVEL,
                         "MACI %03hu %s::%s: src %hu, random %5.4f, result %5.4f",
                         id_,
                         pzLayerName,
                         __func__,
                         src,
                         fRandom,
                         fResult);

  return fResult;
}



float
EMANE::Models::IEEE80211ABG::NeighborManager::getRandomRxPowerHiddenNodesMilliWatts(NEMId src)
{
  // get random probability
  const float fRandom{RNDZeroToOne_()};

  // get result using hidden info
  const float fResult{getRandomRxPowerMilliWatts_i(src,
                                                     fRandom,
                                                     hiddenProbabilityMapMap_,
                                                     hiddenNbrAvgRxPowerMwMap_)};

  LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                         DEBUG_LEVEL,
                         "MACI %03hu %s::%s: src %hu, random %5.4f, result %6.4f mW",
                         id_,
                         pzLayerName,
                         __func__,
                         src,
                         fRandom,
                         fResult);

  // return result
  return fResult;
}



float
EMANE::Models::IEEE80211ABG::NeighborManager::getRandomRxPowerMilliWatts_i(NEMId src,
                                                                           float fRandom,
                                                                           const ProbabilityPairMapMap & pmap,
                                                                           const RxPowerMap & rmap) const
{
  // result
  float fResult{};

  // lookup probabilty pair map
  auto pmmiter = pmap.find(src);

  // entry found
  if(pmmiter != pmap.end())
   {
     // for each common probabilty entry
     for(auto & piter : pmmiter->second)
      {
        // search probability ranges
        if((fRandom >= piter.second.first) && (fRandom <= piter.second.second))
         {
           // lookup common nbr
           auto riter = rmap.find(piter.first);

           // common nbr found
           if(riter != rmap.end())
            {
               // set result
               fResult = riter->second;
            }

            // done
            break;
         }
      }
   }

  LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                         DEBUG_LEVEL,
                         "MACI %03hu %s::%s: src %hu, random %5.4f, result %6.4f",
                         id_,
                         pzLayerName,
                         __func__,
                         src,
                         fRandom,
                         fResult);

  return fResult;
}



EMANE::TimePoint
EMANE::Models::IEEE80211ABG::NeighborManager::getLastOneHopNbrListTxTime() const
{
  return lastOneHopNbrListTxTime_;
}



void
EMANE::Models::IEEE80211ABG::NeighborManager::setNeighborTimeoutMicroseconds(const Microseconds & timeOutMicroseconds)
{
  nbrTimeOutMicroseconds_ = timeOutMicroseconds;
}

void
EMANE::Models::IEEE80211ABG::NeighborManager::start()
{
  lastResetTime_ = Clock::now();
}

void
EMANE::Models::IEEE80211ABG::NeighborManager::resetStatistics()
{
  // current time
  const TimePoint currentTime{Clock::now()};

  Microseconds deltaTMicroseconds{std::chrono::duration_cast<Microseconds>(currentTime - lastResetTime_)};

  // nbr timeout enabled
  if(nbrTimeOutMicroseconds_ != Microseconds::zero())
   {
     // time out old one hop nbrs
     if(flushOneHopNeighbors_i(currentTime, nbrTimeOutMicroseconds_) == true)
      {
        // nbr table changed, send event
        sendOneHopNbrListEvent_i();
      }

     // time out old two hop nbrs
     if(flushTwoHopNeighbors_i(currentTime, nbrTimeOutMicroseconds_) == true)
      {
        // dont care
      }
   }

  // for each one hop nbr
  for(auto & nbrEntry :oneHopNbrMap_)
    {
      // store history and reset all utilizations
      nbrEntry.second.storeUtilization();
    }

  // calculate bw utilization
  calculateBwUtilization_i(deltaTMicroseconds);

  // for each two hop nbr
  for(auto & nbr2Entry : twoHopNbrMap_)
    {
      // reset utilization
      nbr2Entry.second.resetUtilization();
    }

  // get wmm bandiwdth utilization ratio(s)
  utilizationRatioVector_ = wmmManager_.getUtilizationRatios(deltaTMicroseconds);

  lastResetTime_ = currentTime;
}





void
EMANE::Models::IEEE80211ABG::NeighborManager::updateDataChannelActivity(NEMId src,
                                                                        std::uint8_t type,
                                                                        float fRxPowerMilliWatts,
                                                                        const TimePoint & timePoint,
                                                                        const Microseconds & durationMicroseconds,
                                                                        std::uint8_t u8Category)
{
  bool changed{false};

  // add/update one hop nbr
  auto nbrResult = addOneHopNeighbor_i(src);

  // new nbr
  if(nbrResult.second == true)
   {
     // one hop nbr table changed
     changed = true;
   }

  // udpate the duration, last activity time and rx power
  nbrResult.first->second.updateChannelActivity(durationMicroseconds, type, timePoint, fRxPowerMilliWatts);

  if(src == id_)
   {
     wmmManager_.updateLocalActivity(u8Category, durationMicroseconds);
   }
  else
   {
     wmmManager_.updateTotalActivity(u8Category, durationMicroseconds);
   }


  // nbr table changed
  if(changed == true)
   {
     // send one hop nbr list event
     sendOneHopNbrListEvent_i();
   }
}




void
EMANE::Models::IEEE80211ABG::NeighborManager::updateCtrlChannelActivity(NEMId src,
                                                                        NEMId origin,
                                                                        std::uint8_t type,
                                                                        float fRxPowerMilliWatts,
                                                                        const TimePoint & timePoint,
                                                                        const Microseconds & durationMicroseconds,
                                                                        std::uint8_t u8Category)
{
  bool changed{false};

  // check unicast rts/cts origin is not us
  if(origin != id_)
   {
     // rts/cts origin not found
     if(oneHopNbrMap_.find(origin) == oneHopNbrMap_.end())
      {
        // add two hop nbr
        auto nbr2Result = addTwoHopNeighbor_i(origin);

        // udpate the duration (unicast msg), last activity time
        nbr2Result.first->second.updateChannelActivity(durationMicroseconds, timePoint);
      }
   }

  // add/update one hop nbr
  auto nbrResult = addOneHopNeighbor_i(src);

  // new nbr
  if(nbrResult.second == true)
   {
     // one hop nbr table changed
     changed = true;
   }

  // do not include the duration, just set last activity time and rx power
  nbrResult.first->second.updateChannelActivity(Microseconds::zero(), type, timePoint, fRxPowerMilliWatts);

  if(src == id_)
   {
     wmmManager_.updateLocalActivity(u8Category, durationMicroseconds);
   }
  else
   {
     wmmManager_.updateTotalActivity(u8Category, durationMicroseconds);
   }

  // nbr table changed
  if(changed == true)
   {
     // send one hop nbr list event
     sendOneHopNbrListEvent_i();
   }
}




void
EMANE::Models::IEEE80211ABG::NeighborManager::handleOneHopNeighborsEvent(const Serialization &serialization)
{
  try
   {
     OneHopNeighborsEvent event{serialization};

     NbrSet nbrSet{event.getNeighbors()};

     NEMId eventSource{event.getEventSource()};

     // bump num rx events
     pMACLayer_->getStatistics().incrementRxOneHopNbrListEventCount();

     // search for one hop nbr
     auto nbrEntry = oneHopNbrMap_.find(eventSource);

    // src is an active one hop nbr
    if(nbrEntry != oneHopNbrMap_.end())
     {
       LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                              DEBUG_LEVEL,
                              "MACI %03hu %s::%s: update 1hop_nbr_list for nbr %hu, %zd entries",
                              id_,
                              pzLayerName,
                              __func__,
                              eventSource,
                              nbrSet.size());

      // set one hop nbrs of this one hop nbr
      nbrEntry->second.setOneHopNeighbors(nbrSet);
    }
   else
    {
      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                             DEBUG_LEVEL,
                             "MACI %03hu %s::%s: unknown nbr %hu, 1hop_nbr_list with %zd entries will be cached",
                             id_,
                             pzLayerName,
                             __func__,
                             eventSource,
                             nbrSet.size());
   }

   // we want to keep a cache of events in case of unknown or timed out nodes
   // find src of this event in the one hop nbr list event cache
   auto iter = cachedOneHopNbrSetMap_.find(eventSource);

   // not already cached
   if(iter == cachedOneHopNbrSetMap_.end())
    {
      // insert into cache
      cachedOneHopNbrSetMap_[eventSource] = nbrSet;

      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                             DEBUG_LEVEL,
                             "MACI %03hu %s::%s: added event cache 1hop_nbr_list for nbr %hu, %zd entries",
                             id_,
                             pzLayerName,
                             __func__,
                             eventSource,
                             nbrSet.size());
    }
  else
    {
      // update cache
      iter->second = nbrSet;

      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                             DEBUG_LEVEL,
                             "MACI %03hu %s::%s: updated event cache 1hop_nbr_list for nbr %hu, %zd entries",
                             id_,
                             pzLayerName,
                             __func__,
                             eventSource,
                             nbrSet.size());
    }

   }
  catch(SerializationException &ex)
   {
     LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                             ERROR_LEVEL,
                             "MACI %03hu %s::%s: 1hop neighbors event error %s",
                             id_,
                             pzLayerName,
                             __func__,
                             ex.what());

     // bump num rx invalid events
     pMACLayer_->getStatistics().incrementRxOneHopNbrListInvalidEventCount();
   }
}


EMANE::Models::IEEE80211ABG::WMMManager::UtilizationRatioVector
EMANE::Models::IEEE80211ABG::NeighborManager::getUtilizationRatios()
{
  return utilizationRatioVector_;
}




void
EMANE::Models::IEEE80211ABG::NeighborManager::calculateBwUtilization_i(const Microseconds & deltaTMicroseconds)
{
  // internal call

  // reset counters
  resetCounters_i();

  // each one hop nbr
  for(auto & nbrEntry : oneHopNbrMap_)
    {
       // get bandwidth utilization all DATA msg types
       Microseconds utilizationMicroseconds{nbrEntry.second.getUtilizationMicroseconds(MSG_TYPE_MASK_ALL_DATA)};

       // check utilization
       if(utilizationMicroseconds > Microseconds::zero())
        {
          // sum total one hop bandwidth utilization
          totalOneHopUtilizationMicroseconds_ += utilizationMicroseconds;

          // store the one hop utilization
          oneHopUtilizationMap_[nbrEntry.first] = utilizationMicroseconds;

          LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                 DEBUG_LEVEL,
                                 "MACI %03hu %s::%s: 1hop_nbr %hu, 1hop_bw %lf, total_1hop_bw %lf",
                                 id_,
                                 pzLayerName,
                                 __func__,
                                 nbrEntry.first,
                                 std::chrono::duration_cast<DoubleSeconds>(utilizationMicroseconds).count(),
                                 std::chrono::duration_cast<DoubleSeconds>(totalOneHopUtilizationMicroseconds_).count());

          // this nem
          if(nbrEntry.first == id_)
           {
             // set this nem bw utilization
             utilizationThisNEMMicroseconds_ = utilizationMicroseconds;
           }
          // other nem(s)
          else
           {
             // sum number of one hop packets all DATA msg types
             totalOneHopNumPackets_ += nbrEntry.second.getNumberOfPackets(MSG_TYPE_MASK_ALL_DATA);

             // sum one hop rx power all DATA msg types
             fTotalRxPowerMilliWatts_ += nbrEntry.second.getRxPowerMilliWatts(MSG_TYPE_MASK_ALL_DATA);
           }
        }
    }

  // set the local node Tx value avoid / by 0
  if(totalOneHopUtilizationMicroseconds_ != Microseconds::zero())
   {
      fLocalNodeTx_ = getRatio(utilizationThisNEMMicroseconds_, totalOneHopUtilizationMicroseconds_);
   }
  else
   {
      fLocalNodeTx_ = 0.0;
   }

#ifdef VERY_VERBOSE_LOGGING
  LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                         DEBUG_LEVEL,
                         "MACI %03hu %s::%s: %zd active 1hop_nbrs, "
                         "1hop_total_bw %lf, total_1hop_pkts %zd, total_1hop_pwr %5.4f mW, local tx %5.4f",
                         id_,
                         pzLayerName,
                         __func__,
                         oneHopUtilizationMap_.size(),
                         std::chrono::duration_cast<DoubleSeconds>(totalOneHopUtilizationMicroseconds_).count(),
                         totalOneHopNumPackets_,
                         fTotalRxPowerMilliWatts_,
                         fLocalNodeTx_);
#endif

  // each two hop nbr
  for(const auto & nbr2Entry : twoHopNbrMap_)
    {
       // get bandwidth utilization

       const Microseconds utilizationMicroseconds{nbr2Entry.second.getUtilizationMicroseconds()};

       // check any utilization
       if(utilizationMicroseconds > Microseconds::zero())
        {
          // store the two hop utilization
          twoHopUtilizationMap_[nbr2Entry.first] = utilizationMicroseconds;

          // sum total 2 hop bandwidth utilization
          totalTwoHopUtilizationMicroseconds_ += utilizationMicroseconds;

          // sum number of 2 hop packets
          totalTwoHopNumPackets_ += nbr2Entry.second.getNumberOfPackets();

          LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                 DEBUG_LEVEL,
                                 "MACI %03hu %s::%s: 2hop_nbr %hu, 2hop_bw %lf, total_2hop_bw %lf",
                                 id_,
                                 pzLayerName,
                                 __func__,
                                 nbr2Entry.first,
                                 std::chrono::duration_cast<DoubleSeconds>(utilizationMicroseconds).count(),
                                 std::chrono::duration_cast<DoubleSeconds>(totalTwoHopUtilizationMicroseconds_).count());

        }
    }

#ifdef VERY_VERBOSE_LOGGING
  LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                         DEBUG_LEVEL,
                         "MACI %03hu %s::%s: %zd active 2hop_nbrs, total_2hop_bw %lf, total_pkts %zd",
                         id_,
                         pzLayerName,
                         __func__,
                         twoHopUtilizationMap_.size(),
                         std::chrono::duration_cast<DoubleSeconds>(totalTwoHopUtilizationMicroseconds_).count(),
                         totalTwoHopNumPackets_);
#endif

  // check two hop packet count, prevents / by 0
  if(totalTwoHopNumPackets_ > 0)
    {
      // sum B
      float B{};

      // average bandwidth utilization per active two hop nbr
      averageUtilizationPerTwoHopNbrMicroseconds_ =
         std::chrono::duration_cast<Microseconds>(DoubleSeconds{((totalTwoHopUtilizationMicroseconds_.count() *
            (1.0f / twoHopUtilizationMap_.size())) / USEC_PER_SEC_F)});

      // each two hop utilization entry
      for(const auto & twoHopUtilization : twoHopUtilizationMap_)
        {
          // get A
          const float A = getA_i(twoHopUtilization.second, averageUtilizationPerTwoHopNbrMicroseconds_);

          // sum A^2
          B += powf(A, 2.0f);

          LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                 DEBUG_LEVEL,
                                 "MACI %03hu %s::%s: 2hop A = %5.4f, B = %5.4f",
                                 id_,
                                 pzLayerName,
                                 __func__,
                                 A, B);
        }

      // calculate the total estimated number of two hop nbrs
      fEstimatedNumTwoHopNeighbors_ = round(B);
    }


  // check one hop packet count, prevents / by 0
  if(totalOneHopNumPackets_ > 0)
    {
      // set num active nbrs, may include us
      numTotalActiveOneHopNeighbors_ = oneHopUtilizationMap_.size();

      // average bandwidth utilization per active one hop nbr
      averageUtilizationPerOneHopNbrMicroseconds_ =
        std::chrono::duration_cast<Microseconds>(DoubleSeconds{((totalOneHopUtilizationMicroseconds_.count() *
           (1.0f / numTotalActiveOneHopNeighbors_)) / USEC_PER_SEC_F)});

      // average rx power per packet
      fAverageRxPowerPerMessageMilliWatts_ = fTotalRxPowerMilliWatts_ / totalOneHopNumPackets_;

      // average message duration (not including this nem)
      averageMessageDurationMicroseconds_ =
          std::chrono::duration_cast<Microseconds>(DoubleSeconds{(((totalOneHopUtilizationMicroseconds_.count() -
             utilizationThisNEMMicroseconds_.count()) * (1.0f / totalOneHopNumPackets_)) / USEC_PER_SEC_F)});

      float A1{}, A2{}, B1{};

      // each one hop nbr
      for(auto & nbrEntry1 : oneHopNbrMap_)
        {
          // check nbr activity all DATA msg types
          if(nbrEntry1.second.getNumberOfPackets(MSG_TYPE_MASK_ALL_DATA) > 0)
           {
             // get A
             A1 = getA_i(nbrEntry1.second.getUtilizationMicroseconds(MSG_TYPE_MASK_ALL_DATA),
                            averageUtilizationPerOneHopNbrMicroseconds_);

             B1 += powf(A1, 2.0f);

             // check nbr is not us
             if(nbrEntry1.first != id_)
              {
                float B2{};

                // the sum of hidden bandwidth utilization
                Microseconds hiddenUtilizationMicroseconds{};

                // the sum of common rx power in mW
                float fCommonRxPowerMilliWatts{};

                // the sum of hidden rx power in mW
                float fHiddenRxPowerMilliWatts{};

                // the sum of common tx pkts
                size_t numCommonPackets{};

                // the sum of hidden tx pkts
                size_t numHiddenPackets{};

                // get the one hop nbrs of this nbr
                const NbrSet nbrOneHopNbrSet = nbrEntry1.second.getOneHopNeighbors();

#ifdef VERBOSE_LOGGIN
                std::stringstream ss;

                // list the nbrs of this nbr
                for(auto & nbr : nbrOneHopNbrSet)
                 {
                   if(nbr == nbrOneHopNbrSet.begin())
                    {
                      ss << *nbr;
                    }
                   else
                    {
                      ss << ", " << *nbr;
                    }
                 }

                LOGGER_VERBOSE_LOGGING(pPlatformService_,
                                       DEBUG_LEVEL,
                                       "MACI %03hu %s::%s: nbr %hu has %zu 1hop_nbrs: %s",
                                       id_,
                                       pzLayerName,
                                       __func__,
                                       nbrEntry1.first,
                                       nbrOneHopNbrSet.size(),
                                        ss.str().c_str());
#endif

                // nbrs hidden from this nbr
                NbrSet hiddenNbrs;

                // nbrs common with this nbr
                NbrSet commonNbrs;

                // each one hop nbr (again)
                for(auto & nbrEntry2 : oneHopNbrMap_)
                 {
                    // does the nbr's one hop nbr set include our nbr
                    if(nbrOneHopNbrSet.find(nbrEntry2.first) != nbrOneHopNbrSet.end())
                     {
                        // get all DATA msg types
                        const int typeMask{MSG_TYPE_MASK_ALL_DATA};

                        A2 = getA_i(nbrEntry2.second.getUtilizationMicroseconds(typeMask),
                                    averageUtilizationPerOneHopNbrMicroseconds_);

                        B2 +=powf(A2, 2.0f);

                        // get common rx power
                        const float fRxPowermW{nbrEntry2.second.getRxPowerMilliWatts(typeMask)};

                        // get the common number of packets
                        const size_t numPackets{nbrEntry2.second.getNumberOfPackets(typeMask)};

                        LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                               DEBUG_LEVEL,
                                               "MACI %03hu %s::%s: our nbr %hu is common with nbr %hu, pkts %zd, rxpwr %6.4f mW",
                                                id_,
                                                pzLayerName,
                                                __func__,
                                                nbrEntry2.first,
                                                nbrEntry1.first,
                                                numPackets,
                                                fRxPowermW);

                        // add to common nbrs
                        commonNbrs.insert(nbrEntry2.first);

                        // save common rx power avoid / by 0
                        commonNbrAvgRxPowerMwMap_[nbrEntry2.first] = numPackets > 0 ? fRxPowermW / numPackets : 0;

                        // sum the common rx power
                        fCommonRxPowerMilliWatts += fRxPowermW;

                        // sum the common number of packets
                        numCommonPackets += numPackets;
                     }
                    // hidden from this nbr
                    else
                     {
                        // get all unicast and broadcast msg types
                        const int typeMask {MSG_TYPE_MASK_UNICAST | MSG_TYPE_MASK_BROADCAST};

                        // sum the hidden bandwidth utilization
                        hiddenUtilizationMicroseconds += nbrEntry2.second.getUtilizationMicroseconds(typeMask);

                        // get hidden rx power
                        const float fRxPowermW{nbrEntry2.second.getRxPowerMilliWatts(typeMask)};

                        // get the hidden number of packets
                        const size_t numPackets{nbrEntry2.second.getNumberOfPackets(typeMask)};

                        LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                               DEBUG_LEVEL,
                                               "MACI %03hu %s::%s: nbr %hu is hidden from nbr %hu, pkts %zd, rxpwr %6.4f mW",
                                               id_,
                                               pzLayerName,
                                               __func__,
                                               nbrEntry2.first,
                                               nbrEntry1.first,
                                               numPackets,
                                               fRxPowermW);

                        // add to hidden nbrs
                        hiddenNbrs.insert(nbrEntry2.first);

                        // save hidden rx power avoid / by 0
                        hiddenNbrAvgRxPowerMwMap_[nbrEntry2.first] = numPackets > 0 ? fRxPowermW / numPackets : 0;

                        // sum the hidden rx power
                        fHiddenRxPowerMilliWatts += fRxPowermW;

                        // sum the hidden number of packets
                        numHiddenPackets += numPackets;
                     }
                 }

                 // set the common nbrs
                 nbrEntry1.second.setCommonNeighbors(commonNbrs);

                 // set the hidden nbrs
                 nbrEntry1.second.setHiddenNeighbors(hiddenNbrs);

                 // get the hidden channel activity
                 const float H{getH_i(hiddenUtilizationMicroseconds, deltaTMicroseconds)};

                 LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                        DEBUG_LEVEL,
                                        "MACI %03hu %s::%s: 1hop_nbr %hu, A1 = %5.4f, A2 = %5.4f, B1 = %5.4f, B2 = %5.4f, H = %5.4f",
                                        id_,
                                        pzLayerName,
                                         __func__,
                                        nbrEntry1.first,
                                        A1, A2, B1, B2, H);

                 // set the estimated number of common nbrs this nbr
                 nbrEntry1.second.setEstimatedNumCommonNeighbors(round(B2));

                 // set the avg common rx power this nbr, avoid / by 0
                 nbrEntry1.second.setAverageCommonRxPowerMilliWatts(numCommonPackets > 0.0f ?
                   fCommonRxPowerMilliWatts / numCommonPackets : 0.0f);

                 // set the hidden channel activity this nbr
                 nbrEntry1.second.setHiddenChannelActivity(H);

                 // set the avg hidden rx power this nbr, avoid / by 0
                 nbrEntry1.second.setAverageHiddenRxPowerMilliWatts(numHiddenPackets > 0.0f ?
                   fHiddenRxPowerMilliWatts / numHiddenPackets : 0.0f);

                 // set the overall sum of common pkts
                 sumCommonPackets_ += numCommonPackets;

                 // set the overall sum of hidden pkts
                 sumHiddenPackets_ += numHiddenPackets;

                 // set the overall pwr of common pkts
                 fCommonRxPowerMilliWatts_ += fCommonRxPowerMilliWatts;

                 // set the overall pwr of hidden pkts
                 fHiddenRxPowerMilliWatts_ += fHiddenRxPowerMilliWatts;
              }
           }
        }

      // set common and hidden probability
      setCommonAndHiddenProbability_i();

      // calculate the total estimated number of one hop nbrs
      fEstimatedNumOneHopNeighbors_ = round(B1);

      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                             DEBUG_LEVEL,
                             "MACI %03hu %s::%s: est nbrs [1hop %5.4f, 2hop %5.4f], active nbrs %zd, elapsed time %lf",
                             id_,
                             pzLayerName,
                             __func__,
                             fEstimatedNumOneHopNeighbors_,
                             fEstimatedNumTwoHopNeighbors_,
                             numTotalActiveOneHopNeighbors_,
                             std::chrono::duration_cast<DoubleSeconds>(deltaTMicroseconds).count());
    }
}



void
EMANE::Models::IEEE80211ABG::NeighborManager::resetCounters_i()
{
  // internal call

  fLocalNodeTx_ = 0.0f;

  totalOneHopNumPackets_ = 0;

  totalTwoHopNumPackets_ = 0;

  sumCommonPackets_ = 0;

  sumHiddenPackets_ = 0;

  numTotalActiveOneHopNeighbors_ = 0;

  fTotalRxPowerMilliWatts_ = 0.0f;

  fEstimatedNumOneHopNeighbors_ = 0.0f;

  fEstimatedNumTwoHopNeighbors_ = 0.0f;

  fAverageRxPowerPerMessageMilliWatts_ = 0.0f;

  fHiddenRxPowerMilliWatts_ = 0.0f;

  fCommonRxPowerMilliWatts_ = 0.0f;

  averageMessageDurationMicroseconds_ = Microseconds::zero();

  totalOneHopUtilizationMicroseconds_ = Microseconds::zero();

  totalTwoHopUtilizationMicroseconds_ = Microseconds::zero();

  averageUtilizationPerOneHopNbrMicroseconds_ = Microseconds::zero();

  averageUtilizationPerTwoHopNbrMicroseconds_ = Microseconds::zero();

  utilizationThisNEMMicroseconds_ = Microseconds::zero();

  oneHopUtilizationMap_.clear();

  twoHopUtilizationMap_.clear();
}



bool
EMANE::Models::IEEE80211ABG::NeighborManager::flushOneHopNeighbors_i(const TimePoint & currentTime,
                                                                     const Microseconds & timeOutMicroseconds)
{
  // internal call

  // number of expired entries
  size_t numExpired{};

  for(NeighborEntryMap::iterator iter = oneHopNbrMap_.begin(); iter != oneHopNbrMap_.end(); /* bump/erase below */)
    {
      // nbr age
      const Microseconds ageMicroseconds =
        std::chrono::duration_cast<Microseconds>(currentTime - iter->second.getLastActivityTime());

      // nbr timed out
      if(ageMicroseconds > timeOutMicroseconds)
        {
          // log first, then remove
          LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                 DEBUG_LEVEL,
                                 "MACI %03hu %s::%s: remove 1hop_nbr %hu, age %lf, %zd nbrs remaining",
                                 id_,
                                 pzLayerName,
                                 __func__,
                                 iter->first,
                                 std::chrono::duration_cast<DoubleSeconds>(ageMicroseconds).count(),
                                 oneHopNbrMap_.size() - 1);

          pStatisticOneHopNbrTable_->deleteRow(iter->first);

          // remove entry and bump
          oneHopNbrMap_.erase(iter++);

          // bump num expired
          ++numExpired;
        }
      else
        {
          // bump
          ++iter;
        }
     }

   // return expired status
   return numExpired != 0;
}



bool
EMANE::Models::IEEE80211ABG::NeighborManager::flushTwoHopNeighbors_i(const TimePoint & currentTime,
                                                                     const Microseconds & timeOutMicroseconds)
{
   // internal call

   // number of expired entries
   size_t numExpired{};

   // each two hop nbr
   for(Neighbor2HopEntryMap::iterator iter = twoHopNbrMap_.begin(); iter != twoHopNbrMap_.end(); /* bump/erase below */)
     {
       // nbr age
       const Microseconds ageMicroseconds =
         std::chrono::duration_cast<Microseconds>(currentTime - iter->second.getLastActivityTime());

       // nbr timed out
       if(ageMicroseconds > timeOutMicroseconds)
         {
           // log first, then remove
           LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                  DEBUG_LEVEL,
                                  "MACI %03hu %s::%s: remove 2hop_nbr %hu, age %lf, %zd nbrs remaining",
                                  id_,
                                  pzLayerName,
                                  __func__,
                                  iter->first,
                                  std::chrono::duration_cast<DoubleSeconds>(ageMicroseconds).count(),
                                  twoHopNbrMap_.size() - 1);

           pStatisticTwoHopNbrTable_->deleteRow(iter->first);

           // remove entry and bump
           twoHopNbrMap_.erase(iter++);

           // bump num expired
           ++numExpired;

         }
       else
         {
           // bump
           ++iter;
         }
     }

   // return expired status
   return numExpired != 0;
}




void
EMANE::Models::IEEE80211ABG::NeighborManager::sendOneHopNbrListEvent_i()
{
   // internal call

   NbrSet nbrSet;

   // each one hop nbr
   for(auto & nbrEntry : oneHopNbrMap_)
    {
      nbrSet.insert(nbrEntry.first);
    }

   OneHopNeighborsEvent event(id_, nbrSet);

   LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "MACI %03hu %s::%s: sending list of %zd 1hop_nbrs",
                          id_,
                          pzLayerName,
                          __func__,
                          nbrSet.size());

   // send event
   pPlatformService_->eventService().sendEvent(0,                    // nem id
                                               event);               // the event

   // bump tx events
   pMACLayer_->getStatistics().incrementTxOneHopNbrListEventCount();

   // update last event tx time
   lastOneHopNbrListTxTime_ = Clock::now();
}






EMANE::Models::IEEE80211ABG::NeighborManager::NeighborEntryInsertResult
EMANE::Models::IEEE80211ABG::NeighborManager::addOneHopNeighbor_i(NEMId src)
{
  // internal call

  // try to add one hop nbr
  auto nbrResult = oneHopNbrMap_.insert(std::make_pair(src, NeighborEntry()));

  if(nbrResult.second == true)
   {
     LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                            DEBUG_LEVEL,
                            "MACI %03hu %s::%s: added 1hop_nbr %hu, %zd total nbrs",
                            id_,
                            pzLayerName,
                            __func__,
                            src,
                            oneHopNbrMap_.size());

     pStatisticOneHopNbrTable_->addRow(src,{Any{src}});

     // set high water
     pMACLayer_->getStatistics().updateOneHopNbrHighWaterMark(oneHopNbrMap_.size());

     // check one hop nbr list event cache
     auto iter = cachedOneHopNbrSetMap_.find(src);

     if(iter != cachedOneHopNbrSetMap_.end())
      {
        LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                               DEBUG_LEVEL,
                               "MACI %03hu %s::%s: copied 1hop_nbr_list from cache for nbr %hu, has %zd total nbrs",
                               id_,
                               pzLayerName,
                               __func__,
                               src,
                               iter->second.size());

        // set one hop nbrs of this one hop nbr from the cache
        nbrResult.first->second.setOneHopNeighbors(iter->second);
      }
     else
      {
         LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                DEBUG_LEVEL,
                                "MACI %03hu %s::%s: no 1hop_nbr_list cache for nbr %hu, must wait for event update",
                                id_,
                                pzLayerName,
                                __func__,
                                src);
      }
   }
  else
   {
      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                             DEBUG_LEVEL,
                             "MACI %03hu %s::%s: existing 1hop_nbr %hu, keep existing 1hop_nbr_list",
                             id_,
                             pzLayerName,
                             __func__,
                             src);
   }

  return nbrResult;
}



EMANE::Models::IEEE80211ABG::NeighborManager::Neighbor2HopEntryInsertResult
EMANE::Models::IEEE80211ABG::NeighborManager::addTwoHopNeighbor_i(NEMId src)
{
   // internal call

   // add 2 hop nbr
   auto nbr2Result = twoHopNbrMap_.insert(std::make_pair(src, Neighbor2HopEntry()));

   if(nbr2Result.second == true)
    {
      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                             DEBUG_LEVEL,
                             "MACI %03hu %s::%s: added 2hop_nbr %hu, %zd total nbrs",
                             id_,
                             pzLayerName,
                             __func__,
                             src,
                             twoHopNbrMap_.size());

      pStatisticTwoHopNbrTable_->addRow(src,{Any{src}});

      // set high water
      pMACLayer_->getStatistics().updateTwoHopNbrHighWaterMark(twoHopNbrMap_.size());
    }
   else
    {
      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                             DEBUG_LEVEL,
                             "MACI %03hu %s::%s: existing 2hop_nbr %hu, %zd total nbrs",
                             id_,
                             pzLayerName,
                             __func__,
                             src,
                             twoHopNbrMap_.size());
    }

   return nbr2Result;
 }




float
EMANE::Models::IEEE80211ABG::NeighborManager::getA_i(const Microseconds & utilizationMicroseconds,
                                                     const Microseconds & avgUtilizationMicroseconds) const
{
  // internal call

  const float A{getChannelActivity_i(utilizationMicroseconds, avgUtilizationMicroseconds)};

  // cap at 1.0
  if(A > 1.0f)
   {
     return 1.0f;
   }
  else
   {
     return A;
   }
}



float
EMANE::Models::IEEE80211ABG::NeighborManager::getC_i(const Microseconds & utilizationMicroseconds,
                                                     const Microseconds & deltaTMicroseconds) const
{
  // internal call

  const float C{getChannelActivity_i(utilizationMicroseconds, deltaTMicroseconds)};

  // cap to 1.0
  if(C > 1.0f)
   {
     return 1.0f;
   }
  else
   {
     return C;
   }
}



float
EMANE::Models::IEEE80211ABG::NeighborManager::getH_i(const Microseconds & utilizationMicroseconds,
                                                     const Microseconds & deltaTMicroseconds) const
{
  // internal call

  const float H{getChannelActivity_i(utilizationMicroseconds, deltaTMicroseconds)};

  // cap at 1.0
  if(H > 1.0f)
   {
     return 1.0f;
   }
  else
   {
     return H;
   }
}



float
EMANE::Models::IEEE80211ABG::NeighborManager::getChannelActivity_i(const Microseconds & utilizationMicroseconds,
                                                                   const Microseconds & deltaTMicroseconds) const
{
  // internal call

  // check divide by zero
  if(deltaTMicroseconds == Microseconds::zero())
   {
     return 0.0f;
   }
  else
   {
     return getRatio(utilizationMicroseconds, deltaTMicroseconds);
   }
}


void
EMANE::Models::IEEE80211ABG::NeighborManager::setCommonAndHiddenProbability_i()
{
   // for each one hop utilization
   for(auto & oneHopUtilization : oneHopUtilizationMap_)
    {
      // exclude ourself
      if(oneHopUtilization.first != id_)
       {
          // remove local and src utilization from total one hop utilization
          Microseconds adjustedUtililizationMicroseconds{
                           totalOneHopUtilizationMicroseconds_ -
                               utilizationThisNEMMicroseconds_ -
                                 oneHopUtilization.second};

          // check remaining utlzation
          if(adjustedUtililizationMicroseconds > Microseconds::zero())
           {
             // lookup nbr
             auto niter = oneHopNbrMap_.find(oneHopUtilization.first);

             if(niter != oneHopNbrMap_.end())
              {
                // get common nbrs
                const NbrSet commonNbrs = niter->second.getCommonNeighbors();

                // get hidden nbrs
                const NbrSet hiddenNbrs = niter->second.getHiddenNeighbors();

                // get one hop utilzation
                NbrUtilizationMap adjustedUtilzationMap = oneHopUtilizationMap_;

                // remove local node from utilization
                adjustedUtilzationMap.erase(id_);

                // remove src from utilization
                adjustedUtilzationMap.erase(oneHopUtilization.first);

                // get and store common probability using adjusted one hop utilization for this src
                commonProbabilityMapMap_[oneHopUtilization.first] = setProbability_i(oneHopUtilization.first,
                                                                          adjustedUtilzationMap,
                                                                          adjustedUtililizationMicroseconds,
                                                                          commonNbrs,
                                                                          "common");

               // get and store hidden probability using adjusted one hop utilization for this src
               hiddenProbabilityMapMap_[oneHopUtilization.first] = setProbability_i(oneHopUtilization.first,
                                                                         adjustedUtilzationMap,
                                                                         adjustedUtililizationMicroseconds,
                                                                         hiddenNbrs,
                                                                         "hidden");
             }
          }
       }
    }
}



EMANE::Models::IEEE80211ABG::NeighborManager::ProbabilityPairMap
EMANE::Models::IEEE80211ABG::NeighborManager::setProbability_i(NEMId src __attribute__((unused)),
                                                               const NbrUtilizationMap & map,
                                                               const Microseconds & rUtilizationMicroseconds,
                                                               const NbrSet & nbrSet,
                                                               const char *str __attribute__((unused))) const
{
  ProbabilityPairMap probabilityMap;

  NbrUtilizationMap adjustedUtilizationMap;

  Microseconds utilizationMicroseconds{rUtilizationMicroseconds};

  for(auto & utilization : map)
   {
      if(nbrSet.find(utilization.first) != nbrSet.end())
       {
         LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                DEBUG_LEVEL,
                                "MACI %03hu %s::%s_%s: src %hu, is %s w/r to nbr %hu",
                                id_,
                                pzLayerName,
                                __func__,
                                str,
                                src,
                                str,
                                utilization.first);

         // add entry
         adjustedUtilizationMap[utilization.first] = utilization.second;
       }
      else
       {
         LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                DEBUG_LEVEL,
                                "MACI %03hu %s::%s_%s: src %hu, is not %s w/r to nbr %hu, ignore",
                                id_,
                                pzLayerName,
                                __func__,
                                str,
                                src,
                                str,
                                utilization.first);

         // subtract utilization
         utilizationMicroseconds -= utilization.second;
       }
   }

  // check remaining utilization
  if(utilizationMicroseconds > Microseconds::zero())
   {
     // initial values
     float p1{}, p2{};

     size_t num{adjustedUtilizationMap.size()};

     // for each utilization
     for(NbrUtilizationMap::iterator iter = adjustedUtilizationMap.begin(); iter != adjustedUtilizationMap.end(); ++iter, --num)
      {
         // not last entry
         if(num != 1)
          {
            // get ratio
            p2 += getRatio(iter->second, utilizationMicroseconds);
          }
         // last entry
         else
          {
            // set to 1
            p2 = 1.0f;
          }

          LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                 DEBUG_LEVEL,
                                 "MACI %03hu %s::%s_%s: src %hu, nbr %hu, p1 %5.4f, p2 %5.4f",
                                 id_,
                                 pzLayerName,
                                 __func__,
                                 str,
                                 src,
                                 iter->first,
                                 p1,
                                 p2);

          // insert range
          probabilityMap[iter->first] = ProbabilityPair(p1, p2);

          // bump range
          p1 = p2;
      }
   }

  return probabilityMap;
}


void
EMANE::Models::IEEE80211ABG::NeighborManager::registerStatistics(StatisticRegistrar & statisticRegistrar)
{
  pStatisticOneHopNbrTable_ = statisticRegistrar.registerTable<NEMId>("OneHopNeighborTable",
                                                                      {"NEM Id"},
                                                                      StatisticProperties::NONE,
                                                                      "Current One Hop Neighbors");

  pStatisticTwoHopNbrTable_ = statisticRegistrar.registerTable<NEMId>("TwoHopNeighborTable",
                                                                      {"NEM Id"},
                                                                      StatisticProperties::NONE,
                                                                      "Current Two Hop Neighbors");

}
