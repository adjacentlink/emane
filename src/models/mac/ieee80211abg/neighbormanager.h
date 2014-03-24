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

#ifndef EMANEMODELSIEEE802ABGNEIGHBORMANAGER_HEADER_
#define EMANEMODELSIEEE802ABGNEIGHBORMANAGER_HEADER_


#include "emane/types.h"
#include "emane/platformserviceprovider.h"
#include "emane/serializationexception.h"
#include "emane/statistictable.h"

#include "emane/utils/randomnumberdistribution.h"

#include "wmmmanager.h"
#include "neighborentry.h"
#include "neighbor2hopentry.h"
#include "onehopneighborsevent.h"

#include <map>
#include <list>

namespace EMANE
 {
  namespace Models
   {
    namespace IEEE80211ABG
     {
      class MACLayer;

      /**
       * @class  NeighborManager
       *
       * @brief Defines the ieee 80211 abg 1 and 2 hop neighbor manager
       *
       */
        class NeighborManager
        {
        public:
          NeighborManager(NEMId id,
                           PlatformServiceProvider * pPlatformService,
                           MACLayer *pMgr);

          ~NeighborManager();

          void updateDataChannelActivity(NEMId src,
                                          std::uint8_t type,
                                          float fRxPowerMilliWatts, 
                                          const TimePoint & timePoint,
                                          const Microseconds & duration,
                                          std::uint8_t u8Category);

          void updateCtrlChannelActivity(NEMId src,
                                          NEMId origin,
                                          std::uint8_t type,
                                          float fRxPowerMilliWatts,
                                          const TimePoint & tvTime,
                                          const Microseconds & duration,
                                          std::uint8_t u8Category);

          void handleOneHopNeighborsEvent(const Serialization &serialization);

          void start();

          void resetStatistics();

          void setNeighborTimeoutMicroseconds(const Microseconds & timeOutMicroseconds);

          float getNumberOfEstimatedOneHopNeighbors() const;

          float getNumberOfEstimatedTwoHopNeighbors() const;

          float getHiddenChannelActivity(NEMId src) const;

          float getNumberOfEstimatedCommonNeighbors(NEMId src) const;

          float getNumberOfEstimatedHiddenNeighbors(NEMId src) const;

          float getLocalNodeTx() const;

          size_t getTotalActiveOneHopNeighbors() const;

          void setCategories(std::uint8_t u8NumCategories);
 
          Microseconds getTotalOneHopUtilizationMicroseconds() const;

          Microseconds getTotalTwoHopUtilizationMicroseconds() const;

          Microseconds getAverageMessageDurationMicroseconds() const;

          Microseconds getAllUtilizationMicroseconds(NEMId src) const;

          float getAverageRxPowerPerMessageMilliWatts() const;

          float getAverageRxPowerPerMessageHiddenNodesMilliWatts() const;

          float getAverageRxPowerPerMessageCommonNodesMilliWatts() const;
    
          float getRandomRxPowerCommonNodesMilliWatts(NEMId src);

          float getRandomRxPowerHiddenNodesMilliWatts(NEMId src);

          TimePoint getLastOneHopNbrListTxTime() const;

          WMMManager::UtilizationRatioVector getUtilizationRatios();

          void registerStatistics(StatisticRegistrar & statisticRegistrar);

        private:
          using NeighborEntryMap = std::map<NEMId, NeighborEntry>;

          using NeighborEntryInsertResult = std::pair<NeighborEntryMap::iterator, bool>;

          using Neighbor2HopEntryMap = std::map<NEMId, Neighbor2HopEntry>;

          using  Neighbor2HopEntryInsertResult = std::pair<Neighbor2HopEntryMap::iterator, bool>;

          using  NbrSetMap = std::map<NEMId, NbrSet>;

          using  NbrUtilizationMap = std::map<NEMId, Microseconds>;

          using ProbabilityPair = std::pair<float, float>;

          using  ProbabilityPairMap = std::map<NEMId, ProbabilityPair>;

          using  ProbabilityPairMapMap = std::map<NEMId, ProbabilityPairMap>;

          using  RxPowerMap = std::map<NEMId, float>;

          NEMId id_;

          PlatformServiceProvider * pPlatformService_;

          MACLayer *pMACLayer_;

          WMMManager wmmManager_;

          NeighborEntryMap oneHopNbrMap_;

          Neighbor2HopEntryMap twoHopNbrMap_;

          NbrSetMap cachedOneHopNbrSetMap_;

          Microseconds totalOneHopUtilizationMicroseconds_;

          Microseconds totalTwoHopUtilizationMicroseconds_;

          size_t totalOneHopNumPackets_;

          size_t totalTwoHopNumPackets_;

          size_t numTotalActiveOneHopNeighbors_;

          Microseconds averageMessageDurationMicroseconds_;

          float fAverageRxPowerPerMessageMilliWatts_;

          float fTotalRxPowerMilliWatts_;

          Microseconds averageUtilizationPerOneHopNbrMicroseconds_;

          Microseconds averageUtilizationPerTwoHopNbrMicroseconds_;

          float fEstimatedNumOneHopNeighbors_;

          float fEstimatedNumTwoHopNeighbors_;
 
          float fLocalNodeTx_;

          size_t sumCommonPackets_;

          size_t sumHiddenPackets_;

          float fCommonRxPowerMilliWatts_;

          float fHiddenRxPowerMilliWatts_;

          Microseconds utilizationThisNEMMicroseconds_;

          Microseconds nbrTimeOutMicroseconds_;

          TimePoint lastOneHopNbrListTxTime_;

          NbrUtilizationMap oneHopUtilizationMap_;

          NbrUtilizationMap twoHopUtilizationMap_;

          ProbabilityPairMapMap commonProbabilityMapMap_;

          ProbabilityPairMapMap hiddenProbabilityMapMap_;

          RxPowerMap commonNbrAvgRxPowerMwMap_;

          RxPowerMap hiddenNbrAvgRxPowerMwMap_;

          WMMManager::UtilizationRatioVector utilizationRatioVector_;

          Utils::RandomNumberDistribution<std::mt19937, 
                                          std::uniform_real_distribution<float>> RNDZeroToOne_;

          StatisticTable<NEMId> * pStatisticOneHopNbrTable_;

          StatisticTable<NEMId> * pStatisticTwoHopNbrTable_;

          TimePoint lastResetTime_;

          void sendOneHopNbrListEvent_i();

          bool flushOneHopNeighbors_i(const TimePoint & tvCurrentTime, const Microseconds & timeOutMicroseconds);

          bool flushTwoHopNeighbors_i(const TimePoint & tvCurrentTime, const Microseconds & timeOutMicroseconds);

          void resetCounters_i();

          void calculateBwUtilization_i(const Microseconds & deltaTMicroseconds);

          NeighborEntryInsertResult addOneHopNeighbor_i(NEMId src);

          Neighbor2HopEntryInsertResult addTwoHopNeighbor_i(NEMId src);

          float getA_i(const Microseconds & utilizationMicroseconds, const Microseconds & avgUtilizationMicroseconds) const;

          float getC_i(const Microseconds & utilizationMicroseconds, const Microseconds & deltaTMicroseconds) const;

          float getH_i(const Microseconds & utilizationMicroseconds, const Microseconds & deltaTMicroseconds) const;

          float getChannelActivity_i(const Microseconds & utilizationMicroseconds, const Microseconds & deltaTMicroseconds) const;

          void setCommonAndHiddenProbability_i();

          ProbabilityPairMap setProbability_i(NEMId id, const NbrUtilizationMap & map, 
                                               const Microseconds & utilizationMicroseconds, 
                                               const NbrSet & nbrSet, const char * str) const;

          float getRandomRxPowerMilliWatts_i(NEMId src, float R1, 
                                                const ProbabilityPairMapMap & pmap, 
                                                const RxPowerMap & rmap) const;
        };
     }
  }
}
#endif  //EMANEMODELSIEEE802ABGNEIGHBORMANAGER_HEADER_
