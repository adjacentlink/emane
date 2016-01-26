/*
 * Copyright (c) 2013,2016 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2008 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANEMODELSIEEE802ABGMACSTATISTICS_HEADER_
#define EMANEMODELSIEEE802ABGMACSTATISTICS_HEADER_

#include "macconfig.h"

#include "emane/platformserviceprovider.h"
#include "emane/statisticnumeric.h"

#include <map>
#include <vector>

namespace EMANE
{
  namespace Models
  {
    namespace IEEE80211ABG
    {

      /**
       *
       * @brief class used to define the mac layer statistic items
       *
       */
      class MACStatistics
      {
        /*
         * Statistic timers
         */
      public:
        MACStatistics(EMANE::NEMId);

        ~MACStatistics();

        // discards
        void incrementDownstreamUnicastDataDiscardDueToRetries();
        void incrementDownstreamUnicastRtsCtsDataDiscardDueToRetries();

        void incrementDownstreamUnicastDataDiscardDueToTxop();
        void incrementDownstreamBroadcastDataDiscardDueToTxop();

        void incrementUpstreamUnicastDataDiscardDueToSinr();
        void incrementUpstreamBroadcastDataDiscardDueToSinr();

        void incrementUpstreamUnicastDataDiscardDueToClobberRxDuringTx();
        void incrementUpstreamBroadcastDataDiscardDueToClobberRxDuringTx();

        void incrementUpstreamUnicastDataDiscardDueToClobberRxHiddenBusy();
        void incrementUpstreamBroadcastDataDiscardDueToClobberRxHiddenBusy();

        void incrementUpstreamBroadcastNoiseHiddenRx();
        void incrementUpstreamUnicastNoiseHiddenRx();

        void incrementUpstreamBroadcastNoiseRxCommon();
        void incrementUpstreamUnicastNoiseRxCommon();

        void incrementUpstreamUnicastRtsCtsDataRxFromPhy();
        void incrementUpstreamUnicastCtsRxFromPhy();

        void updateOneHopNbrHighWaterMark(size_t num);
        void updateTwoHopNbrHighWaterMark(size_t num);

        // events
        void incrementRxOneHopNbrListEventCount();
        void incrementRxOneHopNbrListInvalidEventCount();
        void incrementTxOneHopNbrListEventCount();

        void registerStatistics(StatisticRegistrar & statisticRegistrar);

      private:
        EMANE::NEMId id_;

        /**
         *
         * @brief ieee80211abg mac statistic items.
         *
         */
        StatisticNumeric<std::uint32_t> * pNumDownstreamUnicastDataDiscardDueToRetries_;
        StatisticNumeric<std::uint32_t> * pNumDownstreamUnicastRtsCtsDataDiscardDueToRetries_;

        StatisticNumeric<std::uint32_t> * pNumUpstreamUnicastDataDiscardDueToSinr_;
        StatisticNumeric<std::uint32_t> * pNumUpstreamBroadcastDataDiscardDueToSinr_;

        StatisticNumeric<std::uint32_t> * pNumUpstreamUnicastDataDiscardDueToClobberRxDuringTx_;
        StatisticNumeric<std::uint32_t> * pNumUpstreamBroadcastDataDiscardDueToClobberRxDuringTx_;

        StatisticNumeric<std::uint32_t> * pNumUpstreamUnicastDataDiscardDueToClobberRxHiddenBusy_;
        StatisticNumeric<std::uint32_t> * pNumUpstreamBroadcastDataDiscardDueToClobberRxHiddenBusy_;

        StatisticNumeric<std::uint32_t> * pNumDownstreamUnicastDataDiscardDueToTxop_;
        StatisticNumeric<std::uint32_t> * pNumDownstreamBroadcastDataDiscardDueToTxop_;

        StatisticNumeric<std::uint32_t> * pNumUpstreamUnicastRtsCtsDataRxFromPhy_;
        StatisticNumeric<std::uint32_t> * pNumUpstreamUnicastCtsRxFromPhy_;

        StatisticNumeric<std::uint32_t> * pNumUpstreamUnicastDataNoiseHiddenRx_;
        StatisticNumeric<std::uint32_t> * pNumUpstreamBroadcastDataNoiseHiddenRx_;

        StatisticNumeric<std::uint32_t> * pNumUpstreamUnicastDataNoiseRxCommon_;
        StatisticNumeric<std::uint32_t> * pNumUpstreamBroadcastDataNoiseRxCommon_;

        StatisticNumeric<std::uint32_t> * pNumOneHopNbrHighWaterMark_;
        StatisticNumeric<std::uint32_t> * pNumTwoHopNbrHighWaterMark_;

        StatisticNumeric<std::uint32_t> * pNumRxOneHopNbrListEvents_;
        StatisticNumeric<std::uint32_t> * pNumRxOneHopNbrListInvalidEvents_;
        StatisticNumeric<std::uint32_t> * pNumTxOneHopNbrListEvents_;
      };
    }
  }
}
#endif  //EMANEMODELSIEEE802ABGMACSTATISTICS_HEADER_
