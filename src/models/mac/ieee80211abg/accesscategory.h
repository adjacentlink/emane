/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANE_MODELS_IEEE80211ABG_ACCESSCATEGORY_HEADER_
#define EMANE_MODELS_IEEE80211ABG_ACCESSCATEGORY_HEADER_

#include "emane/types.h"


#include "macconfig.h"
#include "downstreamqueueentry.h"

#include <queue>
#include <sstream>

namespace EMANE
{
  namespace Models
  {
    namespace IEEE80211ABG
    {
      typedef std::queue <DownstreamQueueEntry> IEEE80211abgDownstreamPacketQueue;

      /**
       * @brief Defines an access category container
       *
       *
       */
      struct AccessCategory
      {
        std::uint8_t category_;

        std::uint8_t u8MaxQueueCapacity_;

        std::uint16_t u16MaxPacketSize_;

        size_t numPacketOverFlow_;

        IEEE80211abgDownstreamPacketQueue queue_;

        /*
         * Stat counters
         */
        // unicast
        StatisticNumeric<std::uint32_t> * pNumUnicastPacketsTooLarge_;
        StatisticNumeric<std::uint32_t> * pNumUnicastBytesTooLarge_;

        // broadcast
        StatisticNumeric<std::uint32_t> * pNumBroadcastPacketsTooLarge_;
        StatisticNumeric<std::uint32_t> * pNumBroadcastBytesTooLarge_;

        // queue size/depth
        StatisticNumeric<std::uint32_t> * pNumHighWaterMark_;
        StatisticNumeric<std::uint32_t> * pNumHighWaterMax_;

        /*
         * Constructor
         */
        AccessCategory() :
          category_{},
          u8MaxQueueCapacity_{QUEUE_SIZE_DEFAULT},
          u16MaxPacketSize_{MAX_PACKET_SIZE},
          numPacketOverFlow_{},
          pNumUnicastPacketsTooLarge_{},
          pNumUnicastBytesTooLarge_{},
          pNumBroadcastPacketsTooLarge_{},
          pNumBroadcastBytesTooLarge_{},
          pNumHighWaterMark_{},
          pNumHighWaterMax_{}
        {} 

        void registerStatistics(StatisticRegistrar & statisticRegistrar)
          {
            std::string sCategory{std::to_string(category_)};

            pNumUnicastPacketsTooLarge_ = 
              statisticRegistrar.registerNumeric<std::uint32_t>("numUnicastPacketsTooLarge" + sCategory,
                                                                StatisticProperties::CLEARABLE);
            pNumUnicastBytesTooLarge_ =
              statisticRegistrar.registerNumeric<std::uint32_t>("numUnicastBytesTooLarge" + sCategory,
                                                                StatisticProperties::CLEARABLE);

            pNumBroadcastPacketsTooLarge_ = 
              statisticRegistrar.registerNumeric<std::uint32_t>("numBroadcastPacketsTooLarge" + sCategory,
                                                                StatisticProperties::CLEARABLE);
            pNumBroadcastBytesTooLarge_ =
              statisticRegistrar.registerNumeric<std::uint32_t>("numBroadcastBytesTooLarge" + sCategory,
                                                                StatisticProperties::CLEARABLE);
             
            pNumHighWaterMark_ =
              statisticRegistrar.registerNumeric<std::uint32_t>("numHighWaterMark" + sCategory,
                                                                StatisticProperties::CLEARABLE);
            pNumHighWaterMax_ =
              statisticRegistrar.registerNumeric<std::uint32_t>("numHighWaterMax" + sCategory,
                                                                StatisticProperties::CLEARABLE);
          }
      };
    }
  }
}
#endif //EMANE_MODELS_IEEE80211ABG_ACCESSCATEGORY_HEADER_
