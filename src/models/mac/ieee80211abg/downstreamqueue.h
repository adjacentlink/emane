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

#ifndef EMANEMODELSIEEE80211ABGDOWNSTREAMQUEUE_HEADER_
#define EMANEMODELSIEEE80211ABGDOWNSTREAMQUEUE_HEADER_

#include "emane/types.h"
#include "emane/statisticnumeric.h"

#include "downstreamqueueentry.h"
#include "accesscategory.h"
#include "macconfig.h"

namespace EMANE
{
  namespace Models
  {
    namespace IEEE80211ABG
    {
      /**
       *
       * @brief class used to define the mac downstream packet queue
       *
       */
      class DownstreamQueue
      {
      public:
        DownstreamQueue(EMANE::NEMId id);

        ~DownstreamQueue();

        void setMaxCapacity(size_t);

        void setMaxCapacity(size_t, std::uint8_t u8Category);

        void setMaxEntrySize(size_t);

        void setMaxEntrySize(size_t, std::uint8_t u8Category);

        size_t getMaxCapacity();

        size_t getMaxCapacity(std::uint8_t u8Category);

        size_t getAvailableSpace();

        size_t getAvailableSpace(std::uint8_t u8Category);

        size_t getDepth();

        size_t getDepth(std::uint8_t u8Category);

        size_t getNumOverFlow(bool bClear);

        size_t getNumOverFlow(std::uint8_t u8Category, bool bClear);

        void setCategories(std::uint8_t u8Category);

        std::pair<DownstreamQueueEntry, bool> dequeue();

        std::vector<DownstreamQueueEntry> enqueue(DownstreamQueueEntry & entry);

        void registerStatistics(StatisticRegistrar & statisticRegistrar);

      private:
        EMANE::NEMId id_;

        AccessCategory categories_[MAX_ACCESS_CATEGORIES];

        std::uint8_t numActiveCategories_;

        StatisticNumeric<std::uint32_t> * pNumUnicastPacketsUnsupported_;
        StatisticNumeric<std::uint32_t> * pNumUnicastBytesUnsupported_;
        StatisticNumeric<std::uint32_t> * pNumBroadcastPacketsUnsupported_;
        StatisticNumeric<std::uint32_t> * pNumBroadcastBytesUnsupported_;
      };
    }
  }
}
#endif //EMANEMODELSIEEE80211ABGDOWNSTREAMQUEUE_HEADER_
