/*
 * Copyright (c) 2015,2023 - Adjacent Link LLC, Bridgewater, New Jersey
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
 * * Neither the name of Adjacent Link LLC nor the names of its
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

#ifndef EMANE_MODELS_BENTPIPE_QUEUESTATUSPUBLISHER_HEADER_
#define EMANE_MODELS_BENTPIPE_QUEUESTATUSPUBLISHER_HEADER_

#include "types.h"
#include "messagecomponent.h"

#include "emane/statisticregistrar.h"
#include <map>
#include <tuple>
#include <array>

namespace EMANE
{
  namespace Models
  {
    namespace BentPipe
    {
      class QueueStatusPublisher
      {
      public:
        QueueStatusPublisher();

        enum class DropReason
          {
            DROP_OVERFLOW,
            DROP_TOOBIG,
          };

        void registerStatistics(StatisticRegistrar & registrar);

        void drop(TransponderIndex index,
                  DropReason reason,
                  size_t count);

        void dequeue(TransponderIndex index,
                     const MessageComponents & components);

        void enqueue(TransponderIndex index);

      private:
        StatisticTable<TransponderIndex> * pQueueStatusTable_;

        using StatusTableEntry = std::tuple<std::uint64_t, // enqueued
                                            std::uint64_t, // dequeued
                                            std::uint64_t, // overflow
                                            std::uint64_t, // too big
                                            std::uint64_t,  // queue depth
                                            std::uint64_t>; // high water mark

        using StatusTableInfo = std::map<TransponderIndex,StatusTableEntry>;

        StatusTableInfo statusTableInfo_;

        StatisticTable<TransponderIndex> * pQueueFragmentHistogram_;

        using FragmentHistogramEntry = std::array<std::uint64_t,10>;

        using FragmentHistogram = std::map<TransponderIndex,
                                           FragmentHistogramEntry>;

        StatisticTable<TransponderIndex> * pQueueAggregateHistogram_;

        using AggregateHistogramEntry = std::array<std::uint64_t,10>;

        using AggregateHistogram = std::map<TransponderIndex,
                                            AggregateHistogramEntry>;


        FragmentHistogram fragmentHistogram_;
        AggregateHistogram aggregateHistogram_;
      };
    }
  }
}

#endif // EMANE_MODELS_BENTPIPE_QUEUESTATUSPUBLISHER_HEADER_
