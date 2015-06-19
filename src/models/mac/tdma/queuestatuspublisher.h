/*
 * Copyright (c) 2015 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANEMODELSTDMAQUEUESTATUSTABLEPUBLISHER_HEADER_
#define EMANEMODELSTDMAQUEUESTATUSTABLEPUBLISHER_HEADER_

#include "emane/statisticregistrar.h"
#include "emane/models/tdma/messagecomponent.h"

#include <map>
#include <tuple>
#include <array>

namespace EMANE
{
  namespace Models
  {
    namespace TDMA
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

        void drop(std::uint8_t u8Queue,
                  DropReason reason,
                  size_t count);

        void dequeue(std::uint8_t u8RequestQueue,
                     std::uint8_t u8ActualQueue,
                     const MessageComponents & components);

        void enqueue(std::uint8_t u8Queue);

      private:
        StatisticTable<std::uint8_t> * pQueueStatusTable_;

        using StatusTableInfo = std::map<std::uint8_t,std::tuple<std::uint64_t, // enqueued
                                                                 std::uint64_t, // dequeued
                                                                 std::uint64_t, // overflow
                                                                 std::uint64_t, // too big
                                                                 std::uint64_t, // queue 0
                                                                 std::uint64_t, // queue 1
                                                                 std::uint64_t, // queue 2
                                                                 std::uint64_t, // queue 3
                                                                 std::uint64_t>>; // queue 4

        StatusTableInfo statusTableInfo_;

        StatisticTable<std::uint8_t> * pQueueFragmentHistogram_;

        using FragmentHistogram = std::map<std::uint8_t,std::array<std::uint64_t,10>>;

        FragmentHistogram fragmentHistogram_;
      };
    }
  }
}

#endif // EMANEMODELSTDMAQUEUESTATUSTABLEPUBLISHER_HEADER_
