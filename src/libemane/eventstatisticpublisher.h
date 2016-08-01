/*
 * Copyright (c) 2016 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANEEVENTSTATISTICPUBLISHER_HEADER_
#define EMANEEVENTSTATISTICPUBLISHER_HEADER_

#include "emane/types.h"
#include "emane/statisticnumeric.h"
#include "emane/statistictable.h"
#include "emane/statisticregistrar.h"

#include <uuid.h>
#include <mutex>
#include <string>
#include <tuple>
#include <map>

namespace EMANE
{
  class EventStatisticPublisher
  {
  public:
    EventStatisticPublisher(const std::string & sPrefix);

    enum class Type
    {
      TYPE_TX,
        TYPE_RX,
    };

    void update(Type type,const uuid_t & uuid, EventId eventId);

    void setRowLimit(size_t rows);

  private:
    using EventCountTableKey = std::pair<std::string,EventId>;

    using EventCountInfo =
      std::map<EventCountTableKey,
               std::tuple<std::uint64_t, // events Tx
                          std::uint64_t>>; // events Rx

    StatisticNumeric<std::uint64_t> * pNumEventsTx_;
    StatisticNumeric<std::uint64_t> * pNumEventsRx_;

    StatisticTable<EventCountTableKey> * pEventCountTable_;

    EventCountInfo eventCountInfo_;

    std::mutex mutexEventCountTable_;

    size_t rowLimit_;
  };
}

#endif //EMANEEVENTSTATISTICPUBLISHER_HEADER_
