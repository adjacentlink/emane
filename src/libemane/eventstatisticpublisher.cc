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

#include "eventstatisticpublisher.h"
#include "statisticregistrarproxy.h"

// specialized hash for EventCountTable
namespace std
{
  template<>
  struct hash<std::pair<std::string,EMANE::EventId>>
  {
    typedef  std::pair<std::string,EMANE::EventId> argument_type;
    typedef std::size_t result_type;

    result_type operator()(argument_type const& s) const
    {
      result_type const h1{std::hash<std::string>()(s.first)};
      result_type const h2{std::hash<EMANE::EventId>()(s.second)};
      return h1 ^ (h2 << 1);
    }
  };
}

namespace
{
  const EMANE::StatisticTableLabels EventCountLabels =
    {
      "Src",
      "Emulator UUID",
      "Num Events Tx",
      "Num Events Rx"
    };

  enum EventCountColumn
    {
      EVENT_COUNT_COLUMN_EVENT = 0,
      EVENT_COUNT_COLUMN_UUID = 1,
      EVENT_COUNT_COLUMN_NUM_EVENTS_TX = 2,
      EVENT_COUNT_COLUMN_NUM_EVENTS_RX = 3,
    };
}

EMANE::EventStatisticPublisher::EventStatisticPublisher(const std::string & sPrefix):
  rowLimit_{0}
{
  auto statisticRegistrar = StatisticRegistrarProxy{*StatisticServiceSingleton::instance(),0};

  pNumEventsTx_ =
    statisticRegistrar.registerNumeric<std::uint64_t>("num" + sPrefix +"EventsTx",
                                                      StatisticProperties::CLEARABLE);
  pNumEventsRx_ =
    statisticRegistrar.registerNumeric<std::uint64_t>("num" + sPrefix + "EventsRx",
                                                      StatisticProperties::CLEARABLE);
  pEventCountTable_ =
    statisticRegistrar.registerTable<EventCountTableKey>(sPrefix + "EventCountTable",
                                                          EventCountLabels,
                                                          [this](StatisticTablePublisher * pTable)
                                                          {
                                                            std::lock_guard<std::mutex> m(mutexEventCountTable_);
                                                            eventCountInfo_.clear();
                                                            pTable->clear();
                                                          },
                                                         sPrefix + " Event count table.");
}

void EMANE::EventStatisticPublisher::update(Type type, const uuid_t & uuid, EventId eventId)
{
  char buf[37];
  uuid_unparse(uuid,buf);
  auto key = EventCountTableKey{buf,eventId};

  std::lock_guard<std::mutex> m(mutexEventCountTable_);

  auto iter = eventCountInfo_.find(key);

  if(iter == eventCountInfo_.end())
    {
      if(eventCountInfo_.size() < rowLimit_)
        {
          iter = eventCountInfo_.insert({key,std::make_tuple(0,0)}).first;

          pEventCountTable_->addRow(key,
                                    {Any{eventId},
                                        Any{buf},
                                          Any{0L},
                                            Any{0L}});
        }
    }


  if(type == Type::TYPE_RX)
    {
      ++*pNumEventsRx_;

       if(iter != eventCountInfo_.end())
         {
           auto & events = std::get<EVENT_COUNT_COLUMN_NUM_EVENTS_RX-2>(iter->second);

           events += 1;

           pEventCountTable_->setCell(key,
                                      EVENT_COUNT_COLUMN_NUM_EVENTS_RX,
                                      Any{events});
         }

    }
  else if(type == Type::TYPE_TX)
    {
      ++*pNumEventsTx_;

      if(iter != eventCountInfo_.end())
        {
          auto & events = std::get<EVENT_COUNT_COLUMN_NUM_EVENTS_TX-2>(iter->second);

          events += 1;

          pEventCountTable_->setCell(key,
                                     EVENT_COUNT_COLUMN_NUM_EVENTS_TX,
                                     Any{events});
        }
    }
}

void  EMANE::EventStatisticPublisher::setRowLimit(size_t rows)
{
  rowLimit_ = rows;
}
