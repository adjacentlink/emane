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

#include "queuestatuspublisher.h"

EMANE::Models::TDMA::QueueStatusPublisher::QueueStatusPublisher():
  pQueueStatusTable_{},
  statusTableInfo_{{0,{}},{1,{}},{2,{}},{3,{}},{4,{}}},
  fragmentHistogram_{{0,{0}},{1,{0}},{2,{0}},{3,{0}},{4,{0}}}
{}

void EMANE::Models::TDMA::QueueStatusPublisher::registerStatistics(StatisticRegistrar & statisticRegistrar)
{
  pQueueStatusTable_ =
    statisticRegistrar.registerTable<std::uint8_t>("QueueStatusTable",
      {"Queue","Enqueued","Dequeued","Overflow","Too Big","0","1","2","3","4"},
      StatisticProperties::NONE,
      "Shows for each queue the number of packets enqueued, dequeued,"
      " dropped due to queue overflow (enqueue), dropped due to too big"
      " (dequeue) and which slot classes fragments are being transmitted.");

  for(const auto & entry : statusTableInfo_)
    {
      pQueueStatusTable_->addRow(entry.first,
                                 {Any{entry.first},
                                     Any{std::get<0>(entry.second)},
                                       Any{std::get<1>(entry.second)},
                                         Any{std::get<2>(entry.second)},
                                           Any{std::get<3>(entry.second)},
                                             Any{std::get<4>(entry.second)},
                                               Any{std::get<5>(entry.second)},
                                                 Any{std::get<6>(entry.second)},
                                                   Any{std::get<7>(entry.second)},
                                                     Any{std::get<8>(entry.second)}});



    }


  pQueueFragmentHistogram_ =
    statisticRegistrar.registerTable<std::uint8_t>("QueueFragmentHistogram",
      {"Queue","1","2","3","4","5","6","7","8","9",">9"},
      StatisticProperties::NONE,
      "Shows a per queue histogram of the number of message components required to transmit packets.");

  for(const auto & entry : fragmentHistogram_)
    {
      pQueueFragmentHistogram_->addRow(entry.first,
                                       {Any{entry.first},
                                           Any{std::get<0>(entry.second)},
                                             Any{std::get<1>(entry.second)},
                                               Any{std::get<2>(entry.second)},
                                                 Any{std::get<3>(entry.second)},
                                                   Any{std::get<4>(entry.second)},
                                                     Any{std::get<5>(entry.second)},
                                                       Any{std::get<6>(entry.second)},
                                                         Any{std::get<7>(entry.second)},
                                                           Any{std::get<8>(entry.second)},
                                                             Any{std::get<9>(entry.second)}});
    }


  pHighWaterMarkQueue_[0] =
    statisticRegistrar.registerNumeric<std::uint64_t>("highWaterMarkQueue0",
                                                      StatisticProperties::CLEARABLE,
                                                      "High water mark queue 0");

  pHighWaterMarkQueue_[1] =
    statisticRegistrar.registerNumeric<std::uint64_t>("highWaterMarkQueue1",
                                                      StatisticProperties::CLEARABLE,
                                                      "High water mark queue 1");

  pHighWaterMarkQueue_[2] =
    statisticRegistrar.registerNumeric<std::uint64_t>("highWaterMarkQueue2",
                                                      StatisticProperties::CLEARABLE,
                                                      "High water mark queue 2");

  pHighWaterMarkQueue_[3] =
    statisticRegistrar.registerNumeric<std::uint64_t>("highWaterMarkQueue3",
                                                      StatisticProperties::CLEARABLE,
                                                      "High water mark queue 3");

  pHighWaterMarkQueue_[4] =
    statisticRegistrar.registerNumeric<std::uint64_t>("highWaterMarkQueue4",
                                                      StatisticProperties::CLEARABLE,
                                                      "High water mark queue 4");
}

void EMANE::Models::TDMA::QueueStatusPublisher::drop(std::uint8_t u8Queue,
                                                     DropReason reason,
                                                     size_t count)
{

  auto & overflow = std::get<2>(statusTableInfo_[u8Queue]);
  auto & toobig = std::get<3>(statusTableInfo_[u8Queue]);

  switch(reason)
    {
    case DropReason::DROP_OVERFLOW:
      overflow += count;
      pQueueStatusTable_->setCell(u8Queue,3,Any{overflow});
      break;

    case DropReason::DROP_TOOBIG:
      toobig += count;
      pQueueStatusTable_->setCell(u8Queue,4,Any{toobig});
      break;
    }

  depthQueueInfo_[u8Queue] -= count;
}

void EMANE::Models::TDMA::QueueStatusPublisher::dequeue(std::uint8_t u8RequestQueue,
                                                        std::uint8_t u8ActualQueue,
                                                        const MessageComponents & components)
{
  auto & dequeued =  std::get<1>(statusTableInfo_[u8ActualQueue]);
  auto & queue0 = std::get<4>(statusTableInfo_[u8ActualQueue]);
  auto & queue1 = std::get<5>(statusTableInfo_[u8ActualQueue]);
  auto & queue2 = std::get<6>(statusTableInfo_[u8ActualQueue]);
  auto & queue3 = std::get<7>(statusTableInfo_[u8ActualQueue]);
  auto & queue4 = std::get<8>(statusTableInfo_[u8ActualQueue]);

  size_t packetsCompletedSend{};

  for(const auto & component : components)
    {
      if(!component.isMoreFragments())
        {
          ++packetsCompletedSend;

          size_t index{};
          size_t parts{component.getFragmentIndex() + 1};

          if(parts <= 9)
            {
              index = parts;
            }
          else
            {
              index = 10;
            }

          ++fragmentHistogram_[u8ActualQueue][index-1];

          pQueueFragmentHistogram_->setCell(u8ActualQueue,
                                            index,
                                            Any{fragmentHistogram_[u8ActualQueue][index-1]});

        }
    }

  dequeued += packetsCompletedSend;

  pQueueStatusTable_->setCell(u8ActualQueue,2,Any{dequeued});

  switch(u8RequestQueue)
    {
  case 0:
    queue0 += components.size();
    pQueueStatusTable_->setCell(u8ActualQueue,5,Any{queue0});
    break;
  case 1:
    queue1 += components.size();
    pQueueStatusTable_->setCell(u8ActualQueue,6,Any{queue1});
    break;
  case 2:
    queue2 += components.size();
    pQueueStatusTable_->setCell(u8ActualQueue,7,Any{queue2});
    break;
  case 3:
    queue3 += components.size();
    pQueueStatusTable_->setCell(u8ActualQueue,8,Any{queue3});
    break;
  case 4:
    queue4 += components.size();
    pQueueStatusTable_->setCell(u8ActualQueue,9,Any{queue4});
    break;
    }

  depthQueueInfo_[u8ActualQueue] -= packetsCompletedSend;
}

void EMANE::Models::TDMA::QueueStatusPublisher::enqueue(std::uint8_t u8Queue)
{
  auto & enqueued = std::get<0>(statusTableInfo_[u8Queue]);
  pQueueStatusTable_->setCell(u8Queue,1,Any{++enqueued});

  ++depthQueueInfo_[u8Queue];

  if(depthQueueInfo_[u8Queue] > highWaterMarkQueueInfo_[u8Queue])
    {
      highWaterMarkQueueInfo_[u8Queue] = depthQueueInfo_[u8Queue];

      *pHighWaterMarkQueue_[u8Queue] = highWaterMarkQueueInfo_[u8Queue];
    }
}
