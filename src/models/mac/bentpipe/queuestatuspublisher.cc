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

#include "queuestatuspublisher.h"

namespace
{
  const EMANE::StatisticTableLabels QueueStatusTableLabels =
    {
      "Transponder",
      "Enqueued",
      "Dequeued",
      "Overflow",
      "Too Big",
      "Depth",
      "High Water"
    };

  enum QueueStatusTableColumn
    {
      QUEUE_STATUS_COLUMN_TRANSPONDER = 0,
      QUEUE_STATUS_COLUMN_ENQUEUED = 1,
      QUEUE_STATUS_COLUMN_DEQUEUED = 2,
      QUEUE_STATUS_COLUMN_OVERFLOW = 3,
      QUEUE_STATUS_COLUMN_TOO_BIG = 4,
      QUEUE_STATUS_COLUMN_DEPTH = 5,
      QUEUE_STATUS_COLUMN_HIGH_WATER = 6
    };

  const EMANE::StatisticTableLabels QueueFragmentHistogramLabels =
    {
      "Transponder",
      "1",
      "2",
      "3",
      "4",
      "5",
      "6",
      "7",
      "8",
      "9",
      ">9",
    };

  const EMANE::StatisticTableLabels QueueAggregateHistogramLabels =
    {
      "Transponder",
      "1",
      "2",
      "3",
      "4",
      "5",
      "6",
      "7",
      "8",
      "9",
      ">9",
    };
}


EMANE::Models::BentPipe::QueueStatusPublisher::QueueStatusPublisher():
  pQueueStatusTable_{},
  statusTableInfo_{},
  fragmentHistogram_{},
  aggregateHistogram_{}
{}

void EMANE::Models::BentPipe::QueueStatusPublisher::registerStatistics(StatisticRegistrar & statisticRegistrar)
{
  pQueueStatusTable_ =
    statisticRegistrar.registerTable<TransponderIndex>("QueueStatusTable",
                                                       QueueStatusTableLabels,
                                                       StatisticProperties::NONE,
                                                       "Shows for each Transponder queue the number of"
                                                       " packets enqueued, dequeued, dropped due to queue"
                                                       " overflow (enqueue), dropped due to too big,"
                                                       " (dequeue), depth, and high water mark.");

  pQueueFragmentHistogram_ =
    statisticRegistrar.registerTable<TransponderIndex>("QueueFragmentHistogram",
                                                       QueueFragmentHistogramLabels,
                                                       StatisticProperties::NONE,
                                                       "Shows a per transponder histogram of the number"
                                                       " of message components required to transmit a single"
                                                       " packet over-the-air.");

  pQueueAggregateHistogram_ = statisticRegistrar.registerTable<TransponderIndex>("QueueAggregateHistogram",
                                                                                 QueueAggregateHistogramLabels,
                                                                                 StatisticProperties::NONE,
                                                                                 "Shows a per transponder histogram of the number"
                                                                                 " of message components contained in an single"
                                                                                 " over-the-air frame.");

}

void EMANE::Models::BentPipe::QueueStatusPublisher::drop(TransponderIndex transponderIndex,
                                                         DropReason reason,
                                                         size_t count)
{
  if(auto iter = statusTableInfo_.find(transponderIndex);
     iter != statusTableInfo_.end())
    {
      auto & overflow = std::get<QUEUE_STATUS_COLUMN_OVERFLOW-1>(iter->second);
      auto & toobig = std::get<QUEUE_STATUS_COLUMN_TOO_BIG-1>(iter->second);
      auto & depth = std::get<QUEUE_STATUS_COLUMN_DEPTH-1>(iter->second);

      switch(reason)
        {
        case DropReason::DROP_OVERFLOW:
          overflow += count;
          pQueueStatusTable_->setCell(transponderIndex,QUEUE_STATUS_COLUMN_OVERFLOW,Any{overflow});
          break;

        case DropReason::DROP_TOOBIG:
          toobig += count;
          pQueueStatusTable_->setCell(transponderIndex,QUEUE_STATUS_COLUMN_TOO_BIG,Any{toobig});
          break;
        }

      depth -= count;
      pQueueStatusTable_->setCell(transponderIndex,QUEUE_STATUS_COLUMN_DEPTH,Any{depth});
    }
}

void EMANE::Models::BentPipe::QueueStatusPublisher::dequeue(TransponderIndex transponderIndex,
                                                            const MessageComponents & components)
{
  if(auto iter = statusTableInfo_.find(transponderIndex);
     iter != statusTableInfo_.end())
    {
      iter = statusTableInfo_.emplace(transponderIndex,StatusTableEntry{}).first;

      auto & dequeued =  std::get<QUEUE_STATUS_COLUMN_DEQUEUED-1>(iter->second);
      auto & depth = std::get<QUEUE_STATUS_COLUMN_DEPTH-1>(iter->second);

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

              if(auto iter = fragmentHistogram_.find(transponderIndex);
                 iter != fragmentHistogram_.end())
                {
                  ++iter->second.at(index-1);


                  pQueueFragmentHistogram_->setCell(transponderIndex,
                                                    index,
                                                    Any{iter->second.at(index-1)});
                }

            }
        }

      size_t index{components.size()};

      if(index > 9)
        {
          index = 10;
        }

      if(auto iter = aggregateHistogram_.find(transponderIndex);
         iter != aggregateHistogram_.end())
        {
          ++iter->second.at(index-1);

          pQueueAggregateHistogram_->setCell(transponderIndex,
                                             index,
                                             Any{iter->second.at(index-1)});
        }

      dequeued += packetsCompletedSend;

      pQueueStatusTable_->setCell(transponderIndex,QUEUE_STATUS_COLUMN_DEQUEUED,Any{dequeued});

      depth -= packetsCompletedSend;

      pQueueStatusTable_->setCell(transponderIndex,QUEUE_STATUS_COLUMN_DEPTH,Any{depth});
    }
}

void EMANE::Models::BentPipe::QueueStatusPublisher::enqueue(TransponderIndex transponderIndex)
{
  auto iter = statusTableInfo_.find(transponderIndex);

  if(iter == statusTableInfo_.end())
    {
      iter = statusTableInfo_.emplace(transponderIndex,StatusTableEntry{}).first;

      fragmentHistogram_.emplace(transponderIndex,FragmentHistogramEntry{});

      pQueueStatusTable_->addRow(transponderIndex,
                                 {Any{transponderIndex},
                                  Any{0UL},
                                  Any{0UL},
                                  Any{0UL},
                                  Any{0UL},
                                  Any{0UL},
                                  Any{0UL}});

      pQueueFragmentHistogram_->addRow(transponderIndex,
                                       {Any{transponderIndex},
                                        Any{0UL},
                                        Any{0UL},
                                        Any{0UL},
                                        Any{0UL},
                                        Any{0UL},
                                        Any{0UL},
                                        Any{0UL},
                                        Any{0UL},
                                        Any{0UL},
                                        Any{0UL}});

      aggregateHistogram_.emplace(transponderIndex,FragmentHistogramEntry{});

      pQueueAggregateHistogram_->addRow(transponderIndex,
                                        {Any{transponderIndex},
                                         Any{0UL},
                                         Any{0UL},
                                         Any{0UL},
                                         Any{0UL},
                                         Any{0UL},
                                         Any{0UL},
                                         Any{0UL},
                                         Any{0UL},
                                         Any{0UL},
                                         Any{0UL}});
    }

  auto & enqueued = std::get<QUEUE_STATUS_COLUMN_ENQUEUED-1>(iter->second);
  auto & depth =  std::get<QUEUE_STATUS_COLUMN_DEPTH-1>(iter->second);
  auto & highwater = std::get<QUEUE_STATUS_COLUMN_HIGH_WATER-1>(iter->second);

  ++depth;
  pQueueStatusTable_->setCell(transponderIndex,QUEUE_STATUS_COLUMN_DEPTH,Any{depth});

  ++enqueued;
  pQueueStatusTable_->setCell(transponderIndex,QUEUE_STATUS_COLUMN_ENQUEUED,Any{enqueued});

  if(depth > highwater)
    {
      highwater = depth;

      pQueueStatusTable_->setCell(transponderIndex,QUEUE_STATUS_COLUMN_HIGH_WATER,Any{highwater});
    }
}
