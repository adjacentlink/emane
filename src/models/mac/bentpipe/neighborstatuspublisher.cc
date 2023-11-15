/*
 * Copyright (c) 2023 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "neighborstatuspublisher.h"

// specialized hash for NeighborStatusTable
namespace std
{
  template<>
  struct hash<std::pair<EMANE::NEMId,EMANE::Models::BentPipe::TransponderIndex>>
  {
    typedef std::pair<EMANE::NEMId,EMANE::Models::BentPipe::TransponderIndex> argument_type;
    typedef std::uint32_t result_type;

    result_type operator()(argument_type const& s) const
    {
      result_type const h1{s.first};
      result_type const h2{s.second};
      return (h1 << 2) ^ h2;
    }
  };
}

namespace
{
  const EMANE::StatisticTableLabels NeighborStatusLabels =
    {
      "NEM",
      "Transponder",
      "SINR_wma",
      "NF_wma",
      "Samples",
      "SINR_avg",
      "NF_avg",
      "Timestamp"
    };

  enum NeighborStatusColumn
    {
      ACCEPT_COLUMN_NEM = 0,
      ACCEPT_COLUMN_TRANSPONDER = 1,
      ACCEPT_COLUMN_SINR_WMA = 2,
      ACCEPT_COLUMN_NOISEFLOOR_WMA = 3,
      ACCEPT_COLUMN_SAMPLES = 4,
      ACCEPT_COLUMN_SINR_AVG = 5,
      ACCEPT_COLUMN_NOISEFLOOR_AVG = 6,
      ACCEPT_COLUMN_TIMESTAMP = 7,
    };
}

EMANE::Models::BentPipe::NeighborStatusPublisher::NeighborStatusPublisher(){}


EMANE::Models::BentPipe::NeighborStatusPublisher::~NeighborStatusPublisher(){}


void EMANE::Models::BentPipe::NeighborStatusPublisher::registerStatistics(StatisticRegistrar & statisticRegistrar)
{
  pNeighborStatusTable_ =
    statisticRegistrar.registerTable<NeighborStatusKey>("NeighborStatusTable",
                                                        NeighborStatusLabels,
                                                        StatisticProperties::NONE,
                                                        "Neighbor status table");
}

void EMANE::Models::BentPipe::NeighborStatusPublisher::update(NEMId remote,
                                                              TransponderIndex transponderIndex,
                                                              double dSINR,
                                                              double dNoiseFloordB,
                                                              const TimePoint & timestamp)
{
  auto key = std::make_pair(remote,transponderIndex);

  std::uint64_t u64TimestampMicroseconds =
    std::chrono::duration_cast<EMANE::Microseconds>(timestamp.time_since_epoch()).count();

  auto iter = known_.find(key);

  if(iter == known_.end())
    {
      iter = known_.emplace(key,NeighborInfo{}).first;

      pNeighborStatusTable_->addRow(key,
                                    {Any{remote},
                                     Any{transponderIndex},
                                     Any{0L},
                                     Any{0L},
                                     Any{0L},
                                     Any{0L},
                                     Any{0L},
                                     Any{0L}});
    }

  iter->second.u64Samples_ += 1;

  iter->second.dSINRAccum_ += dSINR;
  iter->second.dNoiseFloordBAccum_ += dNoiseFloordB;

  iter->second.wmaSINR_.update(dSINR);
  iter->second.wmaNoiseFloordB_.update(dNoiseFloordB);

  pNeighborStatusTable_->setCell(key,
                                 ACCEPT_COLUMN_TIMESTAMP,
                                 Any{u64TimestampMicroseconds});

  pNeighborStatusTable_->setCell(key,
                                 ACCEPT_COLUMN_SINR_WMA,
                                 Any{iter->second.wmaSINR_.value()});

  pNeighborStatusTable_->setCell(key,
                                 ACCEPT_COLUMN_NOISEFLOOR_WMA,
                                 Any{iter->second.wmaNoiseFloordB_.value()});

  pNeighborStatusTable_->setCell(key,
                                 ACCEPT_COLUMN_SAMPLES,
                                 Any{iter->second.u64Samples_});

  pNeighborStatusTable_->setCell(key,
                                 ACCEPT_COLUMN_SINR_AVG,
                                 Any{iter->second.dSINRAccum_ / iter->second.u64Samples_});

  pNeighborStatusTable_->setCell(key,
                                 ACCEPT_COLUMN_NOISEFLOOR_AVG,
                                 Any{iter->second.dNoiseFloordBAccum_ / iter->second.u64Samples_});

}
