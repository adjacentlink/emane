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

#include "aggregationstatuspublisher.h"

EMANE::Models::TDMA::AggregationStatusPublisher::AggregationStatusPublisher():
  pAggregationHistogramTable_{},
  aggregationHistogram_{}
{}

void EMANE::Models::TDMA::AggregationStatusPublisher::registerStatistics(StatisticRegistrar & statisticRegistrar)
{
  pAggregationHistogramTable_ =
    statisticRegistrar.registerTable<std::uint64_t>("PacketComponentAggregationHistogram",
      {"Components","Count"},
      StatisticProperties::NONE,
      "Shows a histogram of the number of components contained in transmitted messages.");
}

void EMANE::Models::TDMA::AggregationStatusPublisher::update(const MessageComponents & components)
{
  std::uint64_t u64NumberComponents{components.size()};

  auto iter = aggregationHistogram_.find(u64NumberComponents);

  if(iter == aggregationHistogram_.end())
    {
      aggregationHistogram_.insert({u64NumberComponents,1});

      pAggregationHistogramTable_->addRow(u64NumberComponents,
                                          {Any{u64NumberComponents},
                                              Any{1L}});
    }
  else
    {
      ++iter->second;

      pAggregationHistogramTable_->setCell(u64NumberComponents,
                                           1,
                                           Any{iter->second});
    }
}
