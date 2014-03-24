/*
 * Copyright (c) 2014 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "emane/controls/r2rineighbormetriccontrolmessageformatter.h"

EMANE::Controls::R2RINeighborMetricControlMessageFormatter::
R2RINeighborMetricControlMessageFormatter(const R2RINeighborMetricControlMessage * pMsg):
  pMsg_{pMsg}{}

EMANE::Strings EMANE::Controls::R2RINeighborMetricControlMessageFormatter::operator()() const
{
  Strings strings{};
  
  for(const auto & metric : pMsg_->getNeighborMetrics())
    {
      strings.push_back("nem: " + std::to_string(metric.getId()));
      strings.push_back("rx frames: " + std::to_string(metric.getNumRxFrames()));
      strings.push_back("tx frames: " + std::to_string(metric.getNumTxFrames()));
      strings.push_back("missed frames: " + std::to_string(metric.getNumMissedFrames()));
      strings.push_back("bandwidth consumptions: " + std::to_string(metric.getBandwidthConsumption().count()));
      strings.push_back("sinr avg: " + std::to_string(metric.getSINRAvgdBm()));
      strings.push_back("sinr stdv: " + std::to_string(metric.getSINRStddev()));
      strings.push_back("noise floor avg: " + std::to_string(metric.getNoiseFloorAvgdBm()));
      strings.push_back("noise floor stdv: " + std::to_string(metric.getNoiseFloorStddev()));
      strings.push_back("rx data rate avg: " + std::to_string(metric.getRxAvgDataRatebps()));
      strings.push_back("tx data rate avg: " + std::to_string(metric.getTxAvgDataRatebps()));
    }
  
  return strings;
}

