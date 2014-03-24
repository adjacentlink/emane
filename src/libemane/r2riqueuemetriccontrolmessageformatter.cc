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

#include "emane/controls/r2riqueuemetriccontrolmessageformatter.h"

EMANE::Controls::R2RIQueueMetricControlMessageFormatter::
R2RIQueueMetricControlMessageFormatter(const R2RIQueueMetricControlMessage * pMsg):
  pMsg_{pMsg}{}

EMANE::Strings EMANE::Controls::R2RIQueueMetricControlMessageFormatter::operator()() const
{
  Strings strings{};
  
  for(const auto & metric : pMsg_->getQueueMetrics())
    {
      strings.push_back("queue: " + std::to_string(metric.getQueueId()));
      strings.push_back("max size: " + std::to_string(metric.getMaxSize()));
      strings.push_back("current size: " + std::to_string(metric.getCurrentDepth()));
      strings.push_back("num discards size: " + std::to_string(metric.getNumDiscards()));
      strings.push_back("avg delay: " + std::to_string(std::chrono::duration_cast
                                        <EMANE::DoubleSeconds>(metric.getAvgDelay()).count()));
    }
  
  return strings;
}

