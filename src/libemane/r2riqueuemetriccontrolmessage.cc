/*
 * Copyright (c) 2013-2014,2016 - Adjacent Link LLC, Bridgewater,
 * New Jersey
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

#include "emane/controls/r2riqueuemetriccontrolmessage.h"
#include "radiotorouter.pb.h"

class EMANE::Controls::R2RIQueueMetricControlMessage::Implementation
{
public:
  Implementation(){}

  Implementation(const R2RIQueueMetrics & queueMetrics):
    queueMetrics_{queueMetrics}{}

  const R2RIQueueMetrics & getQueueMetrics() const
  {
    return queueMetrics_;
  }

private:
  const R2RIQueueMetrics queueMetrics_;
};

EMANE::Controls::R2RIQueueMetricControlMessage::
R2RIQueueMetricControlMessage(const R2RIQueueMetricControlMessage & msg):
  ControlMessage{IDENTIFIER},
  pImpl_{new Implementation{*msg.pImpl_}}
{}

EMANE::Controls::R2RIQueueMetricControlMessage::~R2RIQueueMetricControlMessage(){}

EMANE::Controls::R2RIQueueMetricControlMessage::R2RIQueueMetricControlMessage(const R2RIQueueMetrics & queueMetrics):
  ControlMessage{IDENTIFIER},
  pImpl_{new Implementation{queueMetrics}}{}

const EMANE::Controls::R2RIQueueMetrics &
EMANE::Controls::R2RIQueueMetricControlMessage::getQueueMetrics() const
{
  return pImpl_->getQueueMetrics();
}

EMANE::Controls::R2RIQueueMetricControlMessage *
EMANE::Controls::R2RIQueueMetricControlMessage::create(const R2RIQueueMetrics & queueMetrics)
{
  return new R2RIQueueMetricControlMessage{queueMetrics};
}

EMANE::Serialization EMANE::Controls::R2RIQueueMetricControlMessage::serialize() const
{
  Serialization serialization;

  EMANEMessage::RadioToRouterQueueMetrics msg;

  const R2RIQueueMetrics & metrics = pImpl_->getQueueMetrics();

  R2RIQueueMetrics::const_iterator iter = metrics.begin();

  for(;iter != metrics.end(); ++iter)
    {
      auto pQueueMetric = msg.add_metrics();

      pQueueMetric->set_queueid(iter->getQueueId());
      pQueueMetric->set_maxsize(iter->getMaxSize());
      pQueueMetric->set_currentdepth(iter->getCurrentDepth());
      pQueueMetric->set_numdiscards(iter->getNumDiscards());
      pQueueMetric->set_avgdelay(std::chrono::duration_cast<DoubleSeconds>(iter->getAvgDelay()).count());
    }

  if(!msg.SerializeToString(&serialization))
    {
      throw SerializationException("unable to serialize R2RIQueueMetricControlMessage");
    }

  return serialization;
}

EMANE::Controls::R2RIQueueMetricControlMessage *
EMANE::Controls::R2RIQueueMetricControlMessage::create(const Serialization & serialization)
{
  EMANEMessage::RadioToRouterQueueMetrics msg;

  if(!msg.ParseFromString(serialization))
    {
      throw SerializationException("unable to deserialize : R2RIQueueMetricControlMessage");
    }

  R2RIQueueMetrics metrics;

  for(const auto & metric : msg.metrics())
    {
      metrics.push_back(R2RIQueueMetric{metric.queueid(),
            metric.maxsize(),
            metric.currentdepth(),
            metric.numdiscards(),
            std::chrono::duration_cast<Microseconds>(DoubleSeconds{metric.avgdelay()})});

    }

  return new R2RIQueueMetricControlMessage{metrics};
}


EMANE::Controls::R2RIQueueMetricControlMessage *
EMANE::Controls::R2RIQueueMetricControlMessage::clone() const
{
  return new R2RIQueueMetricControlMessage{*this};
}
