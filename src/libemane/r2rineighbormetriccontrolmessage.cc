/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "emane/controls/r2rineighbormetriccontrolmessage.h"
#include "radiotorouter.pb.h"

class EMANE::Controls::R2RINeighborMetricControlMessage::Implementation
{
public:
  Implementation(){}
  
  Implementation(const R2RINeighborMetrics & neighborMetrics):
    neighborMetrics_{neighborMetrics}{}
  
  const R2RINeighborMetrics & getNeighborMetrics() const
  {
    return neighborMetrics_;
  }
  
private:
  const R2RINeighborMetrics neighborMetrics_;
};

EMANE::Controls::R2RINeighborMetricControlMessage::
R2RINeighborMetricControlMessage(const R2RINeighborMetricControlMessage & msg):
  ControlMessage{IDENTIFIER},
  pImpl_{new Implementation{*msg.pImpl_}}
{}


EMANE::Controls::R2RINeighborMetricControlMessage::R2RINeighborMetricControlMessage(const R2RINeighborMetrics & neighborMetrics):
  ControlMessage(IDENTIFIER),
  pImpl_{new Implementation{neighborMetrics}}{}


EMANE::Controls::R2RINeighborMetricControlMessage::~R2RINeighborMetricControlMessage()
{}

const EMANE::Controls::R2RINeighborMetrics &
EMANE::Controls::R2RINeighborMetricControlMessage::getNeighborMetrics() const
{
  return pImpl_->getNeighborMetrics();
}

EMANE::Controls::R2RINeighborMetricControlMessage *
EMANE::Controls::R2RINeighborMetricControlMessage::create(const R2RINeighborMetrics & neighborMetrics)
{
  return new R2RINeighborMetricControlMessage{neighborMetrics};
}

EMANE::Serialization EMANE::Controls::R2RINeighborMetricControlMessage::serialize() const
{
  EMANE::Serialization serialization;

  EMANEMessage::RadioToRouterNeighborMetrics msg;

  const R2RINeighborMetrics & metrics = pImpl_->getNeighborMetrics();

  R2RINeighborMetrics::const_iterator iter = metrics.begin();

  for(;iter != metrics.end(); ++iter)
    {
      EMANEMessage::RadioToRouterNeighborMetrics::NeighborMetric * pNeighborMetric =
        msg.add_metrics();

      pNeighborMetric->set_neighborid(iter->getId());
      pNeighborMetric->set_numrxframes(iter->getNumRxFrames());
      pNeighborMetric->set_numtxframes(iter->getNumTxFrames());
      pNeighborMetric->set_nummissedframes(iter->getNumMissedFrames());
      pNeighborMetric->set_bandwidthconsumption
        (std::chrono::duration_cast<DoubleSeconds>(iter->getBandwidthConsumption()).count());

      pNeighborMetric->set_sinraverage(iter->getSINRAvgdBm());
      pNeighborMetric->set_sinrstddev(iter->getSINRStddev());
      pNeighborMetric->set_noiseflooravg(iter->getNoiseFloorAvgdBm());
      pNeighborMetric->set_noisefloorstddev(iter->getNoiseFloorStddev());
      pNeighborMetric->set_rxavgdataratebps(iter->getRxAvgDataRatebps());
      pNeighborMetric->set_txavgdataratebps(iter->getTxAvgDataRatebps());
    }
  
  try
    {
      if(!msg.SerializeToString(&serialization))
        {
          throw SerializationException("unable the serialize RadioToRouterNeighborMetrics");
        }
    }
  catch(google::protobuf::FatalException & exp)
    {
      throw SerializationException("unable the serialize RadioToRouterNeighborMetrics");
    }
  
  return serialization;
}
    
EMANE::Controls::R2RINeighborMetricControlMessage *
EMANE::Controls::R2RINeighborMetricControlMessage::create(const Serialization & serialization)
{
  EMANEMessage::RadioToRouterNeighborMetrics msg;
  
  try
    {
      if(!msg.ParseFromString(serialization))
        {
          throw SerializationException("unable to deserialize : R2RINeighborMetricControlMessage");
        }
    }
  catch(google::protobuf::FatalException & exp)
    {
      throw SerializationException("unable to deserialize  : R2RINeighborMetricControlMessage");
    }
    
  R2RINeighborMetrics metrics;

  google::protobuf::RepeatedPtrField<EMANEMessage::RadioToRouterNeighborMetrics::NeighborMetric> 
    repeatedPtrField(msg.metrics());

  for(const auto & neighbor : repeatedPtrField)
    {
      metrics.push_back(R2RINeighborMetric{static_cast<std::uint16_t>(neighbor.neighborid()),
            neighbor.numrxframes(),
            neighbor.numtxframes(),
            neighbor.nummissedframes(),
            std::chrono::duration_cast<Microseconds>(DoubleSeconds{neighbor.bandwidthconsumption()}),
            neighbor.sinraverage(),
            neighbor.sinrstddev(),
            neighbor.noiseflooravg(),
            neighbor.noisefloorstddev(),
            neighbor.rxavgdataratebps(),
            neighbor.txavgdataratebps()});
    }

  return new R2RINeighborMetricControlMessage{metrics};
}


EMANE::Controls::R2RINeighborMetricControlMessage *
EMANE::Controls::R2RINeighborMetricControlMessage::clone() const
{
  return new R2RINeighborMetricControlMessage{*this};
}
