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

#include "emane/controls/r2riselfmetriccontrolmessage.h"
#include "radiotorouter.pb.h"

class EMANE::Controls::R2RISelfMetricControlMessage::Implementation
{
public:

  Implementation():
    u64BroadcastDataRatebps_{},
    u64MaxDataRatebps_{},
    reportInteral_{Microseconds::zero()}{}
  
  Implementation(std::uint64_t u64BroadcastDataRatebps,
                 std::uint64_t u64MaxDataRatebps,
                 const Microseconds & reportInteral):
    u64BroadcastDataRatebps_{u64BroadcastDataRatebps},
    u64MaxDataRatebps_{u64MaxDataRatebps},
    reportInteral_{reportInteral}{}
  
  std::uint64_t getBroadcastDataRatebps() const
  {
    return u64BroadcastDataRatebps_;
  }

  std::uint64_t getMaxDataRatebps() const
  {
    return u64MaxDataRatebps_;
  }

  const Microseconds & getReportInterval() const
  {
    return reportInteral_;
  }
  
private:
  const std::uint64_t u64BroadcastDataRatebps_;
  const std::uint64_t u64MaxDataRatebps_;
  const Microseconds reportInteral_;
};


EMANE::Controls::R2RISelfMetricControlMessage::
R2RISelfMetricControlMessage(std::uint64_t u64BroadcastDataRatebps,
                             std::uint64_t u64MaxDataRatebps,
                             const Microseconds & tvReportInteral):
  ControlMessage{IDENTIFIER},
  pImpl_{new Implementation{u64BroadcastDataRatebps,u64MaxDataRatebps,tvReportInteral}}{}

EMANE::Controls::R2RISelfMetricControlMessage::~R2RISelfMetricControlMessage()
{}

std::uint64_t EMANE::Controls::R2RISelfMetricControlMessage::getBroadcastDataRatebps() const
{
  return  pImpl_->getBroadcastDataRatebps();
}
  
std::uint64_t EMANE::Controls::R2RISelfMetricControlMessage::getMaxDataRatebps() const
{
  return  pImpl_->getMaxDataRatebps();
}

const EMANE::Microseconds & EMANE::Controls::R2RISelfMetricControlMessage::getReportInterval() const
{
  return  pImpl_->getReportInterval();
}

EMANE::Controls::R2RISelfMetricControlMessage *
EMANE::Controls::R2RISelfMetricControlMessage::create(std::uint64_t u64BroadcastDataRatebps,
                                                     std::uint64_t u64MaxDataRatebps,
                                                     const Microseconds & reportInteral)
{
  return new R2RISelfMetricControlMessage{u64BroadcastDataRatebps,
                                          u64MaxDataRatebps,
                                          reportInteral};
}

EMANE::Serialization EMANE::Controls::R2RISelfMetricControlMessage::serialize() const
{
  Serialization serialization;

  EMANEMessage::RadioToRouterSelfMetric msg;

  msg.set_broadcastdataratebps(pImpl_->getBroadcastDataRatebps());
  msg.set_maxdataratebps(pImpl_->getMaxDataRatebps());
  msg.set_reportinterval(std::chrono::duration_cast<DoubleSeconds>(pImpl_->getReportInterval()).count());
                         
  try
    {
      if(!msg.SerializeToString(&serialization))
        {
          throw SerializationException("unable to serialize R2RISelfMetricControlMessage");
        }
    }
  catch(google::protobuf::FatalException & exp)
    {
      throw SerializationException("unable to serialize R2RISelfMetricControlMessage");
    }
  
  return serialization;
}
    
EMANE::Controls::R2RISelfMetricControlMessage * 
EMANE::Controls::R2RISelfMetricControlMessage::create(const Serialization & serialization)
{
  EMANEMessage::RadioToRouterSelfMetric msg;

  try
    {
      if(!msg.ParseFromString(serialization))
        {
          throw SerializationException("unable to deserialize : R2RISelfMetricControlMessage");
        }
    }
  catch(google::protobuf::FatalException & exp)
    {
      throw SerializationException("unable to deserialize  : R2RISelfMetricControlMessage");
    }

  return new R2RISelfMetricControlMessage{msg.broadcastdataratebps(),
      msg.maxdataratebps(),
      std::chrono::duration_cast<Microseconds>(DoubleSeconds{msg.reportinterval()})};
  
}



