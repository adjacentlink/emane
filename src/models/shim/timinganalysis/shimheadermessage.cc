/*
 * Copyright (c) 2013,2016 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "shimheadermessage.h"
#include "shimheader.pb.h"


class EMANE::Models::TimingAnalysis::ShimHeaderMessage::Implementation
{
public:
  Implementation(const EMANE::TimePoint & txTime,
                 std::uint16_t source,
                 std::uint16_t packetId):
    txTime_{txTime},
    source_{source},
    packetId_{packetId}
  { }

  const EMANE::TimePoint & getTxTime() const
  {
    return txTime_;
  }

  std::uint16_t getSource() const
  {
    return source_;
  }

  std::uint16_t getPacketId() const
  {
    return packetId_;
  }

private:
  const EMANE::TimePoint txTime_;
  const std::uint16_t source_;
  const std::uint16_t packetId_;
};


EMANE::Models::TimingAnalysis::ShimHeaderMessage::ShimHeaderMessage(
                                                                    const EMANE::TimePoint & txTime,
                                                                    std::uint16_t source,
                                                                    std::uint16_t packetId) :
  pImpl_{new Implementation{txTime, source, packetId}}
{ }


EMANE::Models::TimingAnalysis::ShimHeaderMessage::ShimHeaderMessage(const void * p, size_t len)
{
  EMANEMessage::SHIMHeader message;

  if(!message.ParseFromArray(p, len))
    {
      throw SerializationException("unable to deserialize ShimHeaderMessage");
    }

  pImpl_.reset(new Implementation{TimePoint{static_cast<Microseconds>(message.txtimemicroseconds())},
        static_cast<std::uint16_t>(message.source()),
          static_cast<std::uint16_t>(message.packetid())});
}



EMANE::Models::TimingAnalysis::ShimHeaderMessage::~ShimHeaderMessage()
{ }


const EMANE::TimePoint & EMANE::Models::TimingAnalysis::ShimHeaderMessage::getTxTime() const
{
  return pImpl_->getTxTime();
}

std::uint16_t EMANE::Models::TimingAnalysis::ShimHeaderMessage::getSource() const
{
  return pImpl_->getSource();
}

std::uint16_t EMANE::Models::TimingAnalysis::ShimHeaderMessage::getPacketId() const
{
  return pImpl_->getPacketId();
}


EMANE::Serialization EMANE::Models::TimingAnalysis::ShimHeaderMessage::serialize() const
{
  Serialization serialization;

  EMANEMessage::SHIMHeader message;

  message.
    set_txtimemicroseconds(std::chrono::duration_cast<Microseconds>(pImpl_->getTxTime().
                                                                    time_since_epoch()).count());

  message.set_source(pImpl_->getSource());

  message.set_packetid(pImpl_->getPacketId());

  if(!message.SerializeToString(&serialization))
    {
      throw SerializationException("unable to serialize ShimHeaderMessage");
    }

  return serialization;
}
