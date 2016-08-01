/*
 * Copyright (c) 2013,2016 - Adjacent Link LLC, Bridgewater,
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

#include "shimheader.h"
#include "commeffectshimheader.pb.h"

EMANE::Models::CommEffect::ShimHeader::ShimHeader(const void * p, size_t len)
{
  EMANEMessage::CommEffectShimHeader msg;

  if(!msg.ParseFromArray(p,len))
    {
      throw SerializationException("unable to deserialize CommEffectShimHeader");
    }

  txTimePoint_ = TimePoint{static_cast<Microseconds>(msg.txtimemicroseconds())};

  u32GroupId_ = msg.groupid();

  u32SequenceNumber_ = msg.sequencenumber();
}

EMANE::Models::CommEffect::ShimHeader::ShimHeader(const TimePoint & txTimePoint,
                                                  std::uint32_t u32GroupId,
                                                  std::uint32_t u32SequenceNumber):
  txTimePoint_{txTimePoint},
  u32GroupId_{u32GroupId},
  u32SequenceNumber_{u32SequenceNumber}
{}

EMANE::Serialization EMANE::Models::CommEffect::ShimHeader::serialize() const
{
  EMANEMessage::CommEffectShimHeader msg;

  msg.set_txtimemicroseconds(std::chrono::duration_cast<Microseconds>(txTimePoint_.time_since_epoch()).count());

  msg.set_groupid(u32GroupId_);

  msg.set_sequencenumber(u32SequenceNumber_);

  Serialization serialization;

  if(!msg.SerializeToString(&serialization))
    {
      throw SerializationException("unable to serialize CommEffectShimHeader");
    }

  return serialization;
}
