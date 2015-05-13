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

#include "tdmabasemodelheader.pb.h"

inline
EMANE::Models::TDMA::BaseModelHeader::BaseModelHeader():
  type_{Type::DATA},
  u64AbsoluteSlotIndex_{},
  u64DataRatebps_{}{}

inline
EMANE::Models::TDMA::BaseModelHeader::BaseModelHeader(Type type,
                                                      std::uint64_t u64AbsoluteSlotIndex,
                                                      std::uint64_t u64DataRatebps):
  type_{type},
  u64AbsoluteSlotIndex_{u64AbsoluteSlotIndex},
  u64DataRatebps_{u64DataRatebps}{}

inline
EMANE::Models::TDMA::BaseModelHeader::BaseModelHeader(const void * p, size_t len)
{
  EMANEMessage::TDMABaseModelHeader message;

  if(!message.ParseFromArray(p, len))
    {
      throw SerializationException("unable to deserialize TDMABaseModelHeader");
    }

  switch(message.type())
    {
    case EMANEMessage::TDMABaseModelHeader::DATA:
      type_ = Type::DATA;
      break;
    case EMANEMessage::TDMABaseModelHeader::CONTROL:
      type_ = Type::CONTROL;
      break;
    default:
      throw SerializationException("TDMABaseModelHeader unkown type");
    }

  u64AbsoluteSlotIndex_ = message.absslotindex();
  u64DataRatebps_ = message.dataratebps();
}

inline
EMANE::Models::TDMA::BaseModelHeader::Type EMANE::Models::TDMA::BaseModelHeader::getType() const
{
  return type_;
}

inline
std::uint64_t EMANE::Models::TDMA::BaseModelHeader::getAbsoluteSlotIndex() const
{
  return u64AbsoluteSlotIndex_;
}

inline
std::uint64_t EMANE::Models::TDMA::BaseModelHeader::getDataRate() const
{
  return u64DataRatebps_;
}

inline
EMANE::Serialization EMANE::Models::TDMA::BaseModelHeader::serialize() const
{
  Serialization serialization{};

  EMANEMessage::TDMABaseModelHeader message{};

  switch(type_)
    {
    case Type::DATA:
      message.set_type(EMANEMessage::TDMABaseModelHeader::DATA);
      break;
    case Type::CONTROL:
      message.set_type(EMANEMessage::TDMABaseModelHeader::CONTROL);
      break;
    default:
      throw SerializationException("TDMABaseModelHeader unkown type");
    }

  message.set_absslotindex(u64AbsoluteSlotIndex_);

  message.set_dataratebps(u64DataRatebps_);
  
  if(!message.SerializeToString(&serialization))
    {
      throw SerializationException("unable to serialize TDMABaseModelHeader");
    }

  return serialization;
}
