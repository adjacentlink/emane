/*
 * Copyright (c) 2015-2016,2023 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "bentpipemessage.pb.h"

inline
EMANE::Models::BentPipe::BentPipeMessage::BentPipeMessage():
  startOfTransmission_{},
  curveIndex_{}{}

inline
EMANE::Models::BentPipe::BentPipeMessage::BentPipeMessage(const TimePoint & startOfTransmission,
                                                          PCRCurveIndex curveIndex,
                                                          MessageComponents && messages):
  startOfTransmission_{startOfTransmission},
  curveIndex_{curveIndex},
  messages_{std::move(messages)}{}

inline
EMANE::Models::BentPipe::BentPipeMessage::BentPipeMessage(const void * p, size_t len)
{
  EMANEMessage::BentPipeMessage message;

  if(!message.ParseFromArray(p, len))
    {
      throw SerializationException("unable to deserialize BentPipeMessage");
    }

  startOfTransmission_ = TimePoint(Microseconds{message.startoftransmissionmicroseconds()});
  curveIndex_ = message.curveindex();

  for(const auto & msg : message.messages())
    {
      if(msg.has_fragment())
        {
          const auto & fragment = msg.fragment();

          messages_.push_back({static_cast<NEMId>(msg.destination()),
                               {Utils::make_iovec(const_cast<char *>(msg.data().c_str()),
                                                  msg.data().size())},
                               fragment.index(),
                               fragment.offset(),
                               fragment.sequence(),
                               fragment.more()});
        }
      else
        {
          messages_.push_back({static_cast<NEMId>(msg.destination()),
                               {Utils::make_iovec(const_cast<char *>(msg.data().c_str()),
                                                  msg.data().size())}});
        }
    }
}

inline
const EMANE::Models::BentPipe::MessageComponents &
EMANE::Models::BentPipe::BentPipeMessage::getMessages() const
{
  return messages_;
}

inline
const EMANE::TimePoint & EMANE::Models::BentPipe::BentPipeMessage::getStartOfTransmission() const
{
  return startOfTransmission_;
}

inline
EMANE::Models::BentPipe::PCRCurveIndex
EMANE::Models::BentPipe::BentPipeMessage::getPCRCurveIndex() const
{
  return curveIndex_;
}

inline
EMANE::Serialization EMANE::Models::BentPipe::BentPipeMessage::serialize() const
{
  Serialization serialization{};

  EMANEMessage::BentPipeMessage baseModelMessage{};

  baseModelMessage.set_startoftransmissionmicroseconds(std::chrono::duration_cast<Microseconds>(startOfTransmission_.time_since_epoch()).count());
  baseModelMessage.set_curveindex(curveIndex_);

  for(const auto & message : messages_)
    {
      auto pMessage = baseModelMessage.add_messages();

      pMessage->set_destination(message.getDestination());

      auto const & data = message.getData();

      pMessage->set_data(std::string(reinterpret_cast<const char *>(data.data()),data.size()));

      if(message.isFragment())
        {
          auto pFragement = pMessage->mutable_fragment();

          pFragement->set_index(message.getFragmentIndex());
          pFragement->set_offset(message.getFragmentOffset());
          pFragement->set_more(message.isMoreFragments());
          pFragement->set_sequence(message.getFragmentSequence());
        }
    }

  if(!baseModelMessage.SerializeToString(&serialization))
    {
      throw SerializationException("unable to serialize BentPipeMessage");
    }


  return serialization;
}
