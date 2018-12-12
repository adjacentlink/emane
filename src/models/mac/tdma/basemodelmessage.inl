/*
 * Copyright (c) 2015-2016 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "tdmabasemodelmessage.pb.h"

inline
EMANE::Models::TDMA::BaseModelMessage::BaseModelMessage():
  u64AbsoluteSlotIndex_{},
  u64DataRatebps_{}{}

inline
EMANE::Models::TDMA::BaseModelMessage::BaseModelMessage(std::uint64_t u64AbsoluteSlotIndex,
                                                        std::uint64_t u64DataRatebps,
                                                        MessageComponents && messages):
  u64AbsoluteSlotIndex_{u64AbsoluteSlotIndex},
  u64DataRatebps_{u64DataRatebps},
  messages_{std::move(messages)}{}

inline
EMANE::Models::TDMA::BaseModelMessage::BaseModelMessage(const void * p, size_t len)
{
  EMANEMessage::TDMABaseModelMessage message;

  if(!message.ParseFromArray(p, len))
    {
      throw SerializationException("unable to deserialize TDMABaseModelMessage");
    }

  u64AbsoluteSlotIndex_ = message.absslotindex();
  u64DataRatebps_ = message.dataratebps();

  for(const auto & msg : message.messages())
    {
      MessageComponent::Type type;

      switch(msg.type())
        {
        case EMANEMessage::TDMABaseModelMessage::Message::DATA:
          type = MessageComponent::Type::DATA;
          break;
        case EMANEMessage::TDMABaseModelMessage::Message::CONTROL:
          type = MessageComponent::Type::CONTROL;
          break;
        default:
          throw SerializationException("TDMABaseModelMessage unkown type");
        }

      if(msg.has_fragment())
        {
          const auto & fragment = msg.fragment();

          messages_.push_back({type,
                static_cast<NEMId>(msg.destination()),
                static_cast<Priority>(msg.priority()),
                  {Utils::make_iovec(const_cast<char *>(msg.data().c_str()),
                                     msg.data().size())},
                fragment.index(),
                  fragment.offset(),
                  fragment.sequence(),
                  fragment.more()});
        }
      else
        {
          messages_.push_back({type,
                static_cast<NEMId>(msg.destination()),
                static_cast<Priority>(msg.priority()),
                  {Utils::make_iovec(const_cast<char *>(msg.data().c_str()),
                                     msg.data().size())}});
        }
    }
}

inline
const EMANE::Models::TDMA::MessageComponents &
EMANE::Models::TDMA::BaseModelMessage::getMessages() const
{
  return messages_;
}

inline
std::uint64_t EMANE::Models::TDMA::BaseModelMessage::getAbsoluteSlotIndex() const
{
  return u64AbsoluteSlotIndex_;
}

inline
std::uint64_t EMANE::Models::TDMA::BaseModelMessage::getDataRate() const
{
  return u64DataRatebps_;
}

inline
EMANE::Serialization EMANE::Models::TDMA::BaseModelMessage::serialize() const
{
  Serialization serialization{};

  EMANEMessage::TDMABaseModelMessage baseModelMessage{};

  baseModelMessage.set_absslotindex(u64AbsoluteSlotIndex_);
  baseModelMessage.set_dataratebps(u64DataRatebps_);

  for(const auto & message : messages_)
    {
      auto pMessage = baseModelMessage.add_messages();

      pMessage->set_destination(message.getDestination());
      pMessage->set_priority(message.getPriority());

      switch(message.getType())
        {
        case MessageComponent::Type::DATA:
          pMessage->set_type(EMANEMessage::TDMABaseModelMessage::Message::DATA);
          break;
        case MessageComponent::Type::CONTROL:
          pMessage->set_type(EMANEMessage::TDMABaseModelMessage::Message::CONTROL);
          break;
        default:
          throw SerializationException("TDMABaseModelMessage unkown type");
        }

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
      throw SerializationException("unable to serialize TDMABaseModelMessage");
    }


  return serialization;
}
