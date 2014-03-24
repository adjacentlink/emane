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

#include "emane/events/commeffectevent.h"
#include "commeffectevent.pb.h"

class EMANE::Events::CommEffectEvent::Implementation
{
public:
  Implementation(const CommEffects & commEffects):
    commEffects_{commEffects}{}

  const CommEffects & getCommEffects() const
  {
    return commEffects_;
  }

private:
  CommEffects commEffects_;
};

EMANE::Events::CommEffectEvent::CommEffectEvent(const Serialization & serialization):
  Event(IDENTIFIER)
{
  EMANEMessage::CommEffectEvent msg;

  try
    {
      if(!msg.ParseFromString(serialization))
        {
          throw SerializationException("unable to deserialize CommEffectEvent");
        }
    }
  catch(google::protobuf::FatalException & exp)
    {
      throw SerializationException("unable to deserialize CommEffectEvent");
    }

  using RepeatedPtrFieldCommEffect = 
    google::protobuf::RepeatedPtrField<EMANEMessage::CommEffectEvent::CommEffect>;
  
  CommEffects commEffects;
  
  for(const auto & repeatedCommEffect : RepeatedPtrFieldCommEffect(msg.commeffects()))
    {
      commEffects.push_back({static_cast<EMANE::NEMId>(repeatedCommEffect.nemid()),
            std::chrono::duration_cast<Microseconds>(DoubleSeconds{repeatedCommEffect.latencyseconds()}),
            std::chrono::duration_cast<Microseconds>(DoubleSeconds{repeatedCommEffect.jitterseconds()}),
            repeatedCommEffect.probabilityloss(),
            repeatedCommEffect.probabilityduplicate(),
            repeatedCommEffect.unicastbitratebps(),
            repeatedCommEffect.broadcastbitratebps(),
            });
    }

  pImpl_.reset(new Implementation{commEffects});
}
    
EMANE::Events::CommEffectEvent::CommEffectEvent(const CommEffects & commEffects):
  Event{IDENTIFIER},
  pImpl_{new Implementation{commEffects}}{}
    
EMANE::Events::CommEffectEvent::CommEffectEvent(const CommEffectEvent & rhs):
  Event{IDENTIFIER},
  pImpl_{new Implementation{rhs.getCommEffects()}}{}
   
EMANE::Events::CommEffectEvent & EMANE::Events::CommEffectEvent::operator=(const CommEffectEvent & rhs)
{
  pImpl_.reset(new Implementation{rhs.getCommEffects()});
  return *this;
}
    
EMANE::Events::CommEffectEvent::CommEffectEvent(CommEffectEvent && rval):
  Event{IDENTIFIER},
  pImpl_{new Implementation{{}}}
{
  rval.pImpl_.swap(pImpl_);
}
    
EMANE::Events::CommEffectEvent & 
EMANE::Events::CommEffectEvent::operator=(CommEffectEvent && rval)
{
  rval.pImpl_.swap(pImpl_);
  return *this;
}

EMANE::Events::CommEffectEvent::~CommEffectEvent(){}
    
const EMANE::Events::CommEffects & 
EMANE::Events::CommEffectEvent::getCommEffects() const
{
  return pImpl_->getCommEffects();
}

EMANE::Serialization EMANE::Events::CommEffectEvent::serialize() const
{
  Serialization serialization;

  EMANEMessage::CommEffectEvent msg;

  for(auto & commEffect : pImpl_->getCommEffects())
    {
      auto pCommEffectMessage = msg.add_commeffects();

      pCommEffectMessage->set_nemid(commEffect.getNEMId());
      pCommEffectMessage->set_latencyseconds(std::chrono::duration_cast<DoubleSeconds>(commEffect.getLatency()).count());
      pCommEffectMessage->set_jitterseconds(std::chrono::duration_cast<DoubleSeconds>(commEffect.getJitter()).count());
      pCommEffectMessage->set_probabilityloss(commEffect.getProbabilityLoss());
      pCommEffectMessage->set_probabilityduplicate(commEffect.getProbabilityDuplicate());
      pCommEffectMessage->set_unicastbitratebps(commEffect.getUnicastBitRate());
      pCommEffectMessage->set_broadcastbitratebps(commEffect.getBroadcastBitRate());
    }

  try
    {
      if(!msg.SerializeToString(&serialization))
        {
          throw SerializationException("unable to serialize CommEffectEvent");
        }
    }
  catch(google::protobuf::FatalException & exp)
    {
      throw SerializationException("unable to serialize CommEffect<Event");
    }

  return serialization;
}
