/*
 * Copyright (c) 2013,2016 - Adjacent Link LLC, Bridgewater New Jersey
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

#include "onehopneighborsevent.h"
#include "onehopneighborsevent.pb.h"

class EMANE::Models::IEEE80211ABG::OneHopNeighborsEvent::Implementation
{
public:
  Implementation(NEMId id, const NbrSet & neighbors) :
    eventSource_(id),
    nbrSet_{neighbors}
  { }

  const NbrSet & getNeighbors() const
  {
    return nbrSet_;
  }

  NEMId getEventSource() const
  {
    return eventSource_;
  }

private:
  NEMId  eventSource_;

  NbrSet nbrSet_;
};


EMANE::Models::IEEE80211ABG::OneHopNeighborsEvent::OneHopNeighborsEvent(const Serialization & serialization):
  Event(IDENTIFIER)
{
  EMANEEventMessage::OneHopNeighborsEvent msg;

  if(!msg.ParseFromString(serialization))
    {
      throw SerializationException("unable to deserialize : OneHopNeighborsEvent");
    }

  NbrSet neighbors;

  for(const auto & iter : msg.neighbors())
    {
      neighbors.insert(static_cast<NEMId>(iter.nemid()));
    }

  pImpl_.reset(new Implementation{static_cast<NEMId>(msg.eventsource()), neighbors});
}

EMANE::Models::IEEE80211ABG::OneHopNeighborsEvent::OneHopNeighborsEvent(NEMId id, const NbrSet & neighbors):
  Event{IDENTIFIER},
  pImpl_{new Implementation{id, neighbors}}{}



EMANE::Models::IEEE80211ABG::OneHopNeighborsEvent::~OneHopNeighborsEvent(){}


const EMANE::Models::IEEE80211ABG::NbrSet &
EMANE::Models::IEEE80211ABG::OneHopNeighborsEvent::getNeighbors() const
{
  return pImpl_->getNeighbors();
}

EMANE::NEMId
EMANE::Models::IEEE80211ABG::OneHopNeighborsEvent::getEventSource() const
{
  return pImpl_->getEventSource();
}



EMANE::Serialization EMANE::Models::IEEE80211ABG::OneHopNeighborsEvent::serialize() const
{
  Serialization serialization;

  EMANEEventMessage::OneHopNeighborsEvent msg;

  msg.set_eventsource(pImpl_->getEventSource());

  for(auto & neighbor : pImpl_->getNeighbors())
    {
      auto iter = msg.add_neighbors();

      iter->set_nemid(neighbor);
    }

  if(!msg.SerializeToString(&serialization))
    {
      throw SerializationException("unable to serialize : OneHopNeighborsEvent");
    }

  return serialization;
}
