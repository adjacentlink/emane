/*
 * Copyright (c) 2025 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "emane/events/pathlossexevent.h"
#include "pathlossexevent.pb.h"

class EMANE::Events::PathlossExEvent::Implementation
{
public:
  Implementation(PathlossExs && pathlossExs):
    pathlossExs_{std::move(pathlossExs)}{}

  Implementation(const PathlossExs & pathlossExs):
    pathlossExs_{pathlossExs}{}

  const PathlossExs & getPathlossExs() const
  {
    return pathlossExs_;
  }

private:
  PathlossExs pathlossExs_;
};

EMANE::Events::PathlossExEvent::PathlossExEvent(const Serialization & serialization):
  Event(IDENTIFIER)
{
  EMANEMessage::PathlossExEvent msg;

  if(!msg.ParseFromString(serialization))
    {
      throw SerializationException("unable to deserialize PathlossExEvent");
    }

  PathlossExs pathlossExs{};

  for(const auto & pathloss : msg.pathlosses())
    {
      PathlossEx::FrequencyPathlossMap frequencyPathlossMap{};

      for(const auto & entry : pathloss.entries())
        {
          frequencyPathlossMap.emplace(entry.frequencyhz(),entry.pathlossdb());
        }

      pathlossExs.emplace_back(static_cast<NEMId>(pathloss.nemid()),
                               std::move(frequencyPathlossMap));
    }

  pImpl_.reset(new Implementation{std::move(pathlossExs)});
}

EMANE::Events::PathlossExEvent::PathlossExEvent(const PathlossExs & pathlossExs):
  Event{IDENTIFIER},
  pImpl_{new Implementation{pathlossExs}}{}

EMANE::Events::PathlossExEvent::PathlossExEvent(const PathlossExEvent & rhs):
  Event{IDENTIFIER},
  pImpl_{new Implementation{rhs.getPathlossExs()}}{}

EMANE::Events::PathlossExEvent &
EMANE::Events::PathlossExEvent::operator=(const PathlossExEvent & rhs)
{
  pImpl_.reset(new Implementation{rhs.getPathlossExs()});
  return *this;
}

EMANE::Events::PathlossExEvent::PathlossExEvent(PathlossExEvent && rval):
  Event{IDENTIFIER},
  pImpl_{new Implementation{{}}}
{
  rval.pImpl_.swap(pImpl_);
}

EMANE::Events::PathlossExEvent &
EMANE::Events::PathlossExEvent::operator=(PathlossExEvent && rval)
{
  rval.pImpl_.swap(pImpl_);
  return *this;
}

EMANE::Events::PathlossExEvent::~PathlossExEvent(){}

const EMANE::Events::PathlossExs &
EMANE::Events::PathlossExEvent::getPathlossExs() const
{
  return pImpl_->getPathlossExs();
}

EMANE::Serialization EMANE::Events::PathlossExEvent::serialize() const
{
  Serialization serialization;

  EMANEMessage::PathlossExEvent msg;

  for(const auto & pathloss : pImpl_->getPathlossExs())
    {
      auto pPathlossMessage = msg.add_pathlosses();

      pPathlossMessage->set_nemid(pathloss.getNEMId());

      for(const auto & entry : pathloss.getFrequencyPathlossMap())
        {
          auto pEntryMessage = pPathlossMessage->add_entries();

          pEntryMessage->set_frequencyhz(entry.first);
          pEntryMessage->set_pathlossdb(entry.second);
        }
    }

  if(!msg.SerializeToString(&serialization))
    {
      throw SerializationException("unable to serialize PathlossExEvent");
    }

  return serialization;
}
