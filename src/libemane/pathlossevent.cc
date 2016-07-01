/*
 * Copyright (c) 2013-2014,2016 - Adjacent Link LLC, Bridgewater,
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

#include "emane/events/pathlossevent.h"
#include "pathlossevent.pb.h"

class EMANE::Events::PathlossEvent::Implementation
{
public:
  Implementation(const Pathlosses & pathlosses):
    pathlosses_{pathlosses}{}

  const Pathlosses & getPathlosses() const
  {
    return pathlosses_;
  }

private:
  Pathlosses pathlosses_;
};

EMANE::Events::PathlossEvent::PathlossEvent(const Serialization & serialization):
  Event(IDENTIFIER)
{
  EMANEMessage::PathlossEvent msg;

  if(!msg.ParseFromString(serialization))
    {
      throw SerializationException("unable to deserialize PathlossEvent");
    }

  Pathlosses pathlosses;

  for(const auto & pathloss : msg.pathlosses())
    {
      pathlosses.push_back({static_cast<EMANE::NEMId>(pathloss.nemid()),
            pathloss.forwardpathlossdb(),
            pathloss.reversepathlossdb()});
    }

  pImpl_.reset(new Implementation{pathlosses});
}

EMANE::Events::PathlossEvent::PathlossEvent(const Pathlosses & pathlosses):
  Event{IDENTIFIER},
  pImpl_{new Implementation{pathlosses}}{}

EMANE::Events::PathlossEvent::PathlossEvent(const PathlossEvent & rhs):
  Event{IDENTIFIER},
  pImpl_{new Implementation{rhs.getPathlosses()}}{}

EMANE::Events::PathlossEvent & EMANE::Events::PathlossEvent::operator=(const PathlossEvent & rhs)
{
  pImpl_.reset(new Implementation{rhs.getPathlosses()});
  return *this;
}

EMANE::Events::PathlossEvent::PathlossEvent(PathlossEvent && rval):
  Event{IDENTIFIER},
  pImpl_{new Implementation{{}}}
{
  rval.pImpl_.swap(pImpl_);
}

EMANE::Events::PathlossEvent & EMANE::Events::PathlossEvent::operator=(PathlossEvent && rval)
{
  rval.pImpl_.swap(pImpl_);
  return *this;
}

EMANE::Events::PathlossEvent::~PathlossEvent(){}

const EMANE::Events::Pathlosses & EMANE::Events::PathlossEvent::getPathlosses() const
{
  return pImpl_->getPathlosses();
}

EMANE::Serialization EMANE::Events::PathlossEvent::serialize() const
{
  Serialization serialization;

  EMANEMessage::PathlossEvent msg;

  for(auto & pathloss : pImpl_->getPathlosses())
    {
      auto pPathlossMessage = msg.add_pathlosses();

      pPathlossMessage->set_nemid(pathloss.getNEMId());

      pPathlossMessage->set_forwardpathlossdb(pathloss.getForwardPathlossdB());

      pPathlossMessage->set_reversepathlossdb(pathloss.getReversePathlossdB());
    }

  if(!msg.SerializeToString(&serialization))
    {
      throw SerializationException("unable to serialize PathlossEvent");
    }

  return serialization;
}
