/*
 * Copyright (c) 2017 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "emane/events/fadingselectionevent.h"

#include "fadingselectionevent.pb.h"

class EMANE::Events::FadingSelectionEvent::Implementation
{
public:
  Implementation(const FadingSelections & selections):
    selections_{selections}{}

  const FadingSelections & getFadingSelections() const
  {
    return selections_;
  }

private:
  FadingSelections selections_;
};

EMANE::Events::FadingSelectionEvent::FadingSelectionEvent(const Serialization & serialization):
  Event(IDENTIFIER)
{
  EMANEMessage::FadingSelectionEvent msg;

  if(!msg.ParseFromString(serialization))
    {
      throw SerializationException("unable to deserialize : FadingSelectionEvent");
    }

  FadingSelections selections;

  for(const auto & entry : msg.entries())
    {
      FadingModel fadingModel{};

      switch(entry.model())
        {
        case EMANEMessage::FadingSelectionEvent::TYPE_NONE:
          fadingModel = FadingModel::NONE;
          break;
        case EMANEMessage::FadingSelectionEvent::TYPE_NAKAGAMI:
          fadingModel = FadingModel::NAKAGAMI;
          break;
        default:
          break;
        }

      selections.push_back({static_cast<EMANE::NEMId>(entry.nemid()),fadingModel});
    }

  pImpl_.reset(new Implementation{selections});
}

EMANE::Events::FadingSelectionEvent::FadingSelectionEvent(const FadingSelections & selections):
  Event{IDENTIFIER},
  pImpl_{new Implementation{selections}}{}

EMANE::Events::FadingSelectionEvent::FadingSelectionEvent(const FadingSelectionEvent & rhs):
  Event{IDENTIFIER},
  pImpl_{new Implementation{rhs.getFadingSelections()}}{}

EMANE::Events::FadingSelectionEvent & EMANE::Events::FadingSelectionEvent::operator=(const FadingSelectionEvent & rhs)
{
  pImpl_.reset(new Implementation{rhs.getFadingSelections()});
  return *this;
}

EMANE::Events::FadingSelectionEvent::FadingSelectionEvent(FadingSelectionEvent && rval):
  Event{IDENTIFIER},
  pImpl_{new Implementation{{}}}
{
  rval.pImpl_.swap(pImpl_);
}

EMANE::Events::FadingSelectionEvent & EMANE::Events::FadingSelectionEvent::operator=(FadingSelectionEvent && rval)
{
  rval.pImpl_.swap(pImpl_);
  return *this;
}

EMANE::Events::FadingSelectionEvent::~FadingSelectionEvent(){}

const EMANE::Events::FadingSelections & EMANE::Events::FadingSelectionEvent::getFadingSelections() const
{
  return pImpl_->getFadingSelections();
}

EMANE::Serialization EMANE::Events::FadingSelectionEvent::serialize() const
{
  Serialization serialization;

  EMANEMessage::FadingSelectionEvent msg;

  for(auto & selection : pImpl_->getFadingSelections())
    {
      auto pFadingSelectionMessage = msg.add_entries();

      pFadingSelectionMessage->set_nemid(selection.getNEMId());

      switch(selection.getFadingModel())
        {
        case FadingModel::NONE:
          pFadingSelectionMessage->set_model(EMANEMessage::FadingSelectionEvent::TYPE_NONE);
          break;
        case FadingModel::NAKAGAMI:
          pFadingSelectionMessage->set_model(EMANEMessage::FadingSelectionEvent::TYPE_NAKAGAMI);
          break;
        default:
          throw SerializationException("unable to serialize : FadingSelectionEvent unknown model");
          break;
        }
    }

  if(!msg.SerializeToString(&serialization))
    {
      throw SerializationException("unable to serialize : FadingSelectionEvent");
    }

  return serialization;
}
