/*
 * Copyright (c) 2013-2014,2020 - Adjacent Link LLC, Bridgewater,
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

#include "emane/controls/frequencycontrolmessage.h"

class EMANE::Controls::FrequencyControlMessage::Implementation
{
public:
  Implementation(std::uint64_t u64BandwidthHz,
                 const FrequencySegments & frequencySegments):
    u64BandwidthHz_{u64BandwidthHz},
    frequencySegments_{frequencySegments}{}

  std::uint64_t getBandwidthHz() const
  {
    return u64BandwidthHz_;
  }

  const FrequencySegments & getFrequencySegments() const
  {
    return frequencySegments_;
  }

private:
  const std::uint64_t u64BandwidthHz_;
  const FrequencySegments frequencySegments_;
};

EMANE::Controls::FrequencyControlMessage::
FrequencyControlMessage(const FrequencyControlMessage & msg):
  ControlMessage{IDENTIFIER},
  pImpl_{msg.pImpl_}
{}

EMANE::Controls::FrequencyControlMessage::FrequencyControlMessage(std::uint64_t u64BandwidthHz,
                                                                  const FrequencySegments & frequencySegments):
  ControlMessage{IDENTIFIER},
  pImpl_{new Implementation{u64BandwidthHz,frequencySegments}}{}

EMANE::Controls::FrequencyControlMessage::~FrequencyControlMessage(){}


EMANE::Controls::FrequencyControlMessage *
EMANE::Controls::FrequencyControlMessage::create(std::uint64_t u64BandwidthHz,
                                                 const FrequencySegments & frequencySegments)
{
  return new FrequencyControlMessage{u64BandwidthHz,frequencySegments};
}

const EMANE::FrequencySegments &
EMANE::Controls::FrequencyControlMessage::getFrequencySegments() const
{
  return pImpl_->getFrequencySegments();
}

std::uint64_t EMANE::Controls::FrequencyControlMessage::getBandwidthHz() const
{
  return pImpl_->getBandwidthHz();
}

EMANE::Controls::FrequencyControlMessage *
EMANE::Controls::FrequencyControlMessage::clone() const
{
  return new FrequencyControlMessage{*this};
}
