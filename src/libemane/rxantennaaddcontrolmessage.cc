/*
 * Copyright (c) 2020 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "emane/controls/rxantennaaddcontrolmessage.h"

class EMANE::Controls::RxAntennaAddControlMessage::Implementation
{
public:
  Implementation(const Antenna & antenna,
                 const FrequencySet & frequencyOfInterestSet):
    antenna_{antenna},
    frequencyOfInterestSet_{frequencyOfInterestSet}{}

  Implementation(const Antenna & antenna,
                 FrequencySet && frequencyOfInterestSet):
    antenna_{antenna},
    frequencyOfInterestSet_{std::move(frequencyOfInterestSet)}{}


  const Antenna & getAntenna() const
  {
    return antenna_;
  }

  const FrequencySet & getFrequencyOfInterestSet() const
  {
    return frequencyOfInterestSet_;
  }

  const std::pair<std::uint64_t,bool> & getBandwidthHz() const
  {
    return optionalBandwidthHz_;
  }

private:
  const Antenna antenna_;
  const FrequencySet frequencyOfInterestSet_;
  const  std::pair<std::uint64_t,bool> optionalBandwidthHz_;
};

EMANE::Controls::RxAntennaAddControlMessage::
RxAntennaAddControlMessage(const RxAntennaAddControlMessage & msg):
  ControlMessage{IDENTIFIER},
  pImpl_{msg.pImpl_}
{}

EMANE::Controls::RxAntennaAddControlMessage::RxAntennaAddControlMessage(const Antenna & antenna,
                                                                        const FrequencySet & frequencyOfInterestSet):
  ControlMessage{IDENTIFIER},
  pImpl_{new Implementation{antenna,
                            frequencyOfInterestSet}}
{}

EMANE::Controls::RxAntennaAddControlMessage::RxAntennaAddControlMessage(const Antenna & antenna,
                                                                        FrequencySet && frequencyOfInterestSet):
  ControlMessage{IDENTIFIER},
  pImpl_{new Implementation{antenna,
                            std::move(frequencyOfInterestSet)}}
{}


EMANE::Controls::RxAntennaAddControlMessage::~RxAntennaAddControlMessage(){}

const EMANE::Antenna & EMANE::Controls::RxAntennaAddControlMessage::getAntenna() const
{
  return pImpl_->getAntenna();
}

const EMANE::FrequencySet & EMANE::Controls::RxAntennaAddControlMessage::getFrequencyOfInterestSet() const
{
  return  pImpl_->getFrequencyOfInterestSet();
}

EMANE::Controls::RxAntennaAddControlMessage *
EMANE::Controls::RxAntennaAddControlMessage::create(const Antenna & antenna,
                                                    const FrequencySet & frequencyOfInterestSet)
{
  return new RxAntennaAddControlMessage{antenna,frequencyOfInterestSet};

}

EMANE::Controls::RxAntennaAddControlMessage *
EMANE::Controls::RxAntennaAddControlMessage::create(const Antenna & antenna,
                                                    FrequencySet && frequencyOfInterestSet)
{
  return new RxAntennaAddControlMessage{antenna,std::move(frequencyOfInterestSet)};

}


EMANE::Controls::RxAntennaAddControlMessage *
EMANE::Controls::RxAntennaAddControlMessage::clone() const
{
  return new RxAntennaAddControlMessage{*this};
}
