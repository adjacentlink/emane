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

#include "emane/controls/mimotxwhilerxinterferencecontrolmessage.h"

class EMANE::Controls::MIMOTxWhileRxInterferenceControlMessage::Implementation
{
public:
  Implementation(const FrequencyGroups & frequencyGroups,
                 const RxAntennaInterferenceMap & rxAntennaSelections):
    frequencyGroups_{frequencyGroups},
    rxAntennaSelections_{rxAntennaSelections}{}

  Implementation(FrequencyGroups && frequencyGroups,
                 const RxAntennaInterferenceMap & rxAntennaSelections):
    frequencyGroups_{std::move(frequencyGroups)},
    rxAntennaSelections_{rxAntennaSelections}{}

  Implementation(FrequencyGroups && frequencyGroups,
                 RxAntennaInterferenceMap && rxAntennaSelections):
    frequencyGroups_{std::move(frequencyGroups)},
    rxAntennaSelections_{std::move(rxAntennaSelections)}{}

  const FrequencyGroups &  getFrequencyGroups() const
  {
    return frequencyGroups_;
  }

  const RxAntennaInterferenceMap & getRxAntennaInterferenceMap() const
  {
    return rxAntennaSelections_;
  }

private:
  const FrequencyGroups frequencyGroups_;
  const RxAntennaInterferenceMap rxAntennaSelections_;
};

EMANE::Controls::MIMOTxWhileRxInterferenceControlMessage::
MIMOTxWhileRxInterferenceControlMessage(const MIMOTxWhileRxInterferenceControlMessage & msg):
  ControlMessage{IDENTIFIER},
  pImpl_{msg.pImpl_}
{}


EMANE::Controls::MIMOTxWhileRxInterferenceControlMessage::MIMOTxWhileRxInterferenceControlMessage(const FrequencyGroups & frequencyGroups,
                                                                                                  const RxAntennaInterferenceMap & rxAntennaSelections):

  ControlMessage{IDENTIFIER},
  pImpl_{new Implementation{frequencyGroups,rxAntennaSelections}}{}

EMANE::Controls::MIMOTxWhileRxInterferenceControlMessage::MIMOTxWhileRxInterferenceControlMessage(FrequencyGroups && frequencyGroups,
                                                                                                  const RxAntennaInterferenceMap & rxAntennaSelections):
  ControlMessage{IDENTIFIER},
  pImpl_{new Implementation{std::move(frequencyGroups),rxAntennaSelections}}{}

EMANE::Controls::MIMOTxWhileRxInterferenceControlMessage::MIMOTxWhileRxInterferenceControlMessage(FrequencyGroups && frequencyGroups,
                                                                                                  RxAntennaInterferenceMap && rxAntennaSelections):
  ControlMessage{IDENTIFIER},
  pImpl_{new Implementation{std::move(frequencyGroups),std::move(rxAntennaSelections)}}{}

EMANE::Controls::MIMOTxWhileRxInterferenceControlMessage::~MIMOTxWhileRxInterferenceControlMessage(){}


EMANE::Controls::MIMOTxWhileRxInterferenceControlMessage *
EMANE::Controls::MIMOTxWhileRxInterferenceControlMessage::create(const FrequencyGroups & frequencyGroups,
                                                                 const RxAntennaInterferenceMap & rxAntennaSelections)
{
  return new MIMOTxWhileRxInterferenceControlMessage{frequencyGroups,rxAntennaSelections};
}

EMANE::Controls::MIMOTxWhileRxInterferenceControlMessage *
EMANE::Controls::MIMOTxWhileRxInterferenceControlMessage::create(FrequencyGroups && frequencyGroups,
                                                                 const RxAntennaInterferenceMap & rxAntennaSelections)
{
  return new MIMOTxWhileRxInterferenceControlMessage{std::move(frequencyGroups),rxAntennaSelections};
}

EMANE::Controls::MIMOTxWhileRxInterferenceControlMessage *
EMANE::Controls::MIMOTxWhileRxInterferenceControlMessage::create(FrequencyGroups && frequencyGroups,
                                                                 RxAntennaInterferenceMap && rxAntennaSelections)
{
  return new MIMOTxWhileRxInterferenceControlMessage{std::move(frequencyGroups),
                                                       std::move(rxAntennaSelections)};
}

const EMANE::FrequencyGroups &
EMANE::Controls::MIMOTxWhileRxInterferenceControlMessage::getFrequencyGroups() const
{
  return pImpl_->getFrequencyGroups();
}

const EMANE::Controls::MIMOTxWhileRxInterferenceControlMessage::RxAntennaInterferenceMap &
EMANE::Controls::MIMOTxWhileRxInterferenceControlMessage::getRxAntennaInterferenceMap() const
{
  return pImpl_->getRxAntennaInterferenceMap();
}

EMANE::Controls::MIMOTxWhileRxInterferenceControlMessage *
EMANE::Controls::MIMOTxWhileRxInterferenceControlMessage::clone() const
{
  return new MIMOTxWhileRxInterferenceControlMessage{*this};
}
