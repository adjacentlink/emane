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

#include "emane/controls/mimotransmitpropertiescontrolmessage.h"

class EMANE::Controls::MIMOTransmitPropertiesControlMessage::Implementation
{
public:
  Implementation(const FrequencyGroups & frequencyGroups,
                 const Antennas & transmitAntennas):
    frequencyGroups_{frequencyGroups},
    transmitAntennas_{transmitAntennas}{}

  Implementation(FrequencyGroups && frequencyGroups,
                 const Antennas & transmitAntennas):
    frequencyGroups_{std::move(frequencyGroups)},
    transmitAntennas_{transmitAntennas}{}

  Implementation(FrequencyGroups && frequencyGroups,
                 Antennas && transmitAntennas):
    frequencyGroups_{std::move(frequencyGroups)},
    transmitAntennas_{std::move(transmitAntennas)}{}

  const FrequencyGroups &  getFrequencyGroups() const
  {
    return frequencyGroups_;
  }

  const Antennas & getTransmitAntennas() const
  {
    return transmitAntennas_;
  }

private:
  const FrequencyGroups frequencyGroups_;
  const Antennas transmitAntennas_;
};

EMANE::Controls::MIMOTransmitPropertiesControlMessage::
MIMOTransmitPropertiesControlMessage(const MIMOTransmitPropertiesControlMessage & msg):
  ControlMessage{IDENTIFIER},
  pImpl_{msg.pImpl_}
{}


EMANE::Controls::MIMOTransmitPropertiesControlMessage::MIMOTransmitPropertiesControlMessage(const FrequencyGroups & frequencyGroups,
                                                                                            const Antennas & transmitAntennas):

  ControlMessage{IDENTIFIER},
  pImpl_{new Implementation{frequencyGroups,transmitAntennas}}{}

EMANE::Controls::MIMOTransmitPropertiesControlMessage::MIMOTransmitPropertiesControlMessage(FrequencyGroups && frequencyGroups,
                                                                                            const Antennas & transmitAntennas):
  ControlMessage{IDENTIFIER},
  pImpl_{new Implementation{std::move(frequencyGroups),transmitAntennas}}{}

EMANE::Controls::MIMOTransmitPropertiesControlMessage::MIMOTransmitPropertiesControlMessage(FrequencyGroups && frequencyGroups,
                                                                                            Antennas && transmitAntennas):
  ControlMessage{IDENTIFIER},
  pImpl_{new Implementation{std::move(frequencyGroups),std::move(transmitAntennas)}}{}

EMANE::Controls::MIMOTransmitPropertiesControlMessage::~MIMOTransmitPropertiesControlMessage(){}


EMANE::Controls::MIMOTransmitPropertiesControlMessage *
EMANE::Controls::MIMOTransmitPropertiesControlMessage::create(const FrequencyGroups & frequencyGroups,
                                                              const Antennas & transmitAntennas)
{
  return new MIMOTransmitPropertiesControlMessage{frequencyGroups,transmitAntennas};
}

EMANE::Controls::MIMOTransmitPropertiesControlMessage *
EMANE::Controls::MIMOTransmitPropertiesControlMessage::create(FrequencyGroups && frequencyGroups,
                                                              const Antennas & transmitAntennas)
{
  return new MIMOTransmitPropertiesControlMessage{std::move(frequencyGroups),transmitAntennas};
}

EMANE::Controls::MIMOTransmitPropertiesControlMessage *
EMANE::Controls::MIMOTransmitPropertiesControlMessage::create(FrequencyGroups && frequencyGroups,
                                                              Antennas && transmitAntennas)
{
  return new MIMOTransmitPropertiesControlMessage{std::move(frequencyGroups),
                                                    std::move(transmitAntennas)};
}

const EMANE::FrequencyGroups &
EMANE::Controls::MIMOTransmitPropertiesControlMessage::getFrequencyGroups() const
{
  return pImpl_->getFrequencyGroups();
}

const EMANE::Antennas &
EMANE::Controls::MIMOTransmitPropertiesControlMessage::getTransmitAntennas() const
{
  return pImpl_->getTransmitAntennas();
}

EMANE::Controls::MIMOTransmitPropertiesControlMessage *
EMANE::Controls::MIMOTransmitPropertiesControlMessage::clone() const
{
  return new MIMOTransmitPropertiesControlMessage{*this};
}
