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

#include "emane/controls/mimoreceivepropertiescontrolmessage.h"

class EMANE::Controls::MIMOReceivePropertiesControlMessage::Implementation
{
public:
  Implementation(const TimePoint & sot,
                 const Microseconds & propagation,
                 const AntennaReceiveInfos & antennaReceiveInfos):
    sot_{sot},
    propagation_{propagation},
    antennaReceiveInfos_{antennaReceiveInfos}{}

  Implementation(const TimePoint & sot,
                 const Microseconds & propagation,
                 AntennaReceiveInfos && antennaReceiveInfos):
    sot_{sot},
    propagation_{propagation},
    antennaReceiveInfos_{std:move(antennaReceiveInfos)}{}

  const TimePoint & getTxTime() const
  {
    return sot_;
  }

  const Microseconds & getPropagationDelay() const
  {
    return propagation_;
  }

  const AntennaReceiveInfos & getAntennaReceiveInfos() const
  {
    return antennaReceiveInfos_;
  }

private:
  const TimePoint sot_;
  const Microseconds propagation_;
  const AntennaReceiveInfos antennaReceiveInfos_;
};

EMANE::Controls::MIMOReceivePropertiesControlMessage::
MIMOReceivePropertiesControlMessage(const MIMOReceivePropertiesControlMessage & msg):
  ControlMessage{IDENTIFIER},
  pImpl_{msg.pImpl_}{}

EMANE::Controls::MIMOReceivePropertiesControlMessage::MIMOReceivePropertiesControlMessage(const TimePoint & sot,
                                                                                          const Microseconds & propagation,
                                                                                          const AntennaReceiveInfos & antennaReceiveInfos):
  ControlMessage{IDENTIFIER},
  pImpl_{new Implementation{sot,propagation,antennaReceiveInfos}}{}

EMANE::Controls::MIMOReceivePropertiesControlMessage::MIMOReceivePropertiesControlMessage(const TimePoint & sot,
                                                                                          const Microseconds & propagation,
                                                                                          AntennaReceiveInfos && antennaReceiveInfos):

  ControlMessage{IDENTIFIER},
  pImpl_{new Implementation{sot,propagation,std::move(antennaReceiveInfos)}}{}

EMANE::Controls::MIMOReceivePropertiesControlMessage::~MIMOReceivePropertiesControlMessage(){}


EMANE::Controls::MIMOReceivePropertiesControlMessage *
EMANE::Controls::MIMOReceivePropertiesControlMessage::create(const TimePoint & sot,
                                                             const Microseconds & propagation,
                                                             const AntennaReceiveInfos & antennaReceiveInfos)
{
  return new MIMOReceivePropertiesControlMessage{sot,propagation,antennaReceiveInfos};
}

EMANE::Controls::MIMOReceivePropertiesControlMessage *
EMANE::Controls::MIMOReceivePropertiesControlMessage::create(const TimePoint & sot,
                                                             const Microseconds & propagation,
                                                             AntennaReceiveInfos && antennaReceiveInfos)
{
  return new MIMOReceivePropertiesControlMessage{sot,propagation,std::move(antennaReceiveInfos)};
}

const EMANE::Controls::AntennaReceiveInfos &
EMANE::Controls::MIMOReceivePropertiesControlMessage::getAntennaReceiveInfos() const
{
  return pImpl_->getAntennaReceiveInfos();
}

const EMANE::Microseconds &
EMANE::Controls::MIMOReceivePropertiesControlMessage::getPropagationDelay() const
{
  return pImpl_->getPropagationDelay();
}

const EMANE::TimePoint & EMANE::Controls::MIMOReceivePropertiesControlMessage::getTxTime() const
{
  return pImpl_->getTxTime();
}

EMANE::Controls::MIMOReceivePropertiesControlMessage *
EMANE::Controls::MIMOReceivePropertiesControlMessage::clone() const
{
  return new MIMOReceivePropertiesControlMessage{*this};
}
