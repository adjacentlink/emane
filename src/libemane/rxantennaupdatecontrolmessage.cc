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

#include "emane/controls/rxantennaupdatecontrolmessage.h"

class EMANE::Controls::RxAntennaUpdateControlMessage::Implementation
{
public:
  Implementation(const Antenna & antenna):
    antenna_{antenna}
  {}

  const Antenna & getAntenna() const
  {
    return antenna_;
  }

private:
  const Antenna antenna_;
};

EMANE::Controls::RxAntennaUpdateControlMessage::
RxAntennaUpdateControlMessage(const RxAntennaUpdateControlMessage & msg):
  ControlMessage{IDENTIFIER},
  pImpl_{msg.pImpl_}
{}

EMANE::Controls::RxAntennaUpdateControlMessage::RxAntennaUpdateControlMessage(const Antenna & antenna):
  ControlMessage{IDENTIFIER},
  pImpl_{new Implementation{antenna}}
{}

EMANE::Controls::RxAntennaUpdateControlMessage::~RxAntennaUpdateControlMessage(){}

const EMANE::Antenna & EMANE::Controls::RxAntennaUpdateControlMessage::getAntenna() const
{
  return pImpl_->getAntenna();
}


EMANE::Controls::RxAntennaUpdateControlMessage *
EMANE::Controls::RxAntennaUpdateControlMessage::create(const Antenna & antenna)
{
  return new RxAntennaUpdateControlMessage{antenna};
}


EMANE::Controls::RxAntennaUpdateControlMessage *
EMANE::Controls::RxAntennaUpdateControlMessage::clone() const
{
  return new RxAntennaUpdateControlMessage{*this};
}
