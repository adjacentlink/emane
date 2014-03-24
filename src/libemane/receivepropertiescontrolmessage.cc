/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "emane/controls/receivepropertiescontrolmessage.h"

class EMANE::Controls::ReceivePropertiesControlMessage::Implementation
{
public:
  Implementation(const TimePoint & sot,
                 const Microseconds & propagation,
                 const Microseconds & span,
                 double dReceiverSensitivitydBm):
    sot_{sot},
    propagation_{propagation},
    span_{span},
    dReceiverSensitivitydBm_{dReceiverSensitivitydBm}{}

  TimePoint getTxTime() const
  {
    return sot_;
  }

  Microseconds getPropagationDelay() const
  {
    return propagation_;
  }
  
  Microseconds getSpan() const
  {
    return span_;
  }

  double getReceiverSensitivitydBm() const
  {
    return dReceiverSensitivitydBm_;
  }
  
private:
  const TimePoint sot_;
  const Microseconds propagation_;
  const Microseconds span_;
  const double dReceiverSensitivitydBm_;
};

EMANE::Controls::ReceivePropertiesControlMessage::
ReceivePropertiesControlMessage(const ReceivePropertiesControlMessage & msg):
  ControlMessage{IDENTIFIER},
  pImpl_{new Implementation{*msg.pImpl_}}
{}


EMANE::Controls::ReceivePropertiesControlMessage::ReceivePropertiesControlMessage(const TimePoint & sot,
                                                                                  const Microseconds & propagation,
                                                                                  const Microseconds & span,
                                                                                  double dReceiverSensitivitydBm):
  ControlMessage{IDENTIFIER},
  pImpl_{new Implementation{sot,propagation,span,dReceiverSensitivitydBm}}{}

EMANE::Controls::ReceivePropertiesControlMessage::~ReceivePropertiesControlMessage(){}


EMANE::Controls::ReceivePropertiesControlMessage * 
EMANE::Controls::ReceivePropertiesControlMessage::create(const TimePoint & sot,
                                                         const Microseconds & propagation,
                                                         const Microseconds & span,
                                                         double dReceiverSensitivitydBm)
{
  return new ReceivePropertiesControlMessage{sot,propagation,span,dReceiverSensitivitydBm};
}

EMANE::TimePoint EMANE::Controls::ReceivePropertiesControlMessage::getTxTime() const
{
  return pImpl_->getTxTime();
}

double EMANE::Controls::ReceivePropertiesControlMessage::getReceiverSensitivitydBm() const
{
  return pImpl_->getReceiverSensitivitydBm();
}


EMANE::Microseconds EMANE::Controls::ReceivePropertiesControlMessage::getPropagationDelay() const
{
  return pImpl_->getPropagationDelay();
}

EMANE::Microseconds EMANE::Controls::ReceivePropertiesControlMessage::getSpan() const
{
  return pImpl_->getSpan();
}

EMANE::Controls::ReceivePropertiesControlMessage *
EMANE::Controls::ReceivePropertiesControlMessage::clone() const
{
  return new ReceivePropertiesControlMessage{*this};
}
