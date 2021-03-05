/*
 * Copyright (c) 2020-2021 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "emane/controls/mimoreceivepropertiescontrolmessageformatter.h"

EMANE::Controls::MIMOReceivePropertiesControlMessageFormatter::
MIMOReceivePropertiesControlMessageFormatter(const MIMOReceivePropertiesControlMessage * pMsg):
  pMsg_{pMsg}{}

EMANE::Strings EMANE::Controls::MIMOReceivePropertiesControlMessageFormatter::operator()() const
{
  Strings strings{};

  strings.push_back("sot: " + std::to_string(std::chrono::duration_cast<DoubleSeconds>(pMsg_->getTxTime().time_since_epoch()).count()));
  strings.push_back("prop delay: " + std::to_string(pMsg_->getPropagationDelay().count()));

  for(const auto & antennaReceiveInfo :  pMsg_->getAntennaReceiveInfos())
    {
      strings.push_back("rx antenna index: " + std::to_string(antennaReceiveInfo.getRxAntennaIndex()));
      strings.push_back("tx antenna index: " + std::to_string(antennaReceiveInfo.getTxAntennaIndex()));
      strings.push_back("span: " + std::to_string(antennaReceiveInfo.getSpan().count()));
      strings.push_back("rx sensitivity: " + std::to_string(antennaReceiveInfo.getReceiverSensitivitydBm()));

      for(const auto & segment : antennaReceiveInfo.getFrequencySegments())
        {
          strings.push_back("freq: " + std::to_string(segment.getFrequencyHz()));
          strings.push_back("duration: " + std::to_string(segment.getDuration().count()));
          strings.push_back("offset: " + std::to_string(segment.getOffset().count()));
          strings.push_back("power: " + std::to_string(segment.getRxPowerdBm()));
        }
    }

  for(const auto & shift : pMsg_->getDopplerShifts())
    {
      strings.push_back("shift freq: " + std::to_string(shift.first));
      strings.push_back("shift hz: " + std::to_string(shift.second));
    }

  return strings;
}
