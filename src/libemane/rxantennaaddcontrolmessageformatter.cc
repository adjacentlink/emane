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

#include "emane/controls/rxantennaaddcontrolmessageformatter.h"

EMANE::Controls::RxAntennaAddControlMessageFormatter::
RxAntennaAddControlMessageFormatter(const RxAntennaAddControlMessage * pMsg):
  pMsg_{pMsg}{}

EMANE::Strings EMANE::Controls::RxAntennaAddControlMessageFormatter::operator()() const
{
  Strings strings{};

  const auto & antenna =  pMsg_->getAntenna();

  strings.push_back("anntena id: " + std::to_string(antenna.getIndex()));

  auto fixedGaindBi = antenna.getFixedGaindBi();

  strings.push_back("fixed gain: " +
                    std::to_string(fixedGaindBi.first) +
                    "/" +
                    std::to_string(fixedGaindBi.second));

  auto pointing = antenna.getPointing();

  strings.push_back("az: " + std::to_string(pointing.first.getAzimuthDegrees()));

  strings.push_back("el: " + std::to_string(pointing.first.getElevationDegrees()) +
                    "/" +
                    std::to_string(pointing.second));

  strings.push_back("profile: " + std::to_string(pointing.first.getProfileId()));

  for(const auto & frequency : pMsg_->getFrequencyOfInterestSet())
    {
      strings.push_back("freq: " + std::to_string(frequency));
    }

  strings.push_back("bandwidth: " +
                    std::to_string(antenna.getBandwidthHz()));

  return strings;
}
