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

inline
EMANE::Controls::AntennaReceiveInfo::AntennaReceiveInfo(AntennaIndex rxAntennaIndex,
                                                        AntennaIndex txAntennaIndex,
                                                        const FrequencySegments & frequencySegments,
                                                        Microseconds span,
                                                        double dReceiverSensitivitydBm):
  rxAntennaIndex_{rxAntennaIndex},
  txAntennaIndex_{txAntennaIndex},
  frequencySegments_{frequencySegments},
  span_{span},
  dReceiverSensitivitydBm_{dReceiverSensitivitydBm}{}

inline
EMANE::Controls::AntennaReceiveInfo::AntennaReceiveInfo(AntennaIndex rxAntennaIndex,
                                                        AntennaIndex txAntennaIndex,
                                                        FrequencySegments && frequencySegments,
                                                        Microseconds span,
                                                        double dReceiverSensitivitydBm):
  rxAntennaIndex_{rxAntennaIndex},
  txAntennaIndex_{txAntennaIndex},
  frequencySegments_{std::move(frequencySegments)},
  span_{span},
  dReceiverSensitivitydBm_{dReceiverSensitivitydBm}{}

inline
EMANE::Controls::AntennaReceiveInfo::~AntennaReceiveInfo(){}

inline
EMANE::AntennaIndex EMANE::Controls::AntennaReceiveInfo::getRxAntennaIndex() const
{
  return rxAntennaIndex_;
}

inline
EMANE::AntennaIndex EMANE::Controls::AntennaReceiveInfo::getTxAntennaIndex() const
{
  return txAntennaIndex_;
}

inline
const EMANE::FrequencySegments &
EMANE::Controls::AntennaReceiveInfo::getFrequencySegments() const
{
  return frequencySegments_;
}

inline
double EMANE::Controls::AntennaReceiveInfo::getReceiverSensitivitydBm() const
{
  return dReceiverSensitivitydBm_;
}

inline
EMANE::Microseconds EMANE::Controls::AntennaReceiveInfo::getSpan() const
{
  return span_;
}

inline
EMANE::Controls::AntennaReceiveInfo::AntennaReceiveInfo(const AntennaReceiveInfo & rval):
  rxAntennaIndex_{rval.rxAntennaIndex_},
  txAntennaIndex_{rval.txAntennaIndex_},
  frequencySegments_{rval.frequencySegments_},
  span_{rval.span_},
  dReceiverSensitivitydBm_{rval.dReceiverSensitivitydBm_}{}

inline
EMANE::Controls::AntennaReceiveInfo &
EMANE::Controls::AntennaReceiveInfo::operator=(const AntennaReceiveInfo & rval)
{
  rxAntennaIndex_ = rval.rxAntennaIndex_;
  txAntennaIndex_ = rval.txAntennaIndex_;
  frequencySegments_ = rval.frequencySegments_;
  span_ = rval.span_;
  dReceiverSensitivitydBm_ = rval.dReceiverSensitivitydBm_;
  return *this;
}

inline
EMANE::Controls::AntennaReceiveInfo::AntennaReceiveInfo(AntennaReceiveInfo && rval):
  rxAntennaIndex_{rval.rxAntennaIndex_},
  txAntennaIndex_{rval.txAntennaIndex_},
  frequencySegments_{std::move(rval.frequencySegments_)},
  span_{std::move(rval.span_)},
  dReceiverSensitivitydBm_{rval.dReceiverSensitivitydBm_}{}

inline
EMANE::Controls::AntennaReceiveInfo &
EMANE::Controls::AntennaReceiveInfo::operator=(AntennaReceiveInfo && rval)
{
  rxAntennaIndex_ = rval.rxAntennaIndex_;
  txAntennaIndex_ = rval.txAntennaIndex_;
  frequencySegments_ = std::move(rval.frequencySegments_);
  span_ = std::move(rval.span_);
  dReceiverSensitivitydBm_ = rval.dReceiverSensitivitydBm_;
  return *this;
}
