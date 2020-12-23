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
EMANE::Controls::AntennaSelfInterference::
AntennaSelfInterference(FrequencyGroupIndex frequencyGroupIndex,
                        double dPowerMilliWatt):
  frequencyGroupIndex_{frequencyGroupIndex},
  powerMilliWatts_{dPowerMilliWatt}{}

inline
EMANE::Controls::AntennaSelfInterference::
AntennaSelfInterference(FrequencyGroupIndex frequencyGroupIndex,
                        const std::vector<double> & powerMilliWatts):
  frequencyGroupIndex_{frequencyGroupIndex},
  powerMilliWatts_{powerMilliWatts}{}

inline
EMANE::Controls::AntennaSelfInterference::
AntennaSelfInterference(FrequencyGroupIndex frequencyGroupIndex,
                        std::vector<double> && powerMilliWatts):
  frequencyGroupIndex_{frequencyGroupIndex},
  powerMilliWatts_{std::move(powerMilliWatts)}{}


inline
EMANE::Controls::AntennaSelfInterference::~AntennaSelfInterference(){}

inline
EMANE::FrequencyGroupIndex EMANE::Controls::AntennaSelfInterference::getFrequencyGroupIndex() const
{
  return frequencyGroupIndex_;
}

inline
const std::vector<double> & EMANE::Controls::AntennaSelfInterference::getPowerMilliWatts() const
{
  return powerMilliWatts_;
}

inline
EMANE::Controls::AntennaSelfInterference::AntennaSelfInterference(const AntennaSelfInterference & rval):
  frequencyGroupIndex_{rval.frequencyGroupIndex_},
  powerMilliWatts_{rval.powerMilliWatts_}{}

inline
EMANE::Controls::AntennaSelfInterference &
EMANE::Controls::AntennaSelfInterference::operator=(const AntennaSelfInterference & rval)
{
  frequencyGroupIndex_ = rval.frequencyGroupIndex_;
  powerMilliWatts_ = rval.powerMilliWatts_;
  return *this;
}

inline
EMANE::Controls::AntennaSelfInterference::AntennaSelfInterference(AntennaSelfInterference && rval):
  frequencyGroupIndex_{rval.frequencyGroupIndex_},
  powerMilliWatts_{std::move(rval.powerMilliWatts_)}{}

inline
EMANE::Controls::AntennaSelfInterference &
EMANE::Controls::AntennaSelfInterference::operator=(AntennaSelfInterference && rval)
{
  frequencyGroupIndex_ = rval.frequencyGroupIndex_;
  powerMilliWatts_ = std::move(rval.powerMilliWatts_);
  return *this;
}
