/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
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
EMANE::FrequencySegment::FrequencySegment(std::uint64_t u64FrequencyHz,
                                          double dPowerdBm,
                                          const Microseconds & durationMicroseconds,
                                          const Microseconds & offsetMicroseconds):
  u64FrequencyHz_{u64FrequencyHz},
  durationMicroseconds_{durationMicroseconds},
  offsetMicroseconds_{offsetMicroseconds},
  dPowerdBm_{dPowerdBm},
  bHasPower_{true}{}

inline
EMANE::FrequencySegment::FrequencySegment(std::uint64_t u64FrequencyHz,
                                          const Microseconds & durationMicroseconds,
                                          const Microseconds & offsetMicroseconds):
  u64FrequencyHz_{u64FrequencyHz},
  durationMicroseconds_{durationMicroseconds},
  offsetMicroseconds_{offsetMicroseconds},
  dPowerdBm_{},
  bHasPower_{false}{}

inline
EMANE::FrequencySegment::FrequencySegment(const FrequencySegment & rhs,
                                          double dPowerdBm):
  u64FrequencyHz_{rhs.u64FrequencyHz_},
  durationMicroseconds_{rhs.durationMicroseconds_},
  offsetMicroseconds_{rhs.offsetMicroseconds_},
  dPowerdBm_{dPowerdBm},
  bHasPower_{true}{}

inline
std::uint64_t EMANE::FrequencySegment::getFrequencyHz() const
{
  return u64FrequencyHz_;
}

inline
const EMANE::Microseconds & EMANE::FrequencySegment::getOffset() const
{
  return offsetMicroseconds_;
}

inline
const EMANE::Microseconds & EMANE::FrequencySegment::getDuration() const
{
  return durationMicroseconds_;
}

inline
double EMANE::FrequencySegment::getRxPowerdBm() const
{
  return dPowerdBm_;
}

inline
std::pair<double,bool> EMANE::FrequencySegment::getPowerdBm() const
{
  return {dPowerdBm_,bHasPower_};
}
