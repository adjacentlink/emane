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
EMANE::Controls::R2RINeighborMetric::R2RINeighborMetric(NEMId id,
                                                        std::uint32_t u32NumRxFrames,
                                                        std::uint32_t u32NumTxFrames,
                                                        std::uint32_t u32NumMissedFrames,
                                                        const EMANE::Microseconds & bandwidthConsumption,
                                                        float fSINRAvgdBm,
                                                        float fSINRStddev,
                                                        float fNoiseFloorAvgdBm,
                                                        float fNoiseFloorStddev,
                                                        std::uint64_t u64RxAvgDataRatebps,
                                                        std::uint64_t u64TxAvgDataRatebps):

id_{id},
  u32NumRxFrames_{u32NumRxFrames},
  u32NumTxFrames_{u32NumTxFrames},
  u32NumMissedFrames_{u32NumMissedFrames},
  bandwidthConsumption_{bandwidthConsumption},
  fSINRAvgdBm_{fSINRAvgdBm},
  fSINRStddev_{fSINRStddev},
  fNoiseFloorAvgdBm_{fNoiseFloorAvgdBm},
  fNoiseFloorStddev_{fNoiseFloorStddev},
  u64RxAvgDataRatebps_{u64RxAvgDataRatebps},
  u64TxAvgDataRatebps_{u64TxAvgDataRatebps}{}

inline
EMANE::Controls::R2RINeighborMetric::~R2RINeighborMetric(){}

inline
EMANE::NEMId EMANE::Controls::R2RINeighborMetric::getId() const
{
  return id_;
}

inline
std::uint32_t EMANE::Controls::R2RINeighborMetric::getNumRxFrames() const
{
  return u32NumRxFrames_;
}

inline   
std::uint32_t EMANE::Controls::R2RINeighborMetric::getNumTxFrames() const
{
  return u32NumTxFrames_;
}

inline   
std::uint32_t EMANE::Controls::R2RINeighborMetric::getNumMissedFrames() const
{
  return u32NumMissedFrames_;
}

inline
const EMANE::Microseconds & EMANE::Controls::R2RINeighborMetric::getBandwidthConsumption() const
{
  return bandwidthConsumption_;
}

inline
float EMANE::Controls::R2RINeighborMetric::getSINRAvgdBm() const
{
  return fSINRAvgdBm_;
}

inline
float EMANE::Controls::R2RINeighborMetric::getSINRStddev() const
{
  return fNoiseFloorStddev_;
}

inline   
float EMANE::Controls::R2RINeighborMetric::getNoiseFloorAvgdBm() const
{
  return fNoiseFloorAvgdBm_;
}

inline
float EMANE::Controls::R2RINeighborMetric::getNoiseFloorStddev() const
{
  return fNoiseFloorStddev_;
}

inline    
std::uint64_t EMANE::Controls::R2RINeighborMetric::getRxAvgDataRatebps() const
{
  return u64RxAvgDataRatebps_;
}

inline   
std::uint64_t EMANE::Controls::R2RINeighborMetric::getTxAvgDataRatebps() const
{
  return u64TxAvgDataRatebps_;
}
