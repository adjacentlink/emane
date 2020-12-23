/*
 * Copyright (c) 2013-2014,2020 - Adjacent Link LLC, Bridgewater,
 * New Jersey
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

#ifndef EMANENOISERECORDER_HEADER_
#define EMANENOISERECORDER_HEADER_

#include <chrono>
#include <cstring>

#include "wheel.h"
#include "emane/types.h"

#include <map>

namespace EMANE
{
  class NoiseRecorder
  {
  public:
    NoiseRecorder(const Microseconds & bin,
                  const Microseconds & maxOffset,
                  const Microseconds & maxPropagation,
                  const Microseconds & maxDuration,
                  double dRxSensitivityMilliWatt,
                  std::uint64_t u64FrequencyHz,
                  std::uint64_t u64BandwidthHz,
                  std::uint64_t u64BandwidthBinSizeHz);

    /**
     * Update the noise recorder with new signal information
     *
     * @pre offset must be <= than MAX_OFFSET
     *      propagation must be <= MAX_PROPAGATION
     *      duration must be <= MAX_DURATION
     */
    std::tuple<TimePoint,TimePoint>
    update(const TimePoint & now,
           const TimePoint & txTime,
           const Microseconds & offset,
           const Microseconds & propagation,
           const Microseconds & duration,
           double dRxPower,
           const std::vector<NEMId> & transmitters,
           std::uint64_t u64StartFrequencyHz,
           std::uint64_t u64EndFrequencyHz,
           AntennaIndex txAntennaIndex);

    std::pair<std::vector<double>, TimePoint>
    get(const TimePoint & now,
        const Microseconds & duration = Microseconds::zero(),
        const TimePoint & startTime = TimePoint::min());

    std::size_t getSubBandBinCount() const;

    // dump the entire wheel for test-only-purposes
    std::vector<double> dump() const;

  private:
    const Microseconds::rep totalWindowBins_;
    const Microseconds::rep totalWheelBins_;
    const Microseconds::rep binSizeMicroseconds_;
    std::uint64_t u64BandwidthBinSizeHz_;
    std::uint64_t u64BandStartFrequencyHz_;
    size_t totalSubBandBins_;

    Wheel<double> wheel_;
    double dRxSensitivityMilliWatt_;
    Microseconds::rep maxEndOfReceptionBin_;
    Microseconds::rep minStartOfReceptionBin_;

    using AntennaIndexEORMap = std::map<AntennaIndex,Microseconds::rep>;
    using NEMAntennaIndexEORBinMap = std::map<NEMId,AntennaIndexEORMap>;
    NEMAntennaIndexEORBinMap nemAntennaIndexEORBinMap_;

    Microseconds::rep timepointToBin(const TimePoint & tp, bool bAdjust = false);
  };
}

#endif //EMANENOISERECORDER_HEADER_
