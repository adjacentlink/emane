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

#ifndef EMANEPHYSPECTRUMMONITOR_HEADER_
#define EMANEPHYSPECTRUMMONITOR_HEADER_

#include "emane/types.h"
#include "emane/frequencysegment.h"
#include "emane/spectrumserviceprovider.h"
#include "emane/models/frameworkphy/noiserecorder.h"

#include <set>
#include <map>
#include <vector>
#include <memory>
#include <tuple>
#include <mutex>

namespace EMANE
{
  class SpectrumMonitor : public SpectrumServiceProvider
  {
  public:
    SpectrumMonitor();


    enum class NoiseMode {NONE, ALL, OUTOFBAND};

    void initialize(const FrequencySet & foi,
                    std::uint64_t u64BandwidthHz,
                    double dReceiverSensitivityMilliWatt,
                    NoiseMode mode,
                    const Microseconds & binSize,
                    const Microseconds & maxOffset,
                    const Microseconds & maxPropagation,
                    const Microseconds & maxDuration,
                    const Microseconds & timeSyncThreshold,
                    bool bMaxClamp);

    std::tuple<TimePoint,Microseconds,Microseconds,FrequencySegments,bool>
    update(const TimePoint & now,
           const TimePoint & txTime,
           const Microseconds & propagationDelay,
           const FrequencySegments & segments,
           std::uint64_t u64SegmentBandwidthHz,
           const std::vector<double> & rxPowersMilliWatt,
           bool bInBand,
           const std::vector<NEMId> & transmitters);


    FrequencySet getFrequencies() const override;

    double getReceiverSensitivitydBm() const override;

    // test harness access
    SpectrumWindow request_i(const TimePoint & now,
                             std::uint64_t u64FrequencyHz,
                             const Microseconds & duration = Microseconds::zero(),
                             const TimePoint & timepoint = TimePoint::min()) const override;


    SpectrumWindow request(std::uint64_t u64FrequencyHz,
                           const Microseconds & duration = Microseconds::zero(),
                           const TimePoint & timepoint = TimePoint::min()) const override;


    std::vector<double> dump(std::uint64_t u64FrequencyHz) const;

  private:
    using NoiseRecorderMap = std::map<std::uint64_t,std::unique_ptr<NoiseRecorder>>;
    using Cache = std::map<std::uint64_t,std::vector<std::tuple<NoiseRecorder *,double,std::uint64_t>>>;
    using TransmitterBandwidthCache = std::map<std::uint64_t,std::unique_ptr<Cache>>;
    Microseconds binSize_;
    Microseconds maxOffset_;
    Microseconds maxPropagation_;
    Microseconds maxDuration_;
    bool bMaxClamp_;
    Microseconds timeSyncThreshold_;
    TransmitterBandwidthCache transmitterBandwidthCache_;
    NoiseRecorderMap noiseRecorderMap_;
    std::uint64_t u64ReceiverBandwidthHz_;
    NoiseMode mode_;
    double dReceiverSensitivityMilliWatt_;
    mutable std::mutex mutex_;
  };
}

#endif // EMANEPHYSPECTRUMMONITOR_HEADER_
