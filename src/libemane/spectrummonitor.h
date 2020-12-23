/*
 * Copyright (c) 2013-2014,2019-2020 - Adjacent Link LLC, Bridgewater,
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

#ifndef EMANEPHYSPECTRUMMONITOR_HEADER_
#define EMANEPHYSPECTRUMMONITOR_HEADER_

#include "emane/types.h"
#include "emane/frequencysegment.h"
#include "emane/spectrumserviceprovider.h"
#include "emane/filtermatchcriterion.h"
#include "noiserecorder.h"
#include "noisemode.h"

#include <map>
#include <vector>
#include <memory>
#include <tuple>
#include <mutex>

namespace EMANE
{
  using SpectrumUpdate =
    std::tuple<TimePoint,Microseconds,Microseconds,FrequencySegments,bool,double>;

  class SpectrumMonitor
  {
  public:
    SpectrumMonitor();

    void initialize(uint16_t u16SubId,
                    const FrequencySet & foi,
                    std::uint64_t u64BandwidthHz,
                    double dReceiverSensitivityMilliWatt,
                    NoiseMode mode,
                    const Microseconds & binSize,
                    const Microseconds & maxOffset,
                    const Microseconds & maxPropagation,
                    const Microseconds & maxDuration,
                    const Microseconds & timeSyncThreshold,
                    bool bMaxClamp,
                    bool bExcludeSameSubIdFromFilter);

    SpectrumUpdate
    update(const TimePoint & now,
           const TimePoint & txTime,
           const Microseconds & propagationDelay,
           const FrequencySegments & segments,
           std::uint64_t u64SegmentBandwidthHz,
           const std::vector<double> & rxPowersMilliWatt,
           bool bInBand,
           const std::vector<NEMId> & transmitters,
           std::uint16_t u16SubId,
           AntennaIndex txAntennaIndex,
           const std::pair<FilterData,bool> & optionalFilterData);


    FrequencySet getFrequencies() const;

    double getReceiverSensitivitydBm() const;

    // test harness access
    SpectrumWindow request_i(const TimePoint & now,
                             std::uint64_t u64FrequencyHz,
                             const Microseconds & duration = Microseconds::zero(),
                             const TimePoint & timepoint = TimePoint::min()) const;


    SpectrumWindow request(std::uint64_t u64FrequencyHz,
                           const Microseconds & duration = Microseconds::zero(),
                           const TimePoint & timepoint = TimePoint::min()) const;

    void initializeFilter(FilterIndex filterIndex,
                          std::uint64_t u64FrequencyHz,
                          std::uint64_t u64BandwidthHz,
                          std::uint64_t u64BandwidthBinSizeHz,
                          const FilterMatchCriterion * pFilterMatchCriterion);

    void removeFilter(FilterIndex filterIndex);


    SpectrumFilterWindow requestFilter(FilterIndex filterIndex,
                                       const Microseconds & duration = Microseconds::zero(),
                                       const TimePoint & timepoint = TimePoint::min()) const;

    std::vector<double> dump(std::uint64_t u64FrequencyHz) const;

  private:
    using NoiseRecorderMap = std::map<std::uint64_t,std::unique_ptr<NoiseRecorder>>;

    using NoiseRecord = std::tuple<NoiseRecorder *,
                                   double,
                                   std::uint64_t,
                                   std::uint64_t, // start rx freq
                                   std::uint64_t>; // end rx freq>;

    using NoiseRecords = std::vector<NoiseRecord>;

    using Cache = std::map<std::uint64_t, // frequency Hz
                           NoiseRecords>;

    using TransmitterBandwidthCache = std::map<std::uint64_t, // bandwidth Hz
                                               std::pair<std::unique_ptr<Cache>,
                                                         FrequencySet>>; // no overlap set
    Microseconds binSize_;
    Microseconds maxOffset_;
    Microseconds maxPropagation_;
    Microseconds maxDuration_;
    bool bMaxClamp_;
    bool bExcludeSameSubIdFromFilter_;
    Microseconds timeSyncThreshold_;
    TransmitterBandwidthCache transmitterBandwidthCache_;
    NoiseRecorderMap noiseRecorderMap_;
    std::uint64_t u64ReceiverBandwidthHz_;
    NoiseMode mode_;
    double dReceiverSensitivityMilliWatt_;
    uint16_t u16SubId_;
    mutable std::mutex mutex_;
    FrequencySet foi_;

    using FilterRecord = std::tuple<NoiseRecorder *, // noise recorder
                                    double, // overlap
                                    std::uint64_t, // tx freq
                                    const FilterMatchCriterion *, // match
                                    std::uint64_t, // start rx freq
                                    std::uint64_t>; // end rx freq

    using FilterRecords = std::vector<FilterRecord>;

    using FilterCache = std::map<std::uint64_t, // frequency Hz
                                 FilterRecords>;

    using FitlerTransmitterBandwidthCache = std::map<std::uint64_t, // bandwidth Hz
                                                     std::pair<std::unique_ptr<FilterCache>,
                                                               FrequencySet>>; // no overlap set


    using FilterNoiseRecorderMap = std::map<std::uint16_t, // filter index
                                            std::tuple<std::uint64_t, // frequency Hz
                                                       std::uint64_t, // bandwidth Hz
                                                       std::unique_ptr<NoiseRecorder>,
                                                       std::unique_ptr<const FilterMatchCriterion>>>;

    FilterNoiseRecorderMap filterNoiseRecorderMap_;
    FitlerTransmitterBandwidthCache filterTransmitterBandwidthCache_;

    void applyEnergyToFilters_i(std::uint64_t u64TxBandwidthHz,
                                std::uint64_t u64TxFrequencyHz,
                                std::uint16_t u16SubId,
                                const TimePoint & now,
                                const TimePoint & txTime,
                                const Microseconds & offset,
                                const Microseconds & propagation,
                                const Microseconds & duration,
                                double dRxPowerMilliWatt,
                                const std::vector<NEMId> & transmitters,
                                AntennaIndex txAntennaIndex,
                                const std::pair<FilterData,bool> & optionalFilterData);
  };
}

#endif // EMANEPHYSPECTRUMMONITOR_HEADER_
