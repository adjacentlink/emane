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

#ifndef EMANEPHYSPECTRUMSERVICE_HEADER_
#define EMANEPHYSPECTRUMSERVICE_HEADER_

#include "emane/types.h"
#include "emane/frequencysegment.h"
#include "emane/spectrumserviceprovider.h"
#include "emane/filtermatchcriterion.h"
#include "spectrummonitor.h"
#include "noisemode.h"

#include <set>
#include <map>
#include <memory>
#include <mutex>

namespace EMANE
{
  class SpectrumService : public SpectrumServiceProvider
  {
  public:
    SpectrumService();

    void initialize(uint16_t u16SubId,
                    NoiseMode mode,
                    const Microseconds & binSize,
                    const Microseconds & maxOffset,
                    const Microseconds & maxPropagation,
                    const Microseconds & maxDuration,
                    const Microseconds & timeSyncThreshold,
                    bool bMaxClamp,
                    bool bExcludeSameSubIdFromFilter);

    SpectrumMonitor * addSpectrumMonitor(AntennaIndex antennaIndex,
                                         const FrequencySet & foi,
                                         std::uint64_t u64BandwidthHz,
                                         double dReceiverSensitivityMilliWatt);

    void resetSpectrumMonitor(AntennaIndex antennaIndex,
                              const FrequencySet & foi,
                              std::uint64_t u64BandwidthHz,
                              double dReceiverSensitivityMilliWatt);

    void removeSpectrumMonitor(AntennaIndex antennaIndex);

    FrequencySet getFrequencies() const override;

    FrequencySet getAntennaFrequencies(AntennaIndex antennaIndex) const override;

    double getReceiverSensitivitydBm() const override;

    double getAntennaReceiverSensitivitydBm(AntennaIndex antennaIndex) const override;

    SpectrumWindow request(std::uint64_t u64FrequencyHz,
                           const Microseconds & duration = Microseconds::zero(),
                           const TimePoint & timepoint = TimePoint::min()) const override;

    SpectrumWindow requestAntenna(AntennaIndex antennaIndex,
                                  std::uint64_t u64FrequencyHz,
                                  const Microseconds & duration = Microseconds::zero(),
                                  const TimePoint & startTime = TimePoint::min()) const override;

    // test harness access
    SpectrumWindow request_i(const TimePoint & now,
                             std::uint64_t u64FrequencyHz,
                             const Microseconds & duration = Microseconds::zero(),
                             const TimePoint & timepoint = TimePoint::min()) const;

    // test harness access
    SpectrumWindow requestAntenna_i(AntennaIndex antennaIndex,
                                    const TimePoint & now,
                                    std::uint64_t u64FrequencyHz,
                                    const Microseconds & duration = Microseconds::zero(),
                                    const TimePoint & timepoint = TimePoint::min()) const;


    void initializeFilter(AntennaIndex antennaIndex,
                          FilterIndex filterIndex,
                          std::uint64_t u64FrequencyHz,
                          std::uint64_t u64BandwidthHz,
                          std::uint64_t u64BandwidthBinSizeHz,
                          const FilterMatchCriterion * pFilterMatchCriterion);

    void removeFilter(AntennaIndex antennaIndex,
                      FilterIndex filterIndex);

    SpectrumFilterWindow requestAntennaFilter(AntennaIndex antennaIndex,
                                              FilterIndex filterIndex,
                                              const Microseconds & duration = Microseconds::zero(),
                                              const TimePoint & timepoint = TimePoint::min()) const override;

    SpectrumFilterWindow requestFilter(FilterIndex filterIndex,
                                       const Microseconds & duration = Microseconds::zero(),
                                       const TimePoint & timepoint = TimePoint::min()) const override;

    //std::set<AntennaIndex> getAntennaIndexes() const;

    bool hasAntenna() const;


    SpectrumUpdate
    update(AntennaIndex antennaIndex,
           const TimePoint & now,
           const TimePoint & txTime,
           const Microseconds & propagationDelay,
           const FrequencySegments & segments,
           std::uint64_t u64SegmentBandwidthHz,
           const std::vector<double> & rxPowersMilliWatt,
           bool bInBand,
           const std::vector<NEMId> & transmitters,
           std::uint16_t u16SubId,
           std::uint64_t u64SpectrumUpdateSequence,
           const std::pair<FilterData,bool> & optionalFilterData);

  private:
    Microseconds binSize_;
    Microseconds maxOffset_;
    Microseconds maxPropagation_;
    Microseconds maxDuration_;
    bool bMaxClamp_;
    bool bExcludeSameSubIdFromFilter_;
    Microseconds timeSyncThreshold_;
    NoiseMode mode_;
    uint16_t u16SubId_;
    mutable std::mutex mutex_;

    using SpectrumMonitorMap =
      std::map<AntennaIndex,std::unique_ptr<SpectrumMonitor>>;

    SpectrumMonitorMap spectrumMonitorMap_;
  };
}

#endif // EMANEPHYSPECTRUMSERVICE_HEADER_
