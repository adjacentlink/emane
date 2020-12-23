/*
 * Copyright (c) 2013-2015,2019-2020 - Adjacent Link LLC, Bridgewater,
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

#include "spectrumservice.h"
#include "emane/spectrumserviceexception.h"

EMANE::SpectrumService::SpectrumService():
  binSize_{},
  maxOffset_{},
  maxPropagation_{},
  maxDuration_{},
  bMaxClamp_{},
  bExcludeSameSubIdFromFilter_{},
  timeSyncThreshold_{},
  mode_{NoiseMode::NONE},
  u16SubId_{}{};

void EMANE::SpectrumService::initialize(std::uint16_t u16SubId,
                                        NoiseMode mode,
                                        const Microseconds & binSize,
                                        const Microseconds & maxOffset,
                                        const Microseconds & maxPropagation,
                                        const Microseconds & maxDuration,
                                        const Microseconds & timeSyncThreshold,
                                        bool bMaxClamp,
                                        bool bExcludeSameSubIdFromFilter)
{
  std::lock_guard<std::mutex> m(mutex_);

  u16SubId_ = u16SubId;

  binSize_ = binSize;

  mode_ = mode;

  maxOffset_ = maxOffset;

  maxPropagation_ = maxPropagation;

  maxDuration_ = maxDuration;

  bMaxClamp_ = bMaxClamp;

  bExcludeSameSubIdFromFilter_ = bExcludeSameSubIdFromFilter;

  timeSyncThreshold_ = timeSyncThreshold;
}

EMANE::SpectrumMonitor * EMANE::SpectrumService::addSpectrumMonitor(AntennaIndex antennaIndex,
                                                                    const FrequencySet & foi,
                                                                    std::uint64_t u64BandwidthHz,
                                                                    double dReceiverSensitivityMilliWatt)
{
  std::lock_guard<std::mutex> m(mutex_);

  if(spectrumMonitorMap_.find(antennaIndex) == spectrumMonitorMap_.end())
    {
      std::unique_ptr<SpectrumMonitor> pSpectrumMonitor{new SpectrumMonitor{}};

      pSpectrumMonitor->initialize(u16SubId_,
                                   foi,
                                   u64BandwidthHz,
                                   dReceiverSensitivityMilliWatt,
                                   mode_,
                                   binSize_,
                                   maxOffset_,
                                   maxPropagation_,
                                   maxDuration_,
                                   timeSyncThreshold_,
                                   bMaxClamp_,
                                   bExcludeSameSubIdFromFilter_);

      auto iter = spectrumMonitorMap_.insert(std::make_pair(antennaIndex,
                                                            std::unique_ptr<SpectrumMonitor>(pSpectrumMonitor.release()))).first;

      return iter->second.get();
    }
  else
    {
      throw makeException<SpectrumServiceException>("unable to add existing antenna index: %hu",
                                                    antennaIndex);
    }
}

void EMANE::SpectrumService::resetSpectrumMonitor(AntennaIndex antennaIndex,
                                                  const FrequencySet & foi,
                                                  std::uint64_t u64BandwidthHz,
                                                  double dReceiverSensitivityMilliWatt)
{
  std::lock_guard<std::mutex> m(mutex_);

  auto iter  = spectrumMonitorMap_.find(antennaIndex);

  if(iter != spectrumMonitorMap_.end())
    {
      iter->second->initialize(u16SubId_,
                               foi,
                               u64BandwidthHz,
                               dReceiverSensitivityMilliWatt,
                               mode_,
                               binSize_,
                               maxOffset_,
                               maxPropagation_,
                               maxDuration_,
                               timeSyncThreshold_,
                               bMaxClamp_,
                               bExcludeSameSubIdFromFilter_);
    }
  else
    {
      throw makeException<SpectrumServiceException>("unknown antenna index: %hu",
                                                    antennaIndex);
    }
}


void EMANE::SpectrumService::removeSpectrumMonitor(AntennaIndex antennaIndex)
{
  std::lock_guard<std::mutex> m(mutex_);

  if(!spectrumMonitorMap_.erase(antennaIndex))
    {
      throw makeException<SpectrumServiceException>("unknown antenna index: %hu",
                                                    antennaIndex);
    }
}

EMANE::FrequencySet
EMANE::SpectrumService::getFrequencies() const
{
  return getAntennaFrequencies(DEFAULT_ANTENNA_INDEX);
}

EMANE::FrequencySet
EMANE::SpectrumService::getAntennaFrequencies(AntennaIndex antennaIndex) const
{
  std::lock_guard<std::mutex> m(mutex_);

  const auto iter = spectrumMonitorMap_.find(antennaIndex);

  if(iter != spectrumMonitorMap_.end())
    {
      return iter->second->getFrequencies();
    }

  throw makeException<SpectrumServiceException>("unknown antenna index: %hu",
                                                antennaIndex);
}

double EMANE::SpectrumService::getReceiverSensitivitydBm() const
{
  return getAntennaReceiverSensitivitydBm(DEFAULT_ANTENNA_INDEX);
}

double EMANE::SpectrumService::getAntennaReceiverSensitivitydBm(AntennaIndex antennaIndex) const
{
  std::lock_guard<std::mutex> m(mutex_);

  const auto iter = spectrumMonitorMap_.find(antennaIndex);

  if(iter != spectrumMonitorMap_.end())
    {
      return iter->second->getReceiverSensitivitydBm();
    }

  throw makeException<SpectrumServiceException>("unknown antenna index: %hu",
                                                antennaIndex);
}

EMANE::SpectrumWindow
EMANE::SpectrumService::request(std::uint64_t u64FrequencyHz,
                                const Microseconds & duration,
                                const TimePoint & timepoint) const
{
  return requestAntenna(DEFAULT_ANTENNA_INDEX,
                        u64FrequencyHz,
                        duration,
                        timepoint);

}

EMANE::SpectrumWindow
EMANE::SpectrumService::requestAntenna(AntennaIndex antennaIndex,
                                       std::uint64_t u64FrequencyHz,
                                       const Microseconds & duration,
                                       const TimePoint & timepoint) const
{
  std::lock_guard<std::mutex> m(mutex_);

  const auto iter = spectrumMonitorMap_.find(antennaIndex);

  if(iter != spectrumMonitorMap_.end())
    {
      return iter->second->request(u64FrequencyHz,
                                   duration,
                                   timepoint);
    }

  throw makeException<SpectrumServiceException>("unknown antenna index: %hu",
                                                antennaIndex);
}

// test harness access
EMANE::SpectrumWindow
EMANE::SpectrumService::request_i(const TimePoint & now,
                                  std::uint64_t u64FrequencyHz,
                                  const Microseconds & duration,
                                  const TimePoint & timepoint) const
{
  return requestAntenna_i(DEFAULT_ANTENNA_INDEX,
                          now,
                          u64FrequencyHz,
                          duration,
                          timepoint);
}

// test harness access
EMANE::SpectrumWindow
EMANE::SpectrumService::requestAntenna_i(AntennaIndex antennaIndex,
                                         const TimePoint & now,
                                         std::uint64_t u64FrequencyHz,
                                         const Microseconds & duration,
                                         const TimePoint & timepoint) const
{
  std::lock_guard<std::mutex> m(mutex_);

  const auto iter = spectrumMonitorMap_.find(antennaIndex);

  if(iter != spectrumMonitorMap_.end())
    {
      return iter->second->request_i(now,
                                     u64FrequencyHz,
                                     duration,
                                     timepoint);
    }

  throw makeException<SpectrumServiceException>("unknown antenna index: %hu",
                                                antennaIndex);
}

void EMANE::SpectrumService::initializeFilter(AntennaIndex antennaIndex,
                                              FilterIndex filterIndex,
                                              std::uint64_t u64FrequencyHz,
                                              std::uint64_t u64BandwidthHz,
                                              std::uint64_t u64BandwidthBinSizeHz,
                                              const FilterMatchCriterion * pFilterMatchCriterion)
{
  std::lock_guard<std::mutex> m(mutex_);

  auto iter = spectrumMonitorMap_.find(antennaIndex);

  if(iter != spectrumMonitorMap_.end())
    {
      iter->second->initializeFilter(filterIndex,
                                     u64FrequencyHz,
                                     u64BandwidthHz,
                                     u64BandwidthBinSizeHz,
                                     pFilterMatchCriterion);
    }
  else
    {
      throw makeException<SpectrumServiceException>("initializeFilter with unknown antenna index: %hu",
                                                    antennaIndex);
    }
}

void EMANE::SpectrumService::removeFilter(AntennaIndex antennaIndex,
                                          FilterIndex filterIndex)
{
  std::lock_guard<std::mutex> m(mutex_);

  auto iter = spectrumMonitorMap_.find(antennaIndex);

  if(iter != spectrumMonitorMap_.end())
    {
      iter->second->removeFilter(filterIndex);
    }
  else
    {
      throw makeException<SpectrumServiceException>("unknown antenna index: %hu",
                                                    antennaIndex);
    }
}

EMANE::SpectrumFilterWindow
EMANE::SpectrumService::requestFilter(FilterIndex filterIndex,
                                      const Microseconds & duration,
                                      const TimePoint & timepoint) const
{
  return requestAntennaFilter(DEFAULT_ANTENNA_INDEX,
                              filterIndex,
                              duration,
                              timepoint);
}


EMANE::SpectrumFilterWindow
EMANE::SpectrumService::requestAntennaFilter(AntennaIndex antennaIndex,
                                             FilterIndex filterIndex,
                                             const Microseconds & duration,
                                             const TimePoint & timepoint) const
{
  std::lock_guard<std::mutex> m(mutex_);

  auto iter = spectrumMonitorMap_.find(antennaIndex);

  if(iter != spectrumMonitorMap_.end())
    {
      return iter->second->requestFilter(filterIndex,
                                         duration,
                                         timepoint);
    }


  throw makeException<SpectrumServiceException>("unknown antenna index: %hu",
                                                antennaIndex);
}

bool EMANE::SpectrumService::hasAntenna() const
{
  std::lock_guard<std::mutex> m(mutex_);

  return !spectrumMonitorMap_.empty();
}
