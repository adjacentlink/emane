/*
 * Copyright (c) 2013-2015,2019-2021 - Adjacent Link LLC, Bridgewater,
 *  New Jersey
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

#include "spectrummonitor.h"
#include "noiserecorder.h"
#include "spectralmaskmanager.h"

#include "emane/spectrumserviceexception.h"
#include "emane/utils/conversionutils.h"
#include "emane/utils/dopplerutils.h"

#include <algorithm>
#include <functional>

EMANE::SpectrumMonitor::SpectrumMonitor():
  binSize_{},
  maxOffset_{},
  maxPropagation_{},
  maxDuration_{},
  bMaxClamp_{},
  bExcludeSameSubIdFromFilter_{},
  timeSyncThreshold_{},
  u64ReceiverBandwidthHz_{},
  mode_{NoiseMode::NONE},
  dReceiverSensitivityMilliWatt_{},
  u16SubId_{}{}

void EMANE::SpectrumMonitor::initialize(std::uint16_t u16SubId,
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

  dReceiverSensitivityMilliWatt_ = dReceiverSensitivityMilliWatt;

  u64ReceiverBandwidthHz_ = u64BandwidthHz;

  foi_ = foi;

  // (re-)initialize transmitter bandwidth cache
  transmitterBandwidthCache_.clear();

  transmitterBandwidthCache_.emplace(u64BandwidthHz,std::make_pair(std::unique_ptr<Cache>(new Cache{}),FrequencySet{}));

  noiseRecorderMap_.clear();

  for(const auto & frequency : foi)
    {
      noiseRecorderMap_.emplace(frequency,
                                std::unique_ptr<NoiseRecorder>{new NoiseRecorder{binSize,
                                      maxOffset,
                                      maxPropagation,
                                      maxDuration,
                                      dReceiverSensitivityMilliWatt,
                                      frequency,
                                      u64BandwidthHz,
                                      0}});
    }

}

EMANE::SpectrumUpdate
EMANE::SpectrumMonitor::update(const TimePoint & now,
                               const TimePoint & txTime,
                               const Microseconds & propagationDelay,
                               double dDopplerFactor,
                               const FrequencySegments & segments,
                               std::uint64_t u64SegmentBandwidthHz,
                               const std::vector<double> & rxPowersMilliWatt,
                               bool bInBand,
                               const std::vector<NEMId> & transmitters,
                               std::uint16_t u16SubId,
                               AntennaIndex txAntennaIndex,
                               SpectralMaskIndex spectralMaskIndex,
                               const std::pair<FilterData,bool> & optionalFilterData)
{
  std::lock_guard<std::mutex> m(mutex_);

  if(segments.size() != rxPowersMilliWatt.size())
    {
      return std::make_tuple(TimePoint{},Microseconds{},Microseconds{},FrequencySegments{},false,0);
    }

  bool bReportAsInBand{};

  FrequencySegments reportableFrequencySegments;

  // validate txTime in case of time sync issues
  auto validTxTime = txTime;

  if(txTime > now + timeSyncThreshold_ || now - txTime > timeSyncThreshold_)
    {
      validTxTime = now;
    }

  auto validPropagation = propagationDelay;

  if(propagationDelay > maxPropagation_)
    {
      if(bMaxClamp_)
        {
          validPropagation = maxPropagation_;
        }
      else
        {
          throw makeException<SpectrumServiceException>("Message propagation %ju usec > max propagation %ju usec and max clamp is %s",
                                                        propagationDelay.count(),
                                                        maxPropagation_.count(),
                                                        bMaxClamp_ ? "on" : "off");
        }
    }

  TimePoint maxEoR{};
  TimePoint minSoR{TimePoint::max()};

  // if noise processing is disabled just check in-band updates for
  // frequencies of interest
  if(mode_ == NoiseMode::NONE ||
     mode_ == NoiseMode::PASSTHROUGH ||
     (mode_ == NoiseMode::OUTOFBAND && bInBand))
    {
      size_t i{};

      bReportAsInBand = true;

      for(const auto & segment : segments)
        {
          // in passthrough mode, you do not need an exact frequency
          // match for processing like you would for inband messages
          if(mode_ == NoiseMode::PASSTHROUGH ||
             noiseRecorderMap_.find(segment.getFrequencyHz()) != noiseRecorderMap_.end())
            {
              std::uint64_t u64DopplerShiftedFequencyHz = segment.getFrequencyHz() +
                Utils::dopplerShift(segment.getFrequencyHz(),dDopplerFactor);

              auto maskOverlap =
                SpectralMaskManager::instance()->getSpectralOverlap(u64DopplerShiftedFequencyHz, // tx freq
                                                                    segment.getFrequencyHz(), // rx freq
                                                                    u64ReceiverBandwidthHz_, // rx bandwidth
                                                                    u64SegmentBandwidthHz,
                                                                    spectralMaskIndex);

              auto & spectralOverlaps = std::get<0>(maskOverlap);
              double dOverlapRxPowerMillWatt{};

              if(!spectralOverlaps.empty())
                {
                  for(const auto & spectralOverlap : spectralOverlaps)
                    {
                      const auto & spectralSegments =  std::get<0>(spectralOverlap);

                      for(const auto & spectralSegment : spectralSegments)
                        {
                          dOverlapRxPowerMillWatt +=
                            // rx_power_mW * modifier_mWr * overlap_ratio
                            rxPowersMilliWatt[i] *
                            std::get<1>(spectralSegment) *
                            std::get<0>(spectralSegment);
                        }
                    }
                }

              if(dOverlapRxPowerMillWatt >= dReceiverSensitivityMilliWatt_)
                {
                  auto validOffset = segment.getOffset();

                  if(validOffset > maxOffset_)
                    {
                      if(bMaxClamp_)
                        {
                          validOffset = maxOffset_;
                        }
                      else
                        {
                          throw makeException<SpectrumServiceException>("Segment offset %ju usec > max offset %ju usec and max clamp is %s",
                                                                        validOffset.count(),
                                                                        maxOffset_.count(),
                                                                        bMaxClamp_ ? "on" : "off");
                        }
                    }

                  auto validDuration = segment.getDuration();

                  if(validDuration > maxDuration_)
                    {
                      if(bMaxClamp_)
                        {
                          validDuration = maxDuration_;
                        }
                      else
                        {
                          throw makeException<SpectrumServiceException>("Segment duration %ju usec > max duration %ju usec and max clamp is %s",
                                                                        validDuration.count(),
                                                                        maxDuration_.count(),
                                                                        bMaxClamp_ ? "on" : "off");
                        }
                    }

                  maxEoR =
                    std::max(maxEoR,validTxTime + validOffset + validDuration + validPropagation);

                  minSoR =
                    std::min(minSoR,validTxTime + validOffset + validPropagation);


                  reportableFrequencySegments.emplace_back(segment.getFrequencyHz(),
                                                           Utils::MILLIWATT_TO_DB(dOverlapRxPowerMillWatt),
                                                           validDuration,
                                                           validOffset);
                }

              ++i;
            }
          else
            {
              bReportAsInBand = false;
            }
        }
    }
  else
    {
      TransmitterBandwidthCache * pTransmitterBandwidthCache{};
      std::uint64_t u64TransmitterBandwidthCacheKey{};

      if(spectralMaskIndex)
        {
          pTransmitterBandwidthCache = &transmitterSpectralMaskCache_;
          u64TransmitterBandwidthCacheKey = spectralMaskIndex;
        }
      else
        {
          pTransmitterBandwidthCache = &transmitterBandwidthCache_;
          u64TransmitterBandwidthCacheKey = u64SegmentBandwidthHz;
        }

      Cache * pCache{};

      // find the cache for the transmitter bandwidth
      auto transmitterBandwidthCacheIter =
        pTransmitterBandwidthCache->find(u64TransmitterBandwidthCacheKey);

      // cache found
      if(transmitterBandwidthCacheIter != pTransmitterBandwidthCache->end())
        {
          // get frequency overlap cache
          pCache = transmitterBandwidthCacheIter->second.first.get();
        }
      else
        {
          // none found - create a transmitter bandwidth cache
          pCache = new Cache{};

          transmitterBandwidthCacheIter = pTransmitterBandwidthCache->emplace(u64TransmitterBandwidthCacheKey,
                                                                              std::make_pair(std::unique_ptr<Cache>(pCache),
                                                                                             FrequencySet{})).first;
        }

      auto & noOverlapSet = transmitterBandwidthCacheIter->second.second;

      size_t i{};

      for(const auto & segment : segments)
        {
          double dOverlapRxPowerMillWatt{};
          bool bFrequencyMatch{};
          bool bAboveSensitivity{};

          TimePoint startOfReception{};
          TimePoint endOfReception{};

          std::uint64_t u64DopplerShiftedFequencyHz = segment.getFrequencyHz() +
            Utils::dopplerShift(segment.getFrequencyHz(),dDopplerFactor);

          auto iter = pCache->find(u64DopplerShiftedFequencyHz);

          auto validOffset = segment.getOffset();

          if(validOffset > maxOffset_)
            {
              if(bMaxClamp_)
                {
                  validOffset = maxOffset_;
                }
              else
                {
                  throw makeException<SpectrumServiceException>("Segment offset %ju usec > max offset %ju usec and max clamp is %s",
                                                                validOffset.count(),
                                                                maxOffset_.count(),
                                                                bMaxClamp_ ? "on" : "off");
                }
            }

          auto validDuration = segment.getDuration();

          if(validDuration > maxDuration_)
            {
              if(bMaxClamp_)
                {
                  validDuration = maxDuration_;
                }
              else
                {
                  throw makeException<SpectrumServiceException>("Segment duration %ju usec > max duration %ju usec and max clamp is %s",
                                                                validDuration.count(),
                                                                maxDuration_.count(),
                                                                bMaxClamp_ ? "on" : "off");
                }
            }

          if(iter != pCache->end())
            {
              for(auto & entry : iter->second)
                {
                  NoiseRecorder * pNoiseRecorder{std::get<0>(entry)};
                  auto & maskOverlap = std::get<1>(entry);
                  std::uint64_t u64RecorderFrequencyHz{std::get<2>(entry)};
                  auto & spectralOverlaps = std::get<0>(maskOverlap);
                  std::uint64_t u64LowerOverlapFrequencyHz{std::get<1>(maskOverlap)};
                  std::uint64_t u64UpperOverlapFrequencyHz{std::get<2>(maskOverlap)};

                  if(!spectralOverlaps.empty())
                    {
                      for(const auto & spectralOverlap : spectralOverlaps)
                        {
                          const auto & spectralSegments =  std::get<0>(spectralOverlap);

                          for(const auto & spectralSegment : spectralSegments)
                            {
                              dOverlapRxPowerMillWatt +=
                                // rx_power_mW * multipler_mWr * overlap_ratio
                                rxPowersMilliWatt[i] *
                                std::get<1>(spectralSegment) *
                                std::get<0>(spectralSegment);
                            }
                        }

                      if(dOverlapRxPowerMillWatt >= dReceiverSensitivityMilliWatt_)
                        {
                          bAboveSensitivity = true;

                          std::tie(startOfReception,endOfReception) =
                            pNoiseRecorder->update(now,
                                                   validTxTime,
                                                   validOffset,
                                                   validPropagation,
                                                   validDuration,
                                                   dOverlapRxPowerMillWatt,
                                                   transmitters,
                                                   u64LowerOverlapFrequencyHz,
                                                   u64UpperOverlapFrequencyHz,
                                                   txAntennaIndex);
                        }

                      if(!bFrequencyMatch)
                        {
                          bFrequencyMatch = (u64RecorderFrequencyHz == segment.getFrequencyHz());
                        }
                    }
                }
            }
          else if(!noOverlapSet.count(u64DopplerShiftedFequencyHz))
            {
              NoiseRecords recorderInfo;

              recorderInfo.reserve(noiseRecorderMap_.size());

              for(const auto & entry : noiseRecorderMap_)
                {
                  auto maskOverlap =
                    SpectralMaskManager::instance()->getSpectralOverlap(u64DopplerShiftedFequencyHz, // tx freq
                                                                        entry.first, // rx freq
                                                                        u64ReceiverBandwidthHz_, // rx bandwidth
                                                                        u64SegmentBandwidthHz,
                                                                        spectralMaskIndex);

                  auto & spectralOverlaps = std::get<0>(maskOverlap);
                  std::uint64_t u64LowerOverlapFrequencyHz{std::get<1>(maskOverlap)};
                  std::uint64_t u64UpperOverlapFrequencyHz{std::get<2>(maskOverlap)};

                  if(!spectralOverlaps.empty())
                    {
                      for(const auto & spectralOverlap : spectralOverlaps)
                        {
                          const auto & spectralSegments =  std::get<0>(spectralOverlap);

                          for(const auto & spectralSegment : spectralSegments)
                            {
                              dOverlapRxPowerMillWatt +=
                                // rx_power_mW * modifier_mWr * overlap_ratio
                                rxPowersMilliWatt[i] *
                                std::get<1>(spectralSegment) *
                                std::get<0>(spectralSegment);
                            }
                        }

                      if(dOverlapRxPowerMillWatt >= dReceiverSensitivityMilliWatt_)
                        {
                          bAboveSensitivity = true;

                          std::tie(startOfReception,endOfReception) =
                            entry.second->update(now,
                                                 validTxTime,
                                                 validOffset,
                                                 validPropagation,
                                                 validDuration,
                                                 dOverlapRxPowerMillWatt,
                                                 transmitters,
                                                 u64LowerOverlapFrequencyHz,
                                                 u64UpperOverlapFrequencyHz,
                                                 txAntennaIndex);
                        }

                      if(!bFrequencyMatch)
                        {
                          bFrequencyMatch = (entry.first == segment.getFrequencyHz());
                        }

                      recorderInfo.emplace_back(entry.second.get(),
                                                std::move(maskOverlap),
                                                entry.first,
                                                u64LowerOverlapFrequencyHz,
                                                u64UpperOverlapFrequencyHz);
                    }
                  else
                    {
                      noOverlapSet.insert(u64DopplerShiftedFequencyHz);
                    }
                }

              pCache->emplace(u64DopplerShiftedFequencyHz,std::move(recorderInfo));
            }

          if(bFrequencyMatch)
            {
              if(bInBand)
                {
                  bReportAsInBand = true;
                }

              if(bAboveSensitivity)
                {
                  maxEoR = std::max(maxEoR,endOfReception);

                  minSoR = std::min(minSoR,startOfReception);

                  reportableFrequencySegments.emplace_back(segment.getFrequencyHz(),
                                                           Utils::MILLIWATT_TO_DB(dOverlapRxPowerMillWatt),//rxPowersMilliWatt[i]),
                                                           validDuration,
                                                           validOffset);
                }
            }

          if(!bExcludeSameSubIdFromFilter_ || u16SubId_ != u16SubId)
            {
              // apply engery to any filters
              applyEnergyToFilters_i(u64SegmentBandwidthHz,
                                     segment.getFrequencyHz(),
                                     u16SubId,
                                     dDopplerFactor,
                                     spectralMaskIndex,
                                     now,
                                     validTxTime,
                                     validOffset,
                                     validPropagation,
                                     validDuration,
                                     rxPowersMilliWatt[i],
                                     transmitters,
                                     txAntennaIndex,
                                     optionalFilterData);
            }

          ++i;
        }
    }

  return std::make_tuple(validTxTime,
                         validPropagation,
                         std::chrono::duration_cast<Microseconds>(maxEoR - minSoR),
                         reportableFrequencySegments,
                         bReportAsInBand,
                         dReceiverSensitivityMilliWatt_);
}

EMANE::FrequencySet
EMANE::SpectrumMonitor::getFrequencies() const
{
  std::lock_guard<std::mutex> m(mutex_);

  return foi_;
}

double EMANE::SpectrumMonitor::getReceiverSensitivitydBm() const
{
  std::lock_guard<std::mutex> m(mutex_);

  return Utils::MILLIWATT_TO_DB(dReceiverSensitivityMilliWatt_);
}

EMANE::SpectrumWindow
EMANE::SpectrumMonitor::request_i(const TimePoint & now,
                                  std::uint64_t u64FrequencyHz,
                                  const Microseconds & duration,
                                  const TimePoint & timepoint) const
{
  std::lock_guard<std::mutex> m(mutex_);

  auto validDuration = duration;

  if(validDuration > maxDuration_)
    {
      if(bMaxClamp_)
        {
          validDuration = maxDuration_;
        }
      else
        {
          throw makeException<SpectrumServiceException>("Segment duration %ju usec > max duration %ju usec and max clamp is %s",
                                                        validDuration.count(),
                                                        maxDuration_.count(),
                                                        bMaxClamp_ ? "on" : "off");
        }
    }

  const auto iter = noiseRecorderMap_.find(u64FrequencyHz);

  if(iter!= noiseRecorderMap_.end())
    {
      auto ret = iter->second->get(now,validDuration,timepoint);

      return std::tuple_cat(std::move(ret),std::make_tuple(binSize_,dReceiverSensitivityMilliWatt_,mode_==NoiseMode::ALL));
    }
  else
    {
      return SpectrumWindow{{},{},{},{},false};
    }
}

EMANE::SpectrumWindow
EMANE::SpectrumMonitor::request(std::uint64_t u64FrequencyHz,
                                const Microseconds & duration,
                                const TimePoint & timepoint) const
{
  return request_i(Clock::now(),u64FrequencyHz,duration,timepoint);
}

EMANE::SpectrumFilterWindow
EMANE::SpectrumMonitor::requestFilter(FilterIndex filterIndex,
                                      const Microseconds & duration,
                                      const TimePoint & timepoint) const
{
  return requestFilter_i(Clock::now(),filterIndex,duration,timepoint);
}

EMANE::SpectrumFilterWindow
EMANE::SpectrumMonitor::requestFilter_i(const TimePoint & now,
                                        FilterIndex filterIndex,
                                        const Microseconds & duration,
                                        const TimePoint & timepoint) const
{
  std::lock_guard<std::mutex> m(mutex_);

  auto validDuration = duration;

  if(validDuration > maxDuration_)
    {
      if(bMaxClamp_)
        {
          validDuration = maxDuration_;
        }
      else
        {
          throw makeException<SpectrumServiceException>("Segment duration %ju usec > max duration %ju usec and max clamp is %s",
                                                        validDuration.count(),
                                                        maxDuration_.count(),
                                                        bMaxClamp_ ? "on" : "off");
        }
    }

  const auto iter =  filterNoiseRecorderMap_.find(filterIndex);

  if(iter !=  filterNoiseRecorderMap_.end())
    {
      auto ret = std::get<2>(iter->second)->get(now,validDuration,timepoint);

      return std::tuple_cat(std::move(ret),std::make_tuple(binSize_,
                                                           dReceiverSensitivityMilliWatt_,
                                                           std::get<2>(iter->second)->getSubBandBinCount()));
    }
  else
    {
      throw makeException<SpectrumServiceException>("Unknown filter id %hu",filterIndex);
    }
}


std::vector<double> EMANE::SpectrumMonitor::dump(std::uint64_t u64FrequencyHz) const
{
  std::lock_guard<std::mutex> m(mutex_);

  const auto iter = noiseRecorderMap_.find(u64FrequencyHz);

  if(iter!= noiseRecorderMap_.end())
    {
      return iter->second->dump();
    }

  return {};
}


std::pair<std::vector<double>,std::size_t>
EMANE::SpectrumMonitor::dumpFilter(FilterIndex filterIndex) const
{
  std::lock_guard<std::mutex> m(mutex_);

  auto iter = filterNoiseRecorderMap_.find(filterIndex);

  if(iter != filterNoiseRecorderMap_.end())
    {
      return std::make_pair(std::get<2>(iter->second)->dump(),
                            std::get<2>(iter->second)->getSubBandBinCount());
    }

  return {{},0};
}


void EMANE::SpectrumMonitor::initializeFilter(FilterIndex filterIndex,
                                              std::uint64_t u64FrequencyHz,
                                              std::uint64_t u64BandwidthHz,
                                              std::uint64_t u64BandwidthBinSizeHz,
                                              const FilterMatchCriterion * pFilterMatchCriterion)
{
  auto iter = filterNoiseRecorderMap_.find(filterIndex);

  if(iter == filterNoiseRecorderMap_.end())
    {
      filterTransmitterBandwidthCache_.clear();

      filterNoiseRecorderMap_.insert(std::make_pair(filterIndex,
                                                    std::make_tuple(u64FrequencyHz,
                                                                    u64BandwidthHz,
                                                                    std::unique_ptr<NoiseRecorder>{new NoiseRecorder{binSize_,
                                                                          maxOffset_,
                                                                          maxPropagation_,
                                                                          maxDuration_,
                                                                          dReceiverSensitivityMilliWatt_,
                                                                          u64FrequencyHz,
                                                                          u64BandwidthHz,
                                                                          u64BandwidthBinSizeHz}},
                                                                    std::unique_ptr<const FilterMatchCriterion>{pFilterMatchCriterion})));
    }
  else
    {
      throw makeException<SpectrumServiceException>("Filter id %hu already present",
                                                    filterIndex);
    }
}

void EMANE::SpectrumMonitor::removeFilter(FilterIndex filterIndex)
{
  if(filterNoiseRecorderMap_.erase(filterIndex))
    {
      filterTransmitterBandwidthCache_.clear();
    }
}

void EMANE::SpectrumMonitor::applyEnergyToFilters_i(std::uint64_t u64TxBandwidthHz,
                                                    std::uint64_t u64TxFrequencyHz,
                                                    std::uint16_t u16SubId,
                                                    double dDopplerFactor,
                                                    SpectralMaskIndex spectralMaskIndex,
                                                    const TimePoint & now,
                                                    const TimePoint & txTime,
                                                    const Microseconds & offset,
                                                    const Microseconds & propagation,
                                                    const Microseconds & duration,
                                                    double dRxPowerMilliWatt,
                                                    const std::vector<NEMId> & transmitters,
                                                    AntennaIndex txAntennaIndex,
                                                    const std::pair<FilterData,bool> & optionalFilterData)
{
  FilterTransmitterBandwidthCache * pFilterTransmitterBandwidthCache{};
  std::uint64_t u64FilterTransmitterBandwidthCacheKey{};

  if(spectralMaskIndex)
    {
      pFilterTransmitterBandwidthCache = &filterTransmitterSpectralMaskCache_;
      u64FilterTransmitterBandwidthCacheKey = spectralMaskIndex;
    }
  else
    {
      pFilterTransmitterBandwidthCache = &filterTransmitterBandwidthCache_;
      u64FilterTransmitterBandwidthCacheKey = u64TxBandwidthHz;
    }

  FilterCache * pCache{};

  // check for a entry in the filter bandwitdh cache for a given transmit bandwidth
  auto filterTransmitterBandwidthCacheIter =
    pFilterTransmitterBandwidthCache->find(u64FilterTransmitterBandwidthCacheKey);

  if(filterTransmitterBandwidthCacheIter != pFilterTransmitterBandwidthCache->end())
    {
      pCache = filterTransmitterBandwidthCacheIter->second.first.get();
    }
  else
    {
      // none found - create a transmitter bandwidth cache
      pCache = new FilterCache{};

      filterTransmitterBandwidthCacheIter =
        pFilterTransmitterBandwidthCache->emplace(u64FilterTransmitterBandwidthCacheKey,
                                                  std::make_pair(std::unique_ptr<FilterCache>(pCache),
                                                                 FrequencySet{})).first;
    }

  auto & noOverlapSet = filterTransmitterBandwidthCacheIter->second.second;

  std::uint64_t u64DopplerShiftedFequencyHz = u64TxFrequencyHz +
    Utils::dopplerShift(u64TxFrequencyHz,dDopplerFactor);

  // check for the tx frequency cache entry
  auto cacheIter = pCache->find(u64DopplerShiftedFequencyHz);

  if(cacheIter != pCache->end())
    {
      // frequency entry found, iterate over all applicable filters and apply the energy
      for(auto & entry : cacheIter->second)
        {
          NoiseRecorder * pNoiseRecorder{std::get<0>(entry)};
          auto & maskOverlap = std::get<1>(entry);
          const FilterMatchCriterion * pFilterMatchCriterion{std::get<3>(entry)};
          std::uint64_t u64LowerOverlapFrequencyHz{std::get<4>(entry)};
          std::uint64_t u64UpperOverlapFrequencyHz{std::get<5>(entry)};
          std::uint64_t u64TotalSegments{std::get<3>(maskOverlap)};

          // subid matching this waveform subid is refered to as subid 0
          bool bApply{pFilterMatchCriterion ? (*pFilterMatchCriterion)({u64TxFrequencyHz,
                u64UpperOverlapFrequencyHz - u64LowerOverlapFrequencyHz,
                u16SubId == u16SubId_ ?
                static_cast<unsigned short>(0) : u16SubId,
                optionalFilterData.first}) : true};

          auto & spectralOverlaps = std::get<0>(maskOverlap);

          if(bApply && !spectralOverlaps.empty() > 0)
            {
              double dOverlapRxPowerMillWatt{};

              for(const auto & spectralOverlap : spectralOverlaps)
                {
                  const auto & spectralSegments =  std::get<0>(spectralOverlap);

                  for(const auto & spectralSegment : spectralSegments)
                    {
                      dOverlapRxPowerMillWatt +=
                        // rx_power_mW * modifier_mWr * overlap_ratio
                        dRxPowerMilliWatt *
                        std::get<1>(spectralSegment) *
                        std::get<0>(spectralSegment);
                    }
                }

              if(dOverlapRxPowerMillWatt >= dReceiverSensitivityMilliWatt_)
                {
                  std::uint64_t u64Index{1};

                  if(pNoiseRecorder->getSubBandBinCount() > 1)
                    {
                      for(const auto & spectralOverlap : spectralOverlaps)
                        {
                          const auto & spectralSegments =  std::get<0>(spectralOverlap);

                          for(const auto & spectralSegment : spectralSegments)
                            {
                              // rx_power_mW * multipler_mWr * overlap_ratio
                              dOverlapRxPowerMillWatt =
                                dRxPowerMilliWatt *
                                std::get<1>(spectralSegment) *
                                std::get<0>(spectralSegment);

                              pNoiseRecorder->update(now,
                                                     txTime,
                                                     offset,
                                                     propagation,
                                                     duration,
                                                     dOverlapRxPowerMillWatt,
                                                     transmitters,
                                                     std::get<2>(spectralSegment),
                                                     std::get<3>(spectralSegment),
                                                     txAntennaIndex,
                                                     u64Index != u64TotalSegments);

                              ++u64Index;
                            }
                        }
                    }
                  else
                    {
                      pNoiseRecorder->update(now,
                                             txTime,
                                             offset,
                                             propagation,
                                             duration,
                                             dOverlapRxPowerMillWatt,
                                             transmitters,
                                             u64LowerOverlapFrequencyHz,
                                             u64UpperOverlapFrequencyHz,
                                             txAntennaIndex);
                    }
                }
            }
        }
    }
  else if(!noOverlapSet.count(u64DopplerShiftedFequencyHz))
    {
      FilterRecords filterRecorderInfo;

      filterRecorderInfo.reserve(filterNoiseRecorderMap_.size());

      for(const auto & entry : filterNoiseRecorderMap_)
        {
          std::uint64_t u64FilterBandwidthHz = std::get<1>(entry.second);
          auto pNoiseRecorder = std::get<2>(entry.second).get();
          auto pFilterMatchCriterion = std::get<3>(entry.second).get();

          auto maskOverlap =
            SpectralMaskManager::instance()->getSpectralOverlap(u64DopplerShiftedFequencyHz, // tx freq
                                                                std::get<0>(entry.second), // rx freq
                                                                u64FilterBandwidthHz,
                                                                u64TxBandwidthHz,
                                                                spectralMaskIndex);

          auto & spectralOverlaps = std::get<0>(maskOverlap);
          std::uint64_t u64LowerOverlapFrequencyHz{std::get<1>(maskOverlap)};
          std::uint64_t u64UpperOverlapFrequencyHz{std::get<2>(maskOverlap)};
          std::uint64_t u64TotalSegments{std::get<3>(maskOverlap)};

          // subid matching this waveform subid is refered to as subid 0
          bool bApply{pFilterMatchCriterion ? (*pFilterMatchCriterion)({u64TxFrequencyHz,
                u64UpperOverlapFrequencyHz - u64LowerOverlapFrequencyHz, //u64TxBandwidthHz,
                u16SubId == u16SubId_ ?
                static_cast<unsigned short>(0) : u16SubId,
                optionalFilterData.first}) : true};

          if(bApply && !spectralOverlaps.empty())
            {
              double dOverlapRxPowerMillWatt{};

              for(const auto & spectralOverlap : spectralOverlaps)
                {
                  const auto & spectralSegments =  std::get<0>(spectralOverlap);

                  for(const auto & spectralSegment : spectralSegments)
                    {
                      dOverlapRxPowerMillWatt +=
                        // rx_power_mW * multipler_mWr * overlap_ratio
                        dRxPowerMilliWatt *
                        std::get<1>(spectralSegment) *
                        std::get<0>(spectralSegment);
                    }
                }

              if(dOverlapRxPowerMillWatt >= dReceiverSensitivityMilliWatt_)
                {
                  if(pNoiseRecorder->getSubBandBinCount() > 1)
                    {
                      std::uint64_t u64Index{1};

                      for(const auto & spectralOverlap : spectralOverlaps)
                        {
                          const auto & spectralSegments =  std::get<0>(spectralOverlap);

                          for(const auto & spectralSegment : spectralSegments)
                            {
                              // overlap is calculated for each sub
                              // bin by the noise recorder
                              pNoiseRecorder->update(now,
                                                     txTime,
                                                     offset,
                                                     propagation,
                                                     duration,
                                                     dRxPowerMilliWatt * std::get<1>(spectralSegment),
                                                     transmitters,
                                                     std::get<2>(spectralSegment),
                                                     std::get<3>(spectralSegment),
                                                     txAntennaIndex,
                                                     u64Index != u64TotalSegments);

                              ++u64Index;
                            }
                        }
                    }
                  else
                    {
                      pNoiseRecorder->update(now,
                                             txTime,
                                             offset,
                                             propagation,
                                             duration,
                                             dOverlapRxPowerMillWatt,
                                             transmitters,
                                             u64LowerOverlapFrequencyHz,
                                             u64UpperOverlapFrequencyHz,
                                             txAntennaIndex);
                    }
                }

              filterRecorderInfo.emplace_back(pNoiseRecorder,
                                              std::move(maskOverlap),
                                              u64TxFrequencyHz,
                                              pFilterMatchCriterion,
                                              u64LowerOverlapFrequencyHz,
                                              u64UpperOverlapFrequencyHz);
            }
        }

      pCache->emplace(u64DopplerShiftedFequencyHz,std::move(filterRecorderInfo));
    }
}
