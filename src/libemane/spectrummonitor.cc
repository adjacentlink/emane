/*
 * Copyright (c) 2013-2015 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "emane/spectrumserviceexception.h"
#include "emane/utils/conversionutils.h"

#include <algorithm>

namespace
{
  double frequencyOverlapRatio(std::uint64_t u64FrequencyHz1,
                               std::uint64_t u64BandwidthHz1,
                               std::uint64_t u64FrequencyHz2,
                               std::uint64_t u64BandwidthHz2);
}

EMANE::SpectrumMonitor::SpectrumMonitor():
  binSize_{},
  maxOffset_{},
  maxPropagation_{},
  maxDuration_{},
  bMaxClamp_{},
  timeSyncThreshold_{},
  u64ReceiverBandwidthHz_{},
  mode_{NoiseMode::NONE},
  dReceiverSensitivityMilliWatt_{}{}

void EMANE::SpectrumMonitor::initialize(const FrequencySet & foi,
                                        std::uint64_t u64BandwidthHz,
                                        double dReceiverSensitivityMilliWatt,
                                        NoiseMode mode,
                                        const Microseconds & binSize,
                                        const Microseconds & maxOffset,
                                        const Microseconds & maxPropagation,
                                        const Microseconds & maxDuration,
                                        const Microseconds & timeSyncThreshold,
                                        bool bMaxClamp)
{
  std::lock_guard<std::mutex> m(mutex_);

  binSize_ = binSize;

  mode_ = mode;

  maxOffset_ = maxOffset;
  
  maxPropagation_ = maxPropagation;

  maxDuration_ = maxDuration;

  bMaxClamp_ = bMaxClamp;

  timeSyncThreshold_ = timeSyncThreshold;

  dReceiverSensitivityMilliWatt_ = dReceiverSensitivityMilliWatt;

  u64ReceiverBandwidthHz_ = u64BandwidthHz;

  // (re-)initialize transmitter bandwidth cache
  transmitterBandwidthCache_.clear();

  TransmitterBandwidthCache transmitterBandwidthCache_;
  
  transmitterBandwidthCache_.insert(std::make_pair(u64BandwidthHz,std::unique_ptr<Cache>(new Cache{})));
  
  noiseRecorderMap_.clear();
  
  for(const auto & frequency : foi)
    {
      noiseRecorderMap_.insert(std::make_pair(frequency,
                                              std::unique_ptr<NoiseRecorder>{new NoiseRecorder{binSize,
                                                    maxOffset,
                                                    maxPropagation,
                                                    maxDuration,
                                                    dReceiverSensitivityMilliWatt}}));
    }

}
    
std::tuple<EMANE::TimePoint,EMANE::Microseconds,EMANE::Microseconds,EMANE::FrequencySegments,bool>
EMANE::SpectrumMonitor::update(const TimePoint & now,
                               const TimePoint & txTime,
                               const Microseconds & propagationDelay,
                               const FrequencySegments & segments,
                               std::uint64_t u64SegmentBandwidthHz,
                               const std::vector<double> & rxPowersMilliWatt,
                               bool bInBand,
                               const std::vector<NEMId> & transmitters)
{
  std::lock_guard<std::mutex> m(mutex_);

  if(segments.size() != rxPowersMilliWatt.size())
    {
      return std::make_tuple(TimePoint{},Microseconds{},Microseconds{},FrequencySegments{},false);
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
  if(mode_ == NoiseMode::NONE || (mode_ == NoiseMode::OUTOFBAND && bInBand))
    {
      size_t i{};

      bReportAsInBand = true;

      for(const auto & segment : segments)
        {
          if(noiseRecorderMap_.find(segment.getFrequencyHz()) != noiseRecorderMap_.end())
            {
              if(rxPowersMilliWatt[i] >= dReceiverSensitivityMilliWatt_)
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

                  reportableFrequencySegments.push_back({segment.getFrequencyHz(),
                        validDuration,
                        validOffset,
                        Utils::MILLIWATT_TO_DB(rxPowersMilliWatt[i])});
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
      Cache * pCache{};
  
      // find the cache for the transmitter bandwidth
      const auto transmitterBandwidthCacheIter =
        transmitterBandwidthCache_.find(u64SegmentBandwidthHz);
  
      // cache found
      if(transmitterBandwidthCacheIter != transmitterBandwidthCache_.end())
        {
          // get frequency overlap cache
          pCache = transmitterBandwidthCacheIter->second.get();
        }
      else
        {
          // none found - create a transmitter bandwidth cache
          pCache = new Cache{};
      
          transmitterBandwidthCache_.insert(std::make_pair(u64SegmentBandwidthHz,
                                                           std::unique_ptr<Cache>(pCache)));
        }

      size_t i{};
      
      for(const auto & segment : segments)
        {
          bool bFrequencyMatch{};
          bool bAboveSensitivity{};

          TimePoint startOfReception{};
          TimePoint endOfReception{};

          auto iter = pCache->find(segment.getFrequencyHz());

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
              for(const auto & entry : iter->second)
                {
                  NoiseRecorder * pNoiseRecorder{};
                  double dOverlapRatio{};
                  std::uint64_t u64RecorderFrequencyHz{};

                  std::tie(pNoiseRecorder,
                           dOverlapRatio,
                           u64RecorderFrequencyHz) = entry;
                
                  if(dOverlapRatio > 0)
                    {
                      double dOverlapRxPowerMillWatt{rxPowersMilliWatt[i] * dOverlapRatio};

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
                                                   transmitters);
                        }
                      
                      if(!bFrequencyMatch)
                        {
                          bFrequencyMatch = (u64RecorderFrequencyHz == segment.getFrequencyHz());
                        }
                    }
                }
            }
          else
            {
              std::vector<std::tuple<NoiseRecorder *,double,std::uint64_t>> recorderInfo;

              recorderInfo.reserve(noiseRecorderMap_.size());

              for(const auto & entry : noiseRecorderMap_)
                {
                  double dOverlapRatio{frequencyOverlapRatio(entry.first,
                                                             u64ReceiverBandwidthHz_,
                                                             segment.getFrequencyHz(),
                                                             u64SegmentBandwidthHz)};
              
                  if(dOverlapRatio > 0)
                    {
                      double dOverlapRxPowerMillWatt{rxPowersMilliWatt[i] * dOverlapRatio};

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
                                                 transmitters);
                        }

                      if(!bFrequencyMatch)
                        {
                          bFrequencyMatch = (entry.first == segment.getFrequencyHz());
                        }
                    }

                  recorderInfo.push_back(std::make_tuple(entry.second.get(),dOverlapRatio,entry.first));
                }

              pCache->insert({segment.getFrequencyHz(),std::move(recorderInfo)});
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
              
                  reportableFrequencySegments.push_back({segment.getFrequencyHz(),
                        validDuration,
                        validOffset,
                        Utils::MILLIWATT_TO_DB(rxPowersMilliWatt[i])});
                }
            }

          ++i;
        }
    }
  
  return std::make_tuple(validTxTime,
                         validPropagation,
                         std::chrono::duration_cast<Microseconds>(maxEoR - minSoR),
                         reportableFrequencySegments,
                         bReportAsInBand);
}

EMANE::FrequencySet
EMANE::SpectrumMonitor::getFrequencies() const
{
  std::lock_guard<std::mutex> m(mutex_);

  FrequencySet frequencySet;

  std::transform(noiseRecorderMap_.begin(),
                 noiseRecorderMap_.end(),
                 std::inserter(frequencySet,frequencySet.begin()),
                 std::bind(&NoiseRecorderMap::value_type::first,
                           std::placeholders::_1));
  return frequencySet;
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

namespace
{
  double frequencyOverlapRatio(std::uint64_t u64FrequencyHz1,
                               std::uint64_t u64BandwidthHz1,
                               std::uint64_t u64FrequencyHz2,
                               std::uint64_t u64BandwidthHz2)
  {
    double u64UpperFrequencyHz1{u64FrequencyHz1 + u64BandwidthHz1 / 2.0};
    double u64LowerFrequencyHz1{u64FrequencyHz1 - u64BandwidthHz1 / 2.0};

    double u64UpperFrequencyHz2{u64FrequencyHz2 + u64BandwidthHz2 / 2.0};
    double u64LowerFrequencyHz2{u64FrequencyHz2 - u64BandwidthHz2 / 2.0};
    
    // percent in band, defaults to no coverage
    double dRatio{};

    // signal is somewhere in band
    if((u64LowerFrequencyHz2 < u64UpperFrequencyHz1) && (u64UpperFrequencyHz2 > u64LowerFrequencyHz1))
      {
        // low is within lower bound
        if(u64LowerFrequencyHz2 >= u64LowerFrequencyHz1)
          {
            // high is within upper bound
            if(u64UpperFrequencyHz2 <= u64UpperFrequencyHz1)
              {
                // full coverage
                dRatio = 1.0;
              }
            // exceeded upper bound
            else
              {
                // partial coverage
                dRatio = (u64UpperFrequencyHz1 - u64LowerFrequencyHz2) / u64BandwidthHz2;
              }
          }
        // low is below lower bound
        else
          {
            // the signal is at or beyond
            if(  u64UpperFrequencyHz2 <= u64UpperFrequencyHz1)
              {
                // partial coverage
                dRatio = (u64UpperFrequencyHz2 - u64LowerFrequencyHz1) / u64BandwidthHz2;
              }
            else
              {
                dRatio = (u64UpperFrequencyHz1 - u64LowerFrequencyHz1) / u64BandwidthHz2;
              }
          }
      }
    
    // return ratio 
    return dRatio;
  }
}
