/*
 * Copyright (c) 2013-2014,2019-2021 - Adjacent Link LLC, Bridgewater,
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

#include "noiserecorder.h"
#include "frequencyoverlapratio.h"
#include "emane/spectrumserviceexception.h"
#include <cmath>

EMANE::NoiseRecorder::NoiseRecorder(const Microseconds & bin,
                                    const Microseconds & maxOffset,
                                    const Microseconds & maxPropagation,
                                    const Microseconds & maxDuration,
                                    double dRxSensitivityMilliWatt,
                                    std::uint64_t u64FrequencyHz,
                                    std::uint64_t u64BandwidthHz,
                                    std::uint64_t u64BandwidthBinSizeHz):
  totalWindowBins_{maxDuration/bin},
  totalWheelBins_{(maxOffset + maxPropagation + 2 * maxDuration)/bin},
  binSizeMicroseconds_{bin.count()},
  u64BandwidthBinSizeHz_{u64BandwidthBinSizeHz},
  u64BandStartFrequencyHz_{static_cast<std::uint64_t>(u64FrequencyHz - u64BandwidthHz / 2.0)},
  totalSubBandBins_{u64BandwidthBinSizeHz ? static_cast<size_t>(std::ceil(u64BandwidthHz/static_cast<double>(u64BandwidthBinSizeHz)))+1 : 1},
  u64BandEndFrequencyHz_{u64BandStartFrequencyHz_ +  totalSubBandBins_ * u64BandwidthBinSizeHz - 1},
  wheel_{static_cast<std::size_t>(totalWheelBins_), totalSubBandBins_},
  dRxSensitivityMilliWatt_{dRxSensitivityMilliWatt},
  maxEndOfReceptionBin_{},
  minStartOfReceptionBin_{}
{}

std::tuple<EMANE::TimePoint,EMANE::TimePoint>
EMANE::NoiseRecorder::update(const TimePoint &,
                             const TimePoint & txTime,
                             const Microseconds & offset,
                             const Microseconds & propagation,
                             const Microseconds & duration,
                             double dRxPower,
                             const std::vector<NEMId> & transmitters,
                             std::uint64_t u64StartFrequencyHz,
                             std::uint64_t u64EndFrequencyHz,
                             AntennaIndex txAntennaIndex,
                             bool bIsMore)
{
  auto startOfReception = txTime + offset + propagation;

  auto endOfReception = startOfReception + duration;

  auto reportedStartOfReceptionBin = timepointToBin(startOfReception);

  auto endOfReceptionBin = timepointToBin(endOfReception,true);

  Microseconds::rep startOfReceptionBin{reportedStartOfReceptionBin};

  Microseconds::rep storedMaxEoRBin{};

  size_t subBandBinStart{0};
  size_t subBandBinEnd{totalSubBandBins_ -1};
  size_t subBandBins{1};

  BinPowerApply pendingBinPowerApply{};
  BinPowerApplyMap::iterator binPowerApplyMapIter{};

  if(u64BandwidthBinSizeHz_)
    {
      if(u64StartFrequencyHz > u64BandStartFrequencyHz_)
        {
          subBandBinStart = (u64StartFrequencyHz - u64BandStartFrequencyHz_) / u64BandwidthBinSizeHz_;
        }

      if(u64EndFrequencyHz < u64BandEndFrequencyHz_)
        {
          subBandBinEnd = totalSubBandBins_ - (u64BandEndFrequencyHz_ - u64EndFrequencyHz) / u64BandwidthBinSizeHz_;
        }


      subBandBins = subBandBinEnd - subBandBinStart + 1;

      binPowerApplyMapIter =
        binPowerApplyMap_.find(std::make_pair(u64StartFrequencyHz,u64EndFrequencyHz));

      // no cache entry found
      if(binPowerApplyMapIter == binPowerApplyMap_.end())
        {
          BinPowerApplies binPowerApplies{};

          // for each bin, determine the overlap multiplier
          for(size_t bin = subBandBinStart; bin <= subBandBinEnd; ++bin)
            {
              std::uint64_t u64BinStartFrequencyHz{bin * u64BandwidthBinSizeHz_ + u64BandStartFrequencyHz_};

              double dOverlapRatio{};
              std::uint64_t u64LowerOverlapFrequencyHz{};
              std::uint64_t u64UpperOverlapFrequencyHz{};

              std::tie(dOverlapRatio,
                       u64LowerOverlapFrequencyHz,
                       u64UpperOverlapFrequencyHz) =
                frequencyOverlapRatio(u64BinStartFrequencyHz + u64BandwidthBinSizeHz_ / 2,
                                      u64BandwidthBinSizeHz_,
                                      u64StartFrequencyHz + (u64EndFrequencyHz - u64StartFrequencyHz) / 2,
                                      u64EndFrequencyHz - u64StartFrequencyHz);


              // signal is partially in bin
              if(dOverlapRatio > 0 && dOverlapRatio < 1)
                {
                  // if keeping count of bins with full overlap, we
                  // need to store and reset
                  if(std::get<2>(pendingBinPowerApply))
                    {
                      // set end bin
                      std::get<1>(pendingBinPowerApply) = bin - 1;

                      // store the pending BinPowerApply
                      binPowerApplies.push_back(pendingBinPowerApply);

                      // reset, the multiplier used as a flag for valid data
                      std::get<2>(pendingBinPowerApply) = 0;
                    }

                  // store the current BinPowerApply, for the single bin
                  binPowerApplies.emplace_back(std::make_tuple(bin,bin,dOverlapRatio));
                }
              else if(dOverlapRatio == 1)
                {
                  // bin has full overlap, if not tracking -- begin
                  if(!std::get<2>(pendingBinPowerApply))
                    {
                      std::get<0>(pendingBinPowerApply) = bin;

                      std::get<2>(pendingBinPowerApply) = dOverlapRatio;
                    }
                }
              else
                {
                  // dOverlapRatio == 0
                  if(std::get<2>(pendingBinPowerApply))
                    {
                      // set end bin
                      std::get<1>(pendingBinPowerApply) = bin - 1;

                      // store the pending BinPowerApply
                      binPowerApplies.push_back(pendingBinPowerApply);

                      // reset, the multiplier used as a flag for valid data
                      std::get<2>(pendingBinPowerApply) = 0;
                    }
                }
            }

          // if the last bin was full overlap, we need to store
          if(std::get<2>(pendingBinPowerApply))
            {
              std::get<1>(pendingBinPowerApply) = subBandBinEnd;
              binPowerApplies.push_back(pendingBinPowerApply);
            }

          binPowerApplyMapIter =
            binPowerApplyMap_.emplace(std::make_pair(u64StartFrequencyHz,
                                                     u64EndFrequencyHz),
                                      std::move(binPowerApplies)).first;
        }
    }

  // Determine the last EoR bin for the transmitter - we only
  // allow one bin noise entry per transmitter per tx antenna. For
  // multiple transmitters use the max EoR

  for(const auto & transmitter : transmitters)
    {
      auto nemIter = nemAntennaIndexEORBinMap_.find(transmitter);

      if(nemIter == nemAntennaIndexEORBinMap_.end())
        {
          nemIter = nemAntennaIndexEORBinMap_.emplace(transmitter,
                                                      AntennaIndexEORMap{}).first;
        }

      auto antennaIter = nemIter->second.find(txAntennaIndex);

      if(antennaIter != nemIter->second.end())
        {
          storedMaxEoRBin = std::max(antennaIter->second,storedMaxEoRBin);
        }
    }

  Microseconds::rep maxEoRBin{storedMaxEoRBin};

  // adjust the SoR bin to be the next after max EoR bin
  // if necessary
  if(maxEoRBin >= reportedStartOfReceptionBin)
    {
      startOfReceptionBin = maxEoRBin + 1;
    }

  // sanity check after any possible adjustments
  if(startOfReceptionBin <= endOfReceptionBin)
    {
      auto durationBinCount = endOfReceptionBin - startOfReceptionBin + 1;

      auto startIndex = startOfReceptionBin % totalWheelBins_;

      // is current SOR on the wheel
      if(maxEndOfReceptionBin_ + totalWheelBins_ <= startOfReceptionBin)
        {
          // reset the max end of reception bin
          //  the last bin we received on
          maxEndOfReceptionBin_ = 0;

          // reset the min start of reception bin
          // the first bin we received on
          minStartOfReceptionBin_ = 0;
        }

      // if wheel is empty, place entire duration
      if(!maxEndOfReceptionBin_ && !minStartOfReceptionBin_)
        {
          if(totalSubBandBins_ > 1)
            {
              for(const auto & binPowerApply :
                    binPowerApplyMapIter->second)
                {
                  auto & start = std::get<0>(binPowerApply);
                  auto & end = std::get<1>(binPowerApply);
                  auto & dMultipler = std::get<2>(binPowerApply);

                  wheel_.set(startIndex,
                             durationBinCount,
                             dRxPower * dMultipler,
                             start,
                             end - start + 1);
                }
            }
          else
            {
              // we can fill the entire duration
              wheel_.set(startIndex,
                         durationBinCount,
                         dRxPower,
                         subBandBinStart,
                         subBandBins);
            }

          minStartOfReceptionBin_ = startOfReceptionBin;

          maxEndOfReceptionBin_ = endOfReceptionBin;
        }
      else
        {
          Microseconds::rep beforeMinSORBinDurationCount{};
          Microseconds::rep afterMaxEORBinDurationCount{};

          // set any values before the minStartOfReceptionBin, we are
          // not accumulating energy - any value in these bins is
          // stale
          if(startOfReceptionBin < minStartOfReceptionBin_)
            {
              // all or part of energy within [MinSOR,MaxEOR]
              //
              // current wheel state. (x) indicates valid energy
              //
              // max EOR ---------------------
              // min SOR -------              |
              //                |             |
              //                v             v
              //  0                   1                   2
              //  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0
              // | | | | | | | |x|x|x|x|x|x|x|x| | | | | | |
              //
              // energy (b) to record before MinSOR w/ overlap into
              // [MinSOR,MaxEOR].
              //
              //  0                   1                   2
              //  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0
              // | | | | | | | |x|x|x|x|x|x|x|x| | | | | | |
              // | | | | | |b|b|b|b|b| | | | | | | | | | | |
              //                ^ ^ ^
              //                | | |
              //                energy to accumulate

              if(endOfReceptionBin < minStartOfReceptionBin_)
                {
                  beforeMinSORBinDurationCount = durationBinCount;
                }
              else
                {
                  beforeMinSORBinDurationCount =
                    minStartOfReceptionBin_ - startOfReceptionBin;
                }

              if(totalSubBandBins_ > 1)
                {
                  wheel_.set(startIndex,
                             beforeMinSORBinDurationCount,
                             0,
                             0,
                             totalSubBandBins_);

                  for(const auto & binPowerApply :
                        binPowerApplyMapIter->second)
                    {
                      auto & start = std::get<0>(binPowerApply);
                      auto & end = std::get<1>(binPowerApply);
                      auto & dMultipler = std::get<2>(binPowerApply);

                      wheel_.set(startIndex,
                                 durationBinCount,
                                 dRxPower * dMultipler,
                                 start,
                                 end - start + 1);
                    }
                }
              else
                {
                  wheel_.set(startIndex,
                             beforeMinSORBinDurationCount,
                             dRxPower,
                             subBandBinStart,
                             subBandBins);
                }
            }

          // set any values after the maxEndOfReceptionBin, we are
          // not accumulating energy - any value in these bins is
          // stale
          if(endOfReceptionBin > maxEndOfReceptionBin_)
            {
              // all or part of energy within [MinSOR,MaxEOR]
              //
              // current wheel state. (x) indicates valid energy
              //
              // max EOR ---------------------
              // min SOR -------              |
              //                |             |
              //                v             v
              //  0                   1                   2
              //  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0
              // | | | | | | | |x|x|x|x|x|x|x|x| | | | | | |
              //
              // energy (a) to record after MaxEOR w/ no overlap into
              // [MinSOR,MaxEOR]
              //  0                   1                   2
              //  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0
              // | | | | | | | |x|x|x|x|x|x|x|x| | | | | | |
              // | | | | | | | | | | | | |a|a|a|a|a| | | | |
              //                          ^ ^ ^
              //                          | | |
              //                          energy to accumulate

              if(startOfReceptionBin > maxEndOfReceptionBin_)
                {
                  afterMaxEORBinDurationCount = durationBinCount;
                }
              else
                {
                  afterMaxEORBinDurationCount = endOfReceptionBin - maxEndOfReceptionBin_;
                }

              if(totalSubBandBins_ > 1)
                {
                  wheel_.set((startIndex + durationBinCount - afterMaxEORBinDurationCount) % totalWheelBins_,
                             afterMaxEORBinDurationCount,
                             0,
                             0,
                             totalSubBandBins_);

                  for(const auto & binPowerApply :
                        binPowerApplyMapIter->second)
                    {
                      auto & start = std::get<0>(binPowerApply);
                      auto & end = std::get<1>(binPowerApply);
                      auto & dMultipler = std::get<2>(binPowerApply);

                      wheel_.set(startIndex,
                                 durationBinCount,
                                 dRxPower * dMultipler,
                                 start,
                                 end - start + 1);
                    }
                }
              else
                {
                  wheel_.set((startIndex + durationBinCount - afterMaxEORBinDurationCount) % totalWheelBins_,
                             afterMaxEORBinDurationCount,
                             dRxPower,
                             subBandBinStart,
                             subBandBins);
                }
            }

          auto withinMinSORMaxEORBinCount =
            durationBinCount - (beforeMinSORBinDurationCount + afterMaxEORBinDurationCount);

          // energy within existing valid wheel bins [MinSOR,MaxEOR]
          // must be accumulated
          if(withinMinSORMaxEORBinCount)
            {
              // all or part of energy within [MinSOR,MaxEOR]
              //
              // current wheel state. (x) indicates valid energy
              //
              // max EOR ---------------------
              // min SOR -------              |
              //                |             |
              //                v             v
              //  0                   1                   2
              //  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0
              // | | | | | | | |x|x|x|x|x|x|x|x| | | | | | |
              //
              // energy (b) to record before MinSOR w/ overlap into
              // [MinSOR,MaxEOR].
              //
              //  0                   1                   2
              //  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0
              // | | | | | | | |x|x|x|x|x|x|x|x| | | | | | |
              // | | | | | |b|b|b|b|b| | | | | | | | | | |
              //                ^ ^ ^
              //                | | |
              //                energy to accumulate
              //
              // energy (a) to record after MaxEOR w/ no overlap into
              // [MinSOR,MaxEOR]
              //  0                   1                   2
              //  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0
              // | | | | | | | |x|x|x|x|x|x|x|x| | | | | | |
              // | | | | | | | | | | | | |a|a|a|a|a| | | | |
              //                          ^ ^ ^
              //                          | | |
              //                          energy to accumulate

              // accumulate bins up to and including maxEndOfReceptionBin
              if(totalSubBandBins_ > 1)
                {
                  for(const auto & binPowerApply :
                        binPowerApplyMapIter->second)
                    {
                      auto & start = std::get<0>(binPowerApply);
                      auto & end = std::get<1>(binPowerApply);
                      auto & dMultipler = std::get<2>(binPowerApply);

                      wheel_.add((startIndex + beforeMinSORBinDurationCount) % totalWheelBins_,
                                 withinMinSORMaxEORBinCount,
                                 dRxPower * dMultipler,
                                 start,
                                 end - start + 1);
                    }
                }
              else
                {
                  wheel_.add((startIndex + beforeMinSORBinDurationCount) % totalWheelBins_,
                             withinMinSORMaxEORBinCount,
                             dRxPower,
                             subBandBinStart,
                             subBandBins);
                }
            }
          else
            {
              // no energy within [MinSOR,MaxEOR]

              // current wheel state. (x) indicates valid energy
              //
              // max EOR ---------------------
              // min SOR -------              |
              //                |             |
              //                v             v
              //  0                   1                   2
              //  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0
              // | | | | | | | |x|x|x|x|x|x|x|x| | | | | | |
              //
              // energy (b) to record before MinSOR w/ no overlap into
              // [MinSOR,MaxEOR].
              //
              //  0                   1                   2
              //  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0
              // | |b|b|b| | | |x|x|x|x|x|x|x|x| | | | | | |
              //          ^ ^ ^
              //          | | |
              //          gap to clear
              //
              // energy (a) to record after MaxEOR w/ no overlap into
              // [MinSOR,MaxEOR]
              //
              //  0                   1                   2
              //  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0
              // | | | | | | | |x|x|x|x|x|x|x|x| | |a|a|a| |
              //                                ^ ^
              //                                | |
              //                                gap to clear

              // clear any gaps
              if(beforeMinSORBinDurationCount)
                {
                  wheel_.set((startIndex + beforeMinSORBinDurationCount) % totalWheelBins_,
                             minStartOfReceptionBin_ -
                             (startOfReceptionBin + beforeMinSORBinDurationCount),
                             0,
                             0,
                             totalSubBandBins_);
                }

              if(afterMaxEORBinDurationCount)
                {
                  wheel_.set((maxEndOfReceptionBin_ + 1) % totalWheelBins_,
                             startOfReceptionBin - maxEndOfReceptionBin_ - 1,
                             0,
                             0,
                             totalSubBandBins_);

                }
            }

          if(beforeMinSORBinDurationCount)
            {
              minStartOfReceptionBin_ = startOfReceptionBin;
            }

          if(afterMaxEORBinDurationCount)
            {
              maxEndOfReceptionBin_ = endOfReceptionBin;
            }
        }

      if(!bIsMore)
        {
          // update the max EOR bin for all the transmitters
          for(const auto & transmitter : transmitters)
            {
              nemAntennaIndexEORBinMap_[transmitter][txAntennaIndex] = endOfReceptionBin;
            }
        }
    }

  return std::make_tuple(startOfReception,endOfReception);
}


std::pair<std::vector<double>, EMANE::TimePoint>
EMANE::NoiseRecorder::get(const TimePoint & now,
                          const Microseconds & duration,
                          const TimePoint & startTime)
{
  auto nowBin = timepointToBin(now,true);

  auto minStartOfWindowTime =
    TimePoint(Microseconds{nowBin - totalWindowBins_ + 1} * binSizeMicroseconds_);

  auto validStartTime = startTime;

  auto validDuration = duration;

  // no start time given, use minStartOfWindowTime
  if(validStartTime == TimePoint::min())
    {
      validStartTime = minStartOfWindowTime;
    }
  else if(validStartTime < minStartOfWindowTime)
    {
      throw makeException<SpectrumServiceException>("window start time too far in the past");
    }
  else if(validStartTime > now)
    {
      throw makeException<SpectrumServiceException>("window start time in the future");
    }

  auto startTimeBin = timepointToBin(validStartTime);

  auto endTime = now;

  if(validDuration != Microseconds::zero())
    {
      endTime = validStartTime + validDuration;

      if(endTime > now)
        {
          throw makeException<SpectrumServiceException>("window end time in the future");
        }
    }

  auto endTimeBin = timepointToBin(endTime,true);

  auto durationBinCount = endTimeBin - startTimeBin + 1;

  std::vector<double> window;

  window.reserve(durationBinCount * totalSubBandBins_);

  // if a startTime was specified that bin time equates window entry 0
  auto startOfWindowTime =
    TimePoint(Microseconds{startTimeBin} * binSizeMicroseconds_);

  if(startOfWindowTime >= minStartOfWindowTime)
    {
      if(maxEndOfReceptionBin_ && minStartOfReceptionBin_)
        {
          try
            {
              Microseconds::rep beforeDurationCount{};
              Microseconds::rep afterDurationCount{};

              if(endTimeBin > maxEndOfReceptionBin_)
                {
                  afterDurationCount =
                    std::min(endTimeBin - maxEndOfReceptionBin_,durationBinCount);
                }

              if(startTimeBin < minStartOfReceptionBin_)
                {
                  beforeDurationCount =
                    std::min(minStartOfReceptionBin_ - startTimeBin,durationBinCount);
                }

              auto remainderBinCount =
                durationBinCount - (beforeDurationCount + afterDurationCount);

              if(remainderBinCount)
                {
                  window = wheel_.get((endTimeBin - afterDurationCount) % totalWheelBins_,remainderBinCount);
                }
              window.insert(window.begin(),beforeDurationCount * totalSubBandBins_,0);
              window.insert(window.end(),afterDurationCount * totalSubBandBins_,0);
            }
          catch(Wheel<double>::IndexError &)
            {
              throw makeException<SpectrumServiceException>("window internal access error");
            }
        }
      else
        {
          // fill in window with 0
          window.insert(window.begin(),durationBinCount * totalSubBandBins_,0);
        }
    }
  else
    {
      throw makeException<SpectrumServiceException>("window start time invalid");
    }

  return std::make_pair(std::move(window),startOfWindowTime);
}

std::vector<double> EMANE::NoiseRecorder::dump() const
{
  return wheel_.dump();
}

EMANE::Microseconds::rep EMANE::NoiseRecorder::timepointToBin(const TimePoint & tp,bool bAdjust)
{
  auto count =
    std::chrono::duration_cast<Microseconds>(tp.time_since_epoch()).count();

  // times that fall on a bin boundary belong to the previous bin
  // (count % binSizeMicroseconds_ == 0) will evaluate to 0 or 1
  return count == 0 ? 0 : count / binSizeMicroseconds_ - (bAdjust && (count % binSizeMicroseconds_ == 0));
}

std::size_t EMANE::NoiseRecorder::getSubBandBinCount() const
{
  return totalSubBandBins_;
}
