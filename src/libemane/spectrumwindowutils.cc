/*
 * Copyright (c) 2014,2021 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "emane/utils/spectrumwindowutils.h"
#include "emane/utils/conversionutils.h"

#include "emane/spectrumserviceexception.h"

#include <algorithm>

std::pair<double,bool> EMANE::Utils::maxBinNoiseFloorRange(const SpectrumWindow & window,
                                                           double dRxPowerdBm,
                                                           const TimePoint & startTime,
                                                           const TimePoint & endTime)
{
  const auto & noiseData = std::get<0>(window);
  const TimePoint & windowStartTime = std::get<1>(window);
  const Microseconds & binSize =  std::get<2>(window);
  const double & dRxSensitivityMilliWatt = std::get<3>(window);
  const bool & bSignalInNoise{std::get<4>(window)};

  Microseconds::rep windowStartBin{timepointToAbsoluteBin(windowStartTime,binSize,false)};

  std::size_t startIndex{};
  std::size_t endIndex{noiseData.size()-1};

  if(startTime < windowStartTime)
    {
      throw makeException<SpectrumServiceException>("max bin start time < window start time");
    }
  else
    {
      startIndex = timepointToAbsoluteBin(startTime,binSize,false) - windowStartBin;
    }

  if(endTime != TimePoint::min())
    {
      if(endTime < startTime)
        {
          throw makeException<SpectrumServiceException>("max bin end time < max bin start time");
        }
      else
        {
          endIndex =  timepointToAbsoluteBin(endTime,binSize,true) - windowStartBin;
        }
    }

  return maxBinNoiseFloor(noiseData,dRxSensitivityMilliWatt,dRxPowerdBm,bSignalInNoise,startIndex,endIndex);
}

std::pair<double,bool> EMANE::Utils::maxBinNoiseFloor(const SpectrumWindow & window,
                                                      double dRxPowerdBm,
                                                      const TimePoint & startTime)
{
  const auto & noiseData = std::get<0>(window);
  const TimePoint & windowStartTime = std::get<1>(window);
  const Microseconds & binSize =  std::get<2>(window);
  const double & dRxSensitivityMilliWatt = std::get<3>(window);
  const bool & bSignalInNoise{std::get<4>(window)};

  Microseconds::rep windowStartBin{timepointToAbsoluteBin(windowStartTime,binSize,false)};

  std::size_t startIndex{};
  std::size_t endIndex{noiseData.size()-1};

  if(startTime != TimePoint::min())
    {
      if(startTime < windowStartTime)
        {
          throw makeException<SpectrumServiceException>("max bin start time %13.6f < window start time %13.6f",
                                                        std::chrono::duration_cast<DoubleSeconds>(startTime.time_since_epoch()).count(),
                                                        std::chrono::duration_cast<DoubleSeconds>(windowStartTime.time_since_epoch()).count());
        }
      else
        {
          startIndex = timepointToAbsoluteBin(startTime,binSize,false) - windowStartBin;
        }
    }

  return maxBinNoiseFloor(noiseData,dRxSensitivityMilliWatt,dRxPowerdBm,bSignalInNoise,startIndex,endIndex);
}

std::pair<double,bool> EMANE::Utils::maxBinNoiseFloor(const std::vector<double> & noiseData,
                                                      double dRxSensitivityMilliWatt,
                                                      double dRxPowerdBm,
                                                      bool bSignalInNoise,
                                                      std::size_t startBin,
                                                      std::size_t endBin)
{
  if(endBin < startBin)
    {
      throw makeException<SpectrumServiceException>("max bin end index %zu < max bin start index %zu,"
                                                    " num bins %zu",
                                                    startBin,
                                                    endBin,
                                                    noiseData.size());
    }
  else if(endBin >= noiseData.size() || startBin >= noiseData.size())
    {
      throw makeException<SpectrumServiceException>("bin index out of range, start index %zu,"
                                                    " end index %zu, num bins %zu",
                                                    startBin,
                                                    endBin,
                                                    noiseData.size());
    }

  double dNoiseFloorMilliWatt{*std::max_element(&noiseData[startBin],&noiseData[endBin]+1)};

  if(bSignalInNoise)
    {
      dNoiseFloorMilliWatt -= Utils::DB_TO_MILLIWATT(dRxPowerdBm);
    }

  if(dNoiseFloorMilliWatt < dRxSensitivityMilliWatt)
    {
      dNoiseFloorMilliWatt = dRxSensitivityMilliWatt;
    }

  return {Utils::MILLIWATT_TO_DB(dNoiseFloorMilliWatt),bSignalInNoise};
}

EMANE::Microseconds::rep EMANE::Utils::timepointToAbsoluteBin(const TimePoint & tp,
                                                              const Microseconds & binSize,
                                                              bool bAdjust)
{
  auto count =
    std::chrono::duration_cast<Microseconds>(tp.time_since_epoch()).count();

  // times that fall on a bin boundary belong to the previous bin
  // (count % binSizeMicroseconds_ == 0) will evaluate to 0 or 1
  return count == 0 ? 0 : count / binSize.count() - (bAdjust && (count % binSize.count() == 0));
}

EMANE::Utils::SpectrumCompressedRepresentation
EMANE::Utils::spectrumCompress(const std::vector<double> & window)
{
  SpectrumCompressedRepresentation ret;

  double dPrevious{};

  std::size_t i{};

  for(const auto & entry : window)
    {
      if(dPrevious != entry || (ret.empty() && entry != 0))
        {
          ret.push_back(std::make_pair(i,entry));
          dPrevious = entry;
        }

      ++i;
    }

  return ret;
}

EMANE::Utils::SpectrumSubBandCompressedRepresentation
EMANE::Utils::spectrumSubBandCompress(const std::vector<double> & window,
                                      std::size_t subBandBinCount)
{
  SpectrumSubBandCompressedRepresentation ret;

  std::vector<double> dPrevious(subBandBinCount,0);

  bool bWrite{};
  std::size_t i{};
  std::size_t subBandBinIndex{};

  // iterate over entire window
  for(const auto & entry : window)
    {
      // compare the subband bin to the previous entry
      if(dPrevious[subBandBinIndex] != entry || (ret.empty() && entry != 0))
        {
          // any bin value difference will cause a compress entry for every bin
          bWrite = true;
          dPrevious[subBandBinIndex] = entry;
        }

      ++subBandBinIndex;

      // once we have seen every bin in the sub band bins for a time
      // entry, see if we need to publish due to change
      if(subBandBinIndex == subBandBinCount)
        {
          // reset the index
          subBandBinIndex = 0;

          if(bWrite)
            {
              ret.push_back(std::make_pair(i,dPrevious));
              bWrite = false;
            }

          ++i;
        }
    }

  return ret;
}
