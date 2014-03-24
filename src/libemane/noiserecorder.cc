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

#include "noiserecorder.h"
#include "emane/spectrumserviceexception.h"

EMANE::NoiseRecorder::NoiseRecorder(const Microseconds & bin,
                                    const Microseconds & maxOffset,
                                    const Microseconds & maxPropagation,
                                    const Microseconds & maxDuration,
                                    double dRxSensitivityMilliWatt):
  totalWindowBins_{maxDuration/bin},
  totalWheelBins_{(maxOffset + maxPropagation + 2 * maxDuration)/bin},
  binSizeMicroseconds_{bin.count()},
  wheel_{static_cast<std::size_t>(totalWheelBins_)},
  dRxSensitivityMilliWatt_{dRxSensitivityMilliWatt},
  maxEndOfReceptionBin_{},
  minStartOfReceptionBin_{}
{}

std::pair<EMANE::TimePoint,EMANE::TimePoint>
EMANE::NoiseRecorder::update(const TimePoint & now,
                             const TimePoint & txTime,
                             const Microseconds & offset,
                             const Microseconds & propagation,
                             const Microseconds & duration,
                             double dRxPower,
                             const std::vector<NEMId> & transmitters)
{
  auto startOfReception = txTime + offset + propagation;

  auto endOfReception = startOfReception + duration;

  auto reportedStartOfReceptionBin = timepointToBin(startOfReception);

  auto endOfReceptionBin = timepointToBin(endOfReception,true);

  Microseconds::rep startOfReceptionBin{reportedStartOfReceptionBin};

  Microseconds::rep maxEoRBin{};
  
  // Determine the last EoR bin for the transmitter - we only 
  // allow one bin noise entry per transmitter. For multiple 
  // transmitters use the max EoR
  for(const auto & transmitter : transmitters)
    {
      const auto iter = nemEoRBinMap_.find(transmitter);

      if(iter != nemEoRBinMap_.end())
        {
          maxEoRBin = std::max(iter->second,maxEoRBin);
        }
    }

  // adjust the SoR bin to be the next past max EoR bin
  // if necessary
  if(maxEoRBin >= reportedStartOfReceptionBin)
    {
      startOfReceptionBin = maxEoRBin + 1;
    }

  // sanity check after any possible adjustments
  if(startOfReceptionBin <= endOfReceptionBin)
    {
      auto nowBin = timepointToBin(now);

      auto durationBinCount = endOfReceptionBin - startOfReceptionBin + 1;
      
      auto startIndex = startOfReceptionBin % totalWheelBins_;
      
      if(nowBin - maxEndOfReceptionBin_ > totalWheelBins_)
        {
          // reset the max end of reception bin
          //  the last bin we received on
          maxEndOfReceptionBin_ = 0;


          // reset the min start of reception bin
          // the first bin we received on
          minStartOfReceptionBin_ = 0;
        }

      if(!maxEndOfReceptionBin_ && !minStartOfReceptionBin_)
        {
          // we can fill the entire duration
          wheel_.set(startIndex,durationBinCount,dRxPower);
      
          minStartOfReceptionBin_ = startOfReceptionBin;

          maxEndOfReceptionBin_ = endOfReceptionBin;
        }
      else
        {
          Microseconds::rep beforeDurationCount{};
          Microseconds::rep afterDurationCount{};
      
          // set any values before the minStartOfReceptionBin
          if(startOfReceptionBin < minStartOfReceptionBin_)
            {
              beforeDurationCount =
                std::min(minStartOfReceptionBin_ - startOfReceptionBin,durationBinCount);

              wheel_.set(startIndex,beforeDurationCount,dRxPower);
            }

          // set any values after the maxEndOfReceptionBin
          if(endOfReceptionBin > maxEndOfReceptionBin_)
            {
              afterDurationCount =
                std::min(endOfReceptionBin - maxEndOfReceptionBin_,durationBinCount);
          
              wheel_.set((startIndex + durationBinCount - afterDurationCount) % totalWheelBins_, afterDurationCount,dRxPower);
            }
      
          auto remainderBinCount = durationBinCount - (beforeDurationCount + afterDurationCount);

          if(remainderBinCount)
            {
              // accumulate bins up to and including maxEndOfReceptionBin
              wheel_.add((startIndex + beforeDurationCount) % totalWheelBins_,
                         remainderBinCount,
                         dRxPower);
            }
          else
            {
              // clear any gaps
              if(beforeDurationCount)
                {
                  wheel_.set((startIndex + durationBinCount) % totalWheelBins_,
                             minStartOfReceptionBin_ - endOfReceptionBin - 1,
                             0);
                }
              else
                {
                  wheel_.set((maxEndOfReceptionBin_ + 1) % totalWheelBins_,
                             startOfReceptionBin - maxEndOfReceptionBin_ - 1,
                             0);
                }
            }

          if(beforeDurationCount)
            {
              minStartOfReceptionBin_ = startOfReceptionBin;
            }

          if(afterDurationCount)
            {
              maxEndOfReceptionBin_ = endOfReceptionBin;
            }
        }

      // update the max EOR bin for all the transmitters
      for(const auto & transmitter : transmitters)
        {
          nemEoRBinMap_[transmitter] = endOfReceptionBin;
        }
    }
  //return {TimePoint{Microseconds{reportedStartOfReceptionBin} * binSizeMicroseconds_},
  //TimePoint{Microseconds{endOfReceptionBin} * binSizeMicroseconds_}};
  return {startOfReception,endOfReception};
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

  window.reserve(durationBinCount);

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

              window.insert(window.begin(),beforeDurationCount,0);
              window.insert(window.end(),afterDurationCount,0);
            }
          catch(Wheel<double>::IndexError &)
            {
              throw makeException<SpectrumServiceException>("window internal access error");
            }
        }
      else
        {
          // fill in window with 0
          window.insert(window.begin(),durationBinCount,0);
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
