/*
 * Copyright (c) 2020-2021 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANERECEIVEPROCESSOR_HEADER_
#define EMANERECEIVEPROCESSOR_HEADER_

#include "locationmanager.h"
#include "propagationmodelalgorithm.h"
#include "gainmanager.h"
#include "fadingmanager.h"
#include "spectrummonitor.h"
#include "antennamanager.h"

#include "emane/commonphyheader.h"
#include "emane/controls/antennareceiveinfo.h"
#include "emane/controls/antennaselfinterference.h"
#include "emane/controls/dopplershifts.h"

namespace EMANE
{
  class ReceiveProcessor
  {
  public:
    ReceiveProcessor(NEMId id,
                     std::uint16_t u16SubId,
                     AntennaIndex rxAntennaIndex,
                     AntennaManager & antennaManager,
                     SpectrumMonitor * pSpectrumMonitor,
                     PropagationModelAlgorithm * pPropagationModelAlgorithm,
                     FadingAlgorithmStore && fadingAlgorithmStore,
                     bool bPopulateReceivePowerMap,
                     bool bPopulateObservedPowerMap,
                     bool bDopperShift);

    struct ProcessResult
    {
      enum class Status
        {
          DROP_UNKNOWN,
          DROP_CODE_ANTENNA_FREQ_INDEX,
          DROP_CODE_FADINGMANAGER_LOCATION,
          DROP_CODE_FADINGMANAGER_ALGORITHM,
          DROP_CODE_FADINGMANAGER_SELECTION,
          DROP_CODE_GAINMANAGER_LOCATION,
          DROP_CODE_GAINMANAGER_ANTENNAPROFILE,
          DROP_CODE_GAINMANAGER_HORIZON,
          DROP_CODE_GAINMANAGER_ANTENNA_INDEX,
          DROP_CODE_PROPAGATIONMODEL,
          DROP_CODE_SPECTRUM_CLAMP,
          DROP_CODE_NOT_FOI,
          DROP_CODE_OUT_OF_BAND,
          SUCCESS
        };

      Status status_{Status::DROP_UNKNOWN};
      TimePoint mimoSoT_{};
      Microseconds mimoPropagationDelay_{};
      Controls::AntennaReceiveInfos antennaReceiveInfos_{};
      bool bGainCacheHit_{};
      std::map<std::tuple<NEMId, // src
                          AntennaIndex, // rx antena
                          AntennaIndex, // tx antenna
                          std::uint64_t>, // freq
               std::tuple<double, // rx power dBm
                          double, // tx gain dBi
                          double, // rx gain dBi
                          double, // tx power
                          double, // pathloss
                          double>> // doppler
      receivePowerMap_{};
      std::map<std::tuple<NEMId, // src
                          AntennaIndex, // rx antena
                          AntennaIndex, // tx antenna
                          std::uint64_t>, // freq
               std::tuple<SpectralMaskIndex,
                          double>> // rx power observed dBm
      observedPowerMap_{};
      Controls::DopplerShifts dopplerShifts_{};

      ProcessResult() = default;

      ProcessResult(ProcessResult && rhs):
        status_{rhs.status_},
        mimoSoT_{rhs.mimoSoT_},
        mimoPropagationDelay_{std::move(rhs.mimoPropagationDelay_)},
        antennaReceiveInfos_{std::move(rhs.antennaReceiveInfos_)},
        bGainCacheHit_{rhs.bGainCacheHit_},
        receivePowerMap_{std::move(rhs.receivePowerMap_)},
        observedPowerMap_{std::move(rhs.observedPowerMap_)},
        dopplerShifts_{std::move(rhs.dopplerShifts_)}{}

    };

    ProcessResult process(const TimePoint & now,
                          const CommonPHYHeader & commonPHYHeader,
                          const std::vector<std::pair<LocationInfo,bool>> & locationInfos,
                          const  std::vector<std::pair<FadingInfo,bool>> & fadingSelection,
                          bool bInBand);


    struct ProcessSelfInterferenceResult
    {
      enum class Status
        {
          ERROR_UNKNOWN,
          ERROR_ANTENNA_FREQ_INDEX,
          ERROR_MISSING_POWER_VALUES,
          SUCCESS
        };

      Status status_{Status::ERROR_UNKNOWN};
    };

    ProcessSelfInterferenceResult
    processSelfInterference(const TimePoint & now,
                            const TimePoint & txTime,
                            const FrequencyGroups & segments,
                            std::uint64_t u64SegmentBandwidthHz,
                            const Controls::AntennaSelfInterferences & antennaInterferences,
                            const std::pair<FilterData,bool> & optionalFilterData);

  private:
    NEMId id_;
    std::uint16_t u16SubId_;
    AntennaIndex rxAntennaIndex_;
    GainManager gainManager_;
    SpectrumMonitor * pSpectrumMonitor_;
    PropagationModelAlgorithm * pPropagationModelAlgorithm_;
    FadingAlgorithmStore fadingAlgorithmStore_;
    bool bPopulateReceivePowerMap_;
    bool bPopulateObservedPowerMap_;
    std::uint64_t u64SpectrumMonitorUpdateSequence_;
    bool bDopplerShift_;
  };
}



#endif //EMANERECEIVEPROCESSOR_HEADER_
