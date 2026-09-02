/*
 * Copyright (c) 2013,2021,2026 - Adjacent Link LLC, Bridgewater,
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

#ifndef EMANEPHYGAINMANAGER_HEADER_
#define EMANEPHYGAINMANAGER_HEADER_

#include "antennapattern.h"
#include "positionneu.h"
#include "locationinfo.h"
#include "antennamanager.h"
#include "emane/types.h"
#include "emane/antenna.h"
#include "emane/events/antennaprofile.h"

#include <optional>

namespace EMANE
{
  class GainManager
  {
  public:
    GainManager(NEMId nemId,
                AntennaIndex rxAntennaIndex,
                AntennaManager & antennaManager,
                bool bHorizonCheck);

    enum class GainStatus {SUCCESS = 0,
                           ERROR_LOCATIONINFO,
                           ERROR_PROFILEINFO,
                           ERROR_HORIZON,
                           ERROR_ANTENNA_INDEX};

    struct GainEntry
    {
      double dRemoteAntennaGaindBi_;
      double dRemoteDirectionAzimuthDegrees_;
      double dRemoteDirectionElevationDegrees_;
      double dRemoteDirectionDistanceMeters_;
      bool bRemoteDirectionVerticallyAligned_;
      double dRemoteLookupAzimuthDegrees_;
      double dRemoteLookupElevationDegrees_;

      double dLocalAntennaGaindBi_;
      double dLocalDirectionAzimuthDegrees_;
      double dLocalDirectionElevationDegrees_;
      double dLocalDirectionDistanceMeters_;
      bool bLocalDirectionVerticallyAligned_;
      double dLocalLookupAzimuthDegrees_;
      double dLocalLookupElevationDegrees_;

      GainEntry() = default;

      GainEntry(double dRemoteAntennaGaindBi,
                double dRemoteDirectionAzimuthDegrees,
                double dRemoteDirectionElevationDegrees,
                double dRemoteDirectionDistanceMeters,
                bool bRemoteDirectionVerticallyAligned,
                double dRemoteLookupAzimuthDegrees,
                double dRemoteLookupElevationDegrees,
                double dLocalAntennaGaindBi,
                double dLocalDirectionAzimuthDegrees,
                double dLocalDirectionElevationDegrees,
                double dLocalDirectionDistanceMeters,
                bool bLocalDirectionVerticallyAligned,
                double dLocalLookupAzimuthDegrees,
                double dLocalLookupElevationDegrees):
        dRemoteAntennaGaindBi_{dRemoteAntennaGaindBi},
        dRemoteDirectionAzimuthDegrees_{dRemoteDirectionAzimuthDegrees},
        dRemoteDirectionElevationDegrees_{dRemoteDirectionElevationDegrees},
        dRemoteDirectionDistanceMeters_{dRemoteDirectionDistanceMeters},
        bRemoteDirectionVerticallyAligned_{bRemoteDirectionVerticallyAligned},
        dRemoteLookupAzimuthDegrees_{dRemoteLookupAzimuthDegrees},
        dRemoteLookupElevationDegrees_{dRemoteLookupElevationDegrees},
        dLocalAntennaGaindBi_{dLocalAntennaGaindBi},
        dLocalDirectionAzimuthDegrees_{dLocalDirectionAzimuthDegrees},
        dLocalDirectionElevationDegrees_{dLocalDirectionElevationDegrees},
        dLocalDirectionDistanceMeters_{dLocalDirectionDistanceMeters},
        bLocalDirectionVerticallyAligned_{bLocalDirectionVerticallyAligned},
        dLocalLookupAzimuthDegrees_{dLocalLookupAzimuthDegrees},
        dLocalLookupElevationDegrees_{dLocalLookupElevationDegrees}{}
    };

    struct GainInfo
    {
      GainEntry entry_;
      GainStatus status_;
      bool bCacheHit_;

      GainInfo(const GainEntry & entry,
               bool bCacheHit):
        entry_{entry},
        status_{GainStatus::SUCCESS},
        bCacheHit_{bCacheHit}{}

      GainInfo(GainStatus status):
        status_{status},
        bCacheHit_{false}{};
    };

    struct GainCacheEntry
    {
      GainEntry entry_;
      std::uint64_t u64TxAntennaInfoSequence_;
      std::uint64_t u64LocationPairSequence_;

      GainCacheEntry() = default;

      GainCacheEntry(const GainEntry & entry,
                     std::uint64_t u64TxAntennaInfoSequence,
                     std::uint64_t u64LocationPairSequence):
        entry_{entry},
        u64TxAntennaInfoSequence_{u64TxAntennaInfoSequence},
        u64LocationPairSequence_{u64LocationPairSequence}{}
    };

    GainInfo determineGain(NEMId transmitterId,
                           AntennaIndex txAntennaIndex,
                           const LocationInfo & locationPairInfo);

  private:
    using AntennaIndexMap = std::map<std::uint16_t,
                                     Antenna>;

    using AntennaStore = std::map<NEMId,
                                  AntennaIndexMap>;
    NEMId id_;
    AntennaIndex rxAntennaIndex_;
    AntennaManager & antennaManager_;
    AntennaStore antennaStore_;
    std::uint64_t u64AntennaUpdateSequence_;
    bool bHorizonCheck_;

    struct AntennaPatternInfo
    {
      AntennaPattern * pPattern_;
      AntennaPattern * pBlockage_;
      PositionNEU placement_;

      AntennaPatternInfo();

      AntennaPatternInfo(AntennaPattern * pPattern,
                         AntennaPattern * pBlockage,
                         const PositionNEU & placement);
    };

    AntennaPatternInfo localAntennaPatternInfo_;

    using Cache = std::map<NEMId, // Tx NEM Id
                           std::map<AntennaIndex, // Tx Antenna Index
                                    GainCacheEntry>>;

    Cache gainCache_;

    std::optional<GainEntry>
    getGainCache(NEMId transmitterId,
                 const AntennaManager::AntennaInfo & txAntennaInfo,
                 const AntennaManager::AntennaInfo & rxAntennaInfo,
                 const LocationInfo & locationPairInfo);

    void setGainCache(NEMId transmitterId,
                      const AntennaManager::AntennaInfo & txAntennaInfo,
                      const LocationInfo & locationPairInfo,
                      const GainEntry & entry);
  };
}

#endif // EMANEPHYGAINMANAGER_HEADER_
