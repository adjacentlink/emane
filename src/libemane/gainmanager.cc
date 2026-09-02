/*
 * Copyright (c) 2013-2014,2021,2026 - Adjacent Link LLC, Bridgewater,
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

#include "gainmanager.h"
#include "antennaprofilemanifest.h"
#include "positionutils.h"
#include "antennaprofileexception.h"
#include "logservice.h"
#include "locationinfoformatter.h"
#include "positionneuformatter.h"

EMANE::GainManager::AntennaPatternInfo::AntennaPatternInfo():
  pPattern_{},
  pBlockage_{},
  placement_{}{}

EMANE::GainManager::AntennaPatternInfo::AntennaPatternInfo(AntennaPattern * pPattern,
                                                           AntennaPattern * pBlockage,
                                                           const PositionNEU & placement):
  pPattern_{pPattern},
  pBlockage_{pBlockage},
  placement_{placement}{}

EMANE::GainManager::GainManager(NEMId id,
                                AntennaIndex rxAntennaIndex,
                                AntennaManager & antennaManager,
                                bool bHorizonCheck):
  id_{id},
  rxAntennaIndex_{rxAntennaIndex},
  antennaManager_(antennaManager),
  u64AntennaUpdateSequence_{},
  bHorizonCheck_{bHorizonCheck}{}

void EMANE::GainManager::setGainCache(NEMId transmitterId,
                                      const AntennaManager::AntennaInfo & txAntennaInfo,
                                      const LocationInfo & locationPairInfo,
                                      const GainEntry & entry)
{
  gainCache_[transmitterId][txAntennaInfo.antenna_.getIndex()] =
    GainCacheEntry{entry,
                   txAntennaInfo.u64UpdateSequence_,
                   locationPairInfo.getSequenceNumber()};
}

std::optional<EMANE::GainManager::GainEntry>
EMANE::GainManager::getGainCache(NEMId transmitterId,
                                 const AntennaManager::AntennaInfo & txAntennaInfo,
                                 const AntennaManager::AntennaInfo & rxAntennaInfo,
                                 const LocationInfo & locationPairInfo)
{
  if(rxAntennaInfo.u64UpdateSequence_ != u64AntennaUpdateSequence_)
    {
      gainCache_.clear();

      u64AntennaUpdateSequence_ = rxAntennaInfo.u64UpdateSequence_;
    }
  else
    {
      auto txNEMIdIter = gainCache_.find(transmitterId);

      if(txNEMIdIter != gainCache_.end())
        {
          auto antennaIndexIter = txNEMIdIter->second.find(txAntennaInfo.antenna_.getIndex());

          if(antennaIndexIter != txNEMIdIter->second.end())
            {
              const auto & cache = antennaIndexIter->second;

              if(cache.u64TxAntennaInfoSequence_ == txAntennaInfo.u64UpdateSequence_ &&
                 cache.u64LocationPairSequence_ == locationPairInfo.getSequenceNumber())
                {
                  return cache.entry_;
                }
            }
        }
    }

  return {};
}

EMANE::GainManager::GainInfo
EMANE::GainManager::determineGain(NEMId transmitterId,
                                  AntennaIndex txAntennaIndex,
                                  const LocationInfo & locationPairInfo)
{
  const auto & remoteAntennaInfo = antennaManager_.getAntennaInfo(transmitterId,
                                                                  txAntennaIndex);

  const auto & localAntennaInfo = antennaManager_.getAntennaInfo(id_,
                                                                 rxAntennaIndex_);

  if(!remoteAntennaInfo.second || !localAntennaInfo.second)
    {
      return GainInfo{GainStatus::ERROR_PROFILEINFO};
    }

  auto cacheEntry = getGainCache(transmitterId,
                                 remoteAntennaInfo.first,
                                 localAntennaInfo.first,
                                 locationPairInfo);

  if(cacheEntry.has_value())
    {
      return GainInfo{*cacheEntry,true};
    }

  const auto & remoteAntenna = remoteAntennaInfo.first.antenna_;

  const auto & localAntenna = localAntennaInfo.first.antenna_;

  const static double dEpsilon{1e-3};

  double dRemoteAntennaGaindBi{};
  double dRemoteDirectionAzimuthDegrees{};
  double dRemoteDirectionElevationDegrees{};
  double dRemoteDirectionDistanceMeters{};
  double dRemoteLookupAzimuthDegrees{};
  double dRemoteLookupElevationDegrees{};
  bool bRemoteDirectionVerticallyAligned{};

  if(!remoteAntenna.isIdealOmni())
    {
      AntennaStore::const_iterator remoteAntennaStoreIter;

      if(!locationPairInfo.isValid())
        {
          return GainInfo{GainStatus::ERROR_LOCATIONINFO};
        }

      auto remotePointing = remoteAntenna.getPointing();

      // we have the profile info w/ pattern info
      if(remotePointing.second && remoteAntennaInfo.first.pPattern_)
        {
          // calculate the direction: azimuth, elvation and distance
          std::tie(dRemoteDirectionAzimuthDegrees,
                   dRemoteDirectionElevationDegrees,
                   dRemoteDirectionDistanceMeters,
                   bRemoteDirectionVerticallyAligned) =
            Utils::calculateDirection(locationPairInfo.getRemotePOV(),
                                      remoteAntennaInfo.first.placement_,
                                      locationPairInfo.getLocalPOV(),
                                      localAntennaInfo.first.placement_);

          // adjust the direction azimuth and elevation based on the antenna pointing azimuth and elvation
          std::tie(dRemoteLookupAzimuthDegrees,
                   dRemoteLookupElevationDegrees) =
            Utils::calculateLookupAngles(dRemoteDirectionAzimuthDegrees,
                                         remotePointing.first.getAzimuthDegrees(),
                                         dRemoteDirectionElevationDegrees,
                                         remotePointing.first.getElevationDegrees());

          if(bRemoteDirectionVerticallyAligned && std::fabs(dRemoteLookupElevationDegrees) < dEpsilon)
            {
              dRemoteLookupAzimuthDegrees = 0;
            }

          double dTxAntennaGaindBi{remoteAntennaInfo.first.pPattern_->getGain(std::round(dRemoteLookupAzimuthDegrees),
                                                                              std::round(dRemoteLookupElevationDegrees))};

          // get the blockage, if specified
          //  Note: no adjustment is necessary to the direction azimuth and elvation

          double dTxAntennaBlockagedBi{remoteAntennaInfo.first.pBlockage_ ?
                                       remoteAntennaInfo.first.pBlockage_->
                                       getGain(std::round(dRemoteLookupAzimuthDegrees),
                                               std::round(dRemoteDirectionElevationDegrees)) :
                                       0};


          LOGGER_VERBOSE_LOGGING_FN_VARGS(*LogServiceSingleton::instance(),
                                          DEBUG_LEVEL,
                                          [this,&remoteAntennaInfo,&localAntennaInfo]()
                                          {
                                            Strings strings;

                                            strings.push_back("remote antenna");
                                            strings.splice(strings.end(),PositionNEUFormatter(remoteAntennaInfo.first.placement_)());

                                            strings.push_back("local antenna");
                                            strings.splice(strings.end(),PositionNEUFormatter(localAntennaInfo.first.placement_)());

                                            return strings;
                                          },
                                          "PHYI %03hu GainManager::%s remote calc tx antenna gain: %lf tx antenna"
                                          " blockage: %lf direction az: %lf el: %lf dist: %lf remote antenna az: %lf el: %lf"
                                          " lookup bearing: %lf lookup el: %lf",
                                          id_,
                                          __func__,
                                          dTxAntennaGaindBi,
                                          dTxAntennaBlockagedBi,
                                          dRemoteDirectionAzimuthDegrees,
                                          dRemoteDirectionElevationDegrees,
                                          dRemoteDirectionDistanceMeters,
                                          remotePointing.first.getAzimuthDegrees(),
                                          remotePointing.first.getElevationDegrees(),
                                          dRemoteLookupAzimuthDegrees,
                                          dRemoteLookupElevationDegrees);

          dRemoteAntennaGaindBi = dTxAntennaGaindBi + dTxAntennaBlockagedBi;

        }
      else
        {
          // profile info is missing
          return GainInfo{GainStatus::ERROR_PROFILEINFO};
        }
    }
  else
    {
      dRemoteAntennaGaindBi = remoteAntenna.getFixedGaindBi().first;
    }

  const auto & localPointing = localAntenna.getPointing();

  double dLocalAntennaGaindBi{};
  double dLocalDirectionAzimuthDegrees{};
  double dLocalDirectionElevationDegrees{};
  double dLocalDirectionDistanceMeters{};
  double dLocalLookupAzimuthDegrees{};
  double dLocalLookupElevationDegrees{};
  bool bLocalDirectionVerticallyAligned{};

  if(!localAntenna.isIdealOmni())
    {
      if(!locationPairInfo.isValid())
        {
          return GainInfo{GainStatus::ERROR_LOCATIONINFO};
        }

      // we have the profile info w/ pattern info
      if(localPointing.second && localAntennaInfo.first.pPattern_)
        {
          // calculate the direction: azimuth, elvation and distance
          std::tie(dLocalDirectionAzimuthDegrees,
                   dLocalDirectionElevationDegrees,
                   dLocalDirectionDistanceMeters,
                   bLocalDirectionVerticallyAligned) =
            Utils::calculateDirection(locationPairInfo.getLocalPOV(),
                                      localAntennaInfo.first.placement_,
                                      locationPairInfo.getRemotePOV(),
                                      remoteAntennaInfo.first.placement_);

          // adjust the direction azimuth and elevation based on the antenna pointing azimuth and elvation
          std::tie(dLocalLookupAzimuthDegrees,
                   dLocalLookupElevationDegrees) =
            Utils::calculateLookupAngles(dLocalDirectionAzimuthDegrees,
                                         localPointing.first.getAzimuthDegrees(),
                                         dLocalDirectionElevationDegrees,
                                         localPointing.first.getElevationDegrees());

          if(bLocalDirectionVerticallyAligned && std::fabs(dLocalLookupElevationDegrees) < dEpsilon)
            {
              dLocalLookupAzimuthDegrees = 0;
            }

          // get the local receiver antenna gain
          double dRxAntennaGaindBi{localAntennaInfo.first.pPattern_->getGain(std::round(dLocalLookupAzimuthDegrees),
                                                                             std::round(dLocalLookupElevationDegrees))};

          // get the blockage, if specified
          //  Note: no adjustment is necessary to the direction azimuth and elvation
          double dRxAntennaBlockagedBi{localAntennaInfo.first.pBlockage_ ?
                                       localAntennaInfo.first.pBlockage_->
                                       getGain(std::round(dLocalDirectionAzimuthDegrees),
                                               std::round(dLocalDirectionElevationDegrees)) :
                                       0};

          LOGGER_VERBOSE_LOGGING_FN_VARGS(*LogServiceSingleton::instance(),
                                          DEBUG_LEVEL,
                                          [this,&remoteAntennaInfo,&localAntennaInfo]()
                                          {
                                            Strings strings;

                                            strings.push_back("local antenna");
                                            strings.splice(strings.end(),PositionNEUFormatter(localAntennaInfo.first.placement_)());

                                            strings.push_back("remote antenna");
                                            strings.splice(strings.end(),PositionNEUFormatter(remoteAntennaInfo.first.placement_)());

                                            return strings;
                                          },
                                          "PHYI %03hu GainManager::%s local calc rx antenna gain: %lf rx antenna"
                                          " blockage: %lf direction az: %lf el: %lf dist: %lf local antenna az: %lf el: %lf"
                                          " lookup bearing: %lf lookup el: %lf",
                                          id_,
                                          __func__,
                                          dRxAntennaGaindBi,
                                          dRxAntennaBlockagedBi,
                                          dLocalDirectionAzimuthDegrees,
                                          dLocalDirectionElevationDegrees,
                                          dLocalDirectionDistanceMeters,
                                          localPointing.first.getAzimuthDegrees(),
                                          localPointing.first.getElevationDegrees(),
                                          dLocalLookupAzimuthDegrees,
                                          dLocalLookupElevationDegrees);

          dLocalAntennaGaindBi = dRxAntennaGaindBi + dRxAntennaBlockagedBi;
        }
      else
        {
          // profile info is missing
          return GainInfo{GainStatus::ERROR_PROFILEINFO};
        }
    }
  else
    {
      dLocalAntennaGaindBi = localAntenna.getFixedGaindBi().first;
    }

  const auto & localPosition = locationPairInfo.getLocalPOV().getPosition();
  const auto & remotePosition = locationPairInfo.getRemotePOV().getPosition();
  double dDistanceMeters{locationPairInfo.getDistanceMeters()};

  GainEntry gainEntry{dRemoteAntennaGaindBi,
                      dRemoteDirectionAzimuthDegrees,
                      dRemoteDirectionElevationDegrees,
                      dRemoteDirectionDistanceMeters,
                      bRemoteDirectionVerticallyAligned,
                      dRemoteLookupAzimuthDegrees,
                      dRemoteLookupElevationDegrees,
                      dLocalAntennaGaindBi,
                      dLocalDirectionAzimuthDegrees,
                      dLocalDirectionElevationDegrees,
                      dLocalDirectionDistanceMeters,
                      bLocalDirectionVerticallyAligned,
                      dLocalLookupAzimuthDegrees,
                      dLocalLookupElevationDegrees};

  // check if antennas are below the horizon
  if(bHorizonCheck_ &&
     locationPairInfo.isValid() &&
     dDistanceMeters > 10 &&
     Utils::checkHorizon(localPosition.getAltitudeMeters() +
                         localAntennaInfo.first.placement_.getUpMeters(),
                         remotePosition.getAltitudeMeters() +
                         remoteAntennaInfo.first.placement_.getUpMeters(),
                         dDistanceMeters) == false)
    {
      // below horizon
      return GainInfo{GainStatus::ERROR_HORIZON};
    }
  else
    {
      setGainCache(transmitterId,
                   remoteAntennaInfo.first,
                   locationPairInfo,
                   gainEntry);

      LOGGER_VERBOSE_LOGGING(*LogServiceSingleton::instance(),
                             DEBUG_LEVEL,
                             "PHYI %03hu GainManager::%s tx antenna index: %hu"
                             " rx antenna index: %hu tx gain: %lf rx gain: %lf",
                             id_,
                             __func__,
                             remoteAntenna.getIndex(),
                             rxAntennaIndex_,
                             dRemoteAntennaGaindBi,
                             dLocalAntennaGaindBi);
    }

  return GainInfo{gainEntry,false};
}
