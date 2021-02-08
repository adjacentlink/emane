/*
 * Copyright (c) 2013-2014,2021 - Adjacent Link LLC, Bridgewater,
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
                                AntennaManager & antennaManager):
  id_{id},
  rxAntennaIndex_{rxAntennaIndex},
  antennaManager_(antennaManager),
  u64AntennaUpdateSequence_{}{}

void EMANE::GainManager::setGainCache(NEMId transmitterId,
                                      const AntennaManager::AntennaInfo & txAntennaInfo,
                                      const LocationInfo & locationPairInfo,
                                      double dRemoteGaindBi,
                                      double dLocalGaindBi)
{
  gainCache_[transmitterId][txAntennaInfo.antenna_.getIndex()] =
    std::make_tuple(txAntennaInfo.u64UpdateSequence_,locationPairInfo.getSequenceNumber(),dRemoteGaindBi,dLocalGaindBi);
}

std::tuple<double,double,bool>
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
              std::uint64_t u64TxAntennaUpdateSequence{};
              std::uint64_t u64LocationUpdateSequence{};
              double dRemoteGaindBi{};
              double dLocalGaindBi{};

              std::tie(u64TxAntennaUpdateSequence,
                       u64LocationUpdateSequence,
                       dRemoteGaindBi,
                       dLocalGaindBi) = antennaIndexIter->second;

              if(u64TxAntennaUpdateSequence == txAntennaInfo.u64UpdateSequence_ &&
                 u64LocationUpdateSequence == locationPairInfo.getSequenceNumber())
                {
                  return std::make_tuple(dRemoteGaindBi,dLocalGaindBi,true);
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
      return std::make_tuple(0,0,GainStatus::ERROR_PROFILEINFO,false);
    }

  auto cacheEntry = getGainCache(transmitterId,
                                 remoteAntennaInfo.first,
                                 localAntennaInfo.first,
                                 locationPairInfo);

  if(std::get<2>(cacheEntry))
    {
      return std::make_tuple(std::get<0>(cacheEntry),
                             std::get<1>(cacheEntry),
                             GainStatus::SUCCESS,
                             true);
    }

  GainInfo gainInfo{};

  const auto & remoteAntenna = remoteAntennaInfo.first.antenna_;

  const auto & localAntenna = localAntennaInfo.first.antenna_;

  double dRemoteAntennaGaindBi{};
  double dLocalAntennaGaindBi{};

  if(!remoteAntenna.isIdealOmni())
    {
      AntennaStore::const_iterator remoteAntennaStoreIter;

      if(!locationPairInfo.isValid())
        {
          return std::make_tuple(0,0,GainStatus::ERROR_LOCATIONINFO,false);
        }

      auto remotePointing = remoteAntenna.getPointing();

      // we have the profile info w/ pattern info
      if(remotePointing.second && remoteAntennaInfo.first.pPattern_)
        {
          // calculate the direction: azimuth, elvation and distance
          auto direction =
            Utils::calculateDirection(locationPairInfo.getRemotePOV(),
                                      remoteAntennaInfo.first.placement_,
                                      locationPairInfo.getLocalPOV(),
                                      localAntennaInfo.first.placement_);

          // adjust the direction azimuth and elevation based on the antenna pointing azimuth and elvation
          auto lookupAngles =
            Utils::calculateLookupAngles(std::get<0>(direction),
                                         remotePointing.first.getAzimuthDegrees(),
                                         std::get<1>(direction),
                                         remotePointing.first.getElevationDegrees());

          double dTxAntennaGaindBi{remoteAntennaInfo.first.pPattern_->getGain(std::round(lookupAngles.first),
                                                                              std::round(lookupAngles.second))};

          // get the blockage, if specified
          //  Note: no adjustment is necessary to the direction azimuth and elvation

          double dTxAntennaBlockagedBi{remoteAntennaInfo.first.pBlockage_ ?
            remoteAntennaInfo.first.pBlockage_->getGain(std::round(std::get<0>(direction)),
                                                        std::round(std::get<1>(direction))) :
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
                                          std::get<0>(direction),
                                          std::get<1>(direction),
                                          std::get<2>(direction),
                                          remotePointing.first.getAzimuthDegrees(),
                                          remotePointing.first.getElevationDegrees(),
                                          lookupAngles.first,
                                          lookupAngles.second);

          dRemoteAntennaGaindBi = dTxAntennaGaindBi + dTxAntennaBlockagedBi;

        }
      else
        {
          // profile info is missing
          return std::make_tuple(0,0,GainStatus::ERROR_PROFILEINFO,false);
        }
    }
  else
    {
      dRemoteAntennaGaindBi = remoteAntenna.getFixedGaindBi().first;
    }

  const auto & localPointing = localAntenna.getPointing();

  if(!localAntenna.isIdealOmni())
    {
      if(!locationPairInfo.isValid())
        {
          return std::make_tuple(0,0,GainStatus::ERROR_LOCATIONINFO,false);
        }

      if(!localPointing.second)
        {
          return std::make_tuple(0,0,GainStatus::ERROR_PROFILEINFO,false);
        }

      // calculate the direction: azimuth, elvation and distance
      auto direction =
        Utils::calculateDirection(locationPairInfo.getLocalPOV(),
                                  localAntennaInfo.first.placement_,
                                  locationPairInfo.getRemotePOV(),
                                  remoteAntennaInfo.first.placement_);

      // adjust the direction azimuth and elevation based on the antenna pointing azimuth and elvation
      auto lookupAngles =
        Utils::calculateLookupAngles(std::get<0>(direction),
                                     localPointing.first.getAzimuthDegrees(),
                                     std::get<1>(direction),
                                     localPointing.first.getElevationDegrees());

      // get the local receiver antenna gain
      double dRxAntennaGaindBi{localAntennaInfo.first.pPattern_->getGain(std::round(lookupAngles.first),
                                                                         std::round(lookupAngles.second))};

      // get the blockage, if specified
      //  Note: no adjustment is necessary to the direction azimuth and elvation
      double dRxAntennaBlockagedBi{localAntennaInfo.first.pBlockage_ ?
        localAntennaInfo.first.pBlockage_->getGain(std::round(std::get<0>(direction)),
                                                   std::round(std::get<1>(direction))) :
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
                                      std::get<0>(direction),
                                      std::get<1>(direction),
                                      std::get<2>(direction),
                                      localPointing.first.getAzimuthDegrees(),
                                      localPointing.first.getElevationDegrees(),
                                      lookupAngles.first,
                                      lookupAngles.second);

      dLocalAntennaGaindBi = dRxAntennaGaindBi + dRxAntennaBlockagedBi;
    }
  else
    {
      dLocalAntennaGaindBi = localAntenna.getFixedGaindBi().first;
    }

  const auto & localPosition = locationPairInfo.getLocalPOV().getPosition();
  const auto & remotePosition = locationPairInfo.getRemotePOV().getPosition();
  double dDistanceMeters{locationPairInfo.getDistanceMeters()};

  // check if antennas are below the horizon
  if(locationPairInfo.isValid() &&
     dDistanceMeters > 10 &&
     Utils::checkHorizon(localPosition.getAltitudeMeters() +
                         localAntennaInfo.first.placement_.getUpMeters(),
                         remotePosition.getAltitudeMeters() +
                         remoteAntennaInfo.first.placement_.getUpMeters(),
                         dDistanceMeters) == false)
    {
      // below horizon
      return std::make_tuple(0,0,GainStatus::ERROR_HORIZON,false);
    }
  else
    {
      setGainCache(transmitterId,
                   remoteAntennaInfo.first,
                   locationPairInfo,
                   dRemoteAntennaGaindBi,
                   dLocalAntennaGaindBi);

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

  return std::make_tuple(dRemoteAntennaGaindBi,dLocalAntennaGaindBi,GainStatus::SUCCESS,false);
}
