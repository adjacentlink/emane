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

#include "emane/models/frameworkphy/gainmanager.h"
#include "emane/models/frameworkphy/antennaprofilemanifest.h"
#include "emane/models/frameworkphy/positionutils.h"
#include "emane/models/frameworkphy/antennaprofileexception.h"
#include "logservice.h"
#include "emane/models/frameworkphy/locationinfoformatter.h"
#include "emane/models/frameworkphy/positionneuformatter.h"

EMANE::GainManager::GainManager(NEMId nemId):
  nemId_{nemId},
   pLocalPattern_{},
   pLocalBlockage_{},
   localAntennaPlacement_{},
   dLocalAntennaAzimuthDegrees_{},
   dLocalAntennaElevationDegrees_{},
   bHasLocalAntennaProfile_{false}{}

void EMANE::GainManager::update(const Events::AntennaProfiles & antennaProfiles)
{
  for(const auto & antennaProfile : antennaProfiles)
    {
      const auto ret = 
        AntennaProfileManifest::instance()->getProfileInfo(antennaProfile.getAntennaProfileId());

      // do we have the profile
      if(ret.second)
        {
          if(nemId_ == antennaProfile.getNEMId())
            {
              pLocalPattern_ = std::get<0>(ret.first);
              pLocalBlockage_ = std::get<1>(ret.first);
              localAntennaPlacement_ =  std::get<2>(ret.first);
              dLocalAntennaAzimuthDegrees_ = antennaProfile.getAntennaAzimuthDegrees();
              dLocalAntennaElevationDegrees_ = antennaProfile.getAntennaElevationDegrees();
              bHasLocalAntennaProfile_ = true;
            }
          else
            {
              antennaProfileStore_[antennaProfile.getNEMId()] = antennaProfile;
            }
        }
      else
        {
          throw makeException<AntennaProfileException>("NEM %hu: unknown antenna profile %hu",
                                                       nemId_,
                                                       antennaProfile);
        }
    }
}


std::pair<double,EMANE::GainManager::GainStatus> 
EMANE::GainManager::determineGain(NEMId transmitterId,
                                  const LocationInfo & locationPairInfo,
                                  const std::pair<double, bool> & optionalRxFixedGaindBi,
                                  const std::pair<double, bool> & optionalTxFixedGaindBi) const
{
  double dRxAntennaGaindBi{optionalRxFixedGaindBi.first};
  double dTxAntennaGaindBi{optionalTxFixedGaindBi.first};
  double dRxAntennaBlockagedBi{};
  double dTxAntennaBlockagedBi{};

  PositionNEU antennaPlacement{};

  LOGGER_VERBOSE_LOGGING_FN_VARGS(*LogServiceSingleton::instance(),
                                  DEBUG_LEVEL,
                                  LocationInfoFormatter(locationPairInfo),
                                  "PHYI %03hu GainManager::%s src: %hu rx fixed gain %lf (%s) tx fixed gain %lf (%s)",
                                  nemId_,
                                  __func__,
                                  transmitterId,
                                  dRxAntennaGaindBi,
                                  optionalRxFixedGaindBi.second ? "yes" : "no",
                                  dTxAntennaGaindBi,
                                  optionalTxFixedGaindBi.second ? "yes" : "no");
  
  if(!optionalTxFixedGaindBi.second)
    {
      AntennaProfileStore::const_iterator remoteAntennaProfileIter;
      
      if(!locationPairInfo)
        {
          return {0,GainStatus::ERROR_LOCATIONINFO};
        }

      if((remoteAntennaProfileIter = antennaProfileStore_.find(transmitterId)) == antennaProfileStore_.end())
        {
          return {0,GainStatus::ERROR_PROFILEINFO};
        }
      
      // retrieve remote transmitter antenna profile id and pointing info
      auto ret = 
        AntennaProfileManifest::instance()->getProfileInfo(remoteAntennaProfileIter->second.getAntennaProfileId());

      // we have the profile info
      if(ret.second)
        {
          // get the antenna pattern and optional blockage and antenna placement
          AntennaPattern * pRemotePattern{std::get<0>(ret.first)};
          AntennaPattern * pRemoteBlockage{std::get<1>(ret.first)};
          antennaPlacement = std::get<2>(ret.first);
      
          // calculate the direction: azimuth, elvation and distance
          auto direction = 
            Utils::calculateDirection(locationPairInfo.getRemotePOV(),
                                      antennaPlacement,
                                      locationPairInfo.getLocalPOV(),
                                      localAntennaPlacement_);
                
          // adjust the direction azimuth and elvation based on the antenna pointing azimuth and elvation
          auto lookupAngles = 
            Utils::calculateLookupAngles(std::get<0>(direction),
                                         remoteAntennaProfileIter->second.getAntennaAzimuthDegrees(),
                                         std::get<1>(direction),
                                         remoteAntennaProfileIter->second.getAntennaElevationDegrees());
          
          // get the remote transmitter antenna gain
          dTxAntennaGaindBi = pRemotePattern->getGain(std::round(lookupAngles.first),
                                                      std::round(lookupAngles.second));
          
          // get the blocakage, if specified 
          //  Note: no adjustment is necessary to the direction azimuth and elvation
          if(pRemoteBlockage)
            {
              dTxAntennaBlockagedBi = pRemoteBlockage->getGain(std::round(std::get<0>(direction)),
                                                               std::round(std::get<1>(direction)));
            }

          LOGGER_VERBOSE_LOGGING_FN_VARGS(*LogServiceSingleton::instance(),
                                          DEBUG_LEVEL,
                                          [this,&antennaPlacement]()
                                          {
                                            Strings strings;
                                            
                                            strings.push_back("remote antenna");
                                            strings.splice(strings.end(),PositionNEUFormatter(antennaPlacement)());

                                            strings.push_back("local antenna");
                                            strings.splice(strings.end(),PositionNEUFormatter(localAntennaPlacement_)());

                                            return strings;
                                          },
                                          "PHYI %03hu GainManager::%s remote calc tx antenna gain: %lf tx antenna"
                                          " blockage: %lf direction az: %lf el: %lf dist: %lf remote antenna az: %lf el: %lf"
                                          " lookup bearing: %lf lookup el: %lf",
                                          nemId_,
                                          __func__,
                                          dTxAntennaGaindBi,
                                          dTxAntennaBlockagedBi,
                                          std::get<0>(direction),
                                          std::get<1>(direction),
                                          std::get<2>(direction),
                                          remoteAntennaProfileIter->second.getAntennaAzimuthDegrees(),
                                          remoteAntennaProfileIter->second.getAntennaElevationDegrees(),
                                          lookupAngles.first,
                                          lookupAngles.second);
        }
      else
        {
          // profile info is missing
          return {0,GainStatus::ERROR_PROFILEINFO};
        }
    }
            
  if(!optionalRxFixedGaindBi.second)
    {
      if(!locationPairInfo)
        {
          return {0,GainStatus::ERROR_LOCATIONINFO};
        }

      if(!bHasLocalAntennaProfile_)
        {
          return {0,GainStatus::ERROR_PROFILEINFO};
        }

      // calculate the direction: azimuth, elvation and distance
      auto direction = 
        Utils::calculateDirection(locationPairInfo.getLocalPOV(),
                                  localAntennaPlacement_,
                                  locationPairInfo.getRemotePOV(),
                                  antennaPlacement);

      // adjust the direction azimuth and elvation based on the antenna pointing azimuth and elvation
      auto lookupAngles = 
        Utils::calculateLookupAngles(std::get<0>(direction),
                                     dLocalAntennaAzimuthDegrees_,
                                     std::get<1>(direction),
                                     dLocalAntennaElevationDegrees_);

      // get the local receiver antenna gain   
      dRxAntennaGaindBi =
        pLocalPattern_->getGain(std::round(lookupAngles.first),
                                std::round(lookupAngles.second));

      // get the blocakage, if specified 
      //  Note: no adjustment is necessary to the direction azimuth and elvation
      if(pLocalBlockage_)
        {
          dRxAntennaBlockagedBi =
            pLocalBlockage_->getGain(std::round(std::get<0>(direction)),
                                     std::round(std::get<1>(direction)));
        }

      LOGGER_VERBOSE_LOGGING_FN_VARGS(*LogServiceSingleton::instance(),
                                      DEBUG_LEVEL,
                                      [this,&antennaPlacement]()
                                      {
                                        Strings strings;

                                        strings.push_back("local antenna");
                                        strings.splice(strings.end(),PositionNEUFormatter(localAntennaPlacement_)());
                                        
                                        strings.push_back("remote antenna");
                                        strings.splice(strings.end(),PositionNEUFormatter(antennaPlacement)());

                                        return strings;
                                      },
                                      "PHYI %03hu GainManager::%s local calc rx antenna gain: %lf rx antenna"
                                      " blockage: %lf direction az: %lf el: %lf dist: %lf local antenna az: %lf el: %lf"
                                      " lookup bearing: %lf lookup el: %lf",
                                      nemId_,
                                      __func__,
                                      dRxAntennaGaindBi,
                                      dRxAntennaBlockagedBi,
                                      std::get<0>(direction),
                                      std::get<1>(direction),
                                      std::get<2>(direction),
                                      dLocalAntennaAzimuthDegrees_,
                                      dLocalAntennaElevationDegrees_,
                                      lookupAngles.first,
                                      lookupAngles.second);                            
          

    }

  const auto & localPosition = locationPairInfo.getLocalPOV().getPosition();
  const auto & remotePosition = locationPairInfo.getRemotePOV().getPosition();
  double dDistanceMeters{locationPairInfo.getDistanceMeters()};
  
  // check if antennas are below the horizon
  if(!locationPairInfo == false && 
     dDistanceMeters > 10 &&
     Utils::checkHorizon(localPosition.getAltitudeMeters() +
                         localAntennaPlacement_.getUpMeters(),
                         remotePosition.getAltitudeMeters() +
                         antennaPlacement.getUpMeters(),
                         dDistanceMeters) == false) 
    {
      // below horizon use minimal gain
      return {0,GainStatus::ERROR_HORIZON};
    }

  double dCombinedGaindBi{dRxAntennaGaindBi + dRxAntennaBlockagedBi + dTxAntennaGaindBi + dTxAntennaBlockagedBi};

  LOGGER_VERBOSE_LOGGING(*LogServiceSingleton::instance(),
                         DEBUG_LEVEL,
                         "PHYI %03hu GainManager::%s combined gain: %lf",
                         nemId_,
                         __func__,
                         dCombinedGaindBi);
  
  return {dCombinedGaindBi, GainStatus::SUCCESS};
}
