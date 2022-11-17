/*
 * Copyright (c) 2013,2020 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "locationmanager.h"
#include "positionutils.h"

EMANE::LocationManager::LocationManager(NEMId nemId):
  nemId_{nemId},
  u64CacheSequenceNumber_{}{}

void EMANE::LocationManager::update(const Events::Locations & locations)
{
  for(const auto & location : locations)
    {
      EMANE::NEMId targetNEMId{location.getNEMId()};

      if(nemId_ == targetNEMId)
        {
          // if self nem location changes clear
          //  the location pair cache
          if(localPOV_.update(location.getPosition(),
                              location.getOrientation(),
                              location.getVelocity()))
            {
              locationInfoCache_.clear();
            }
        }
      else
        {
          auto iter = locationStore_.find(targetNEMId);

          if(iter != locationStore_.end())
            {
              // if nem location changes clear the
              //   location pair chache if any
              if(iter->second.update(location.getPosition(),
                                     location.getOrientation(),
                                     location.getVelocity()))
                {
                  locationInfoCache_.erase(targetNEMId);
                }
            }
          else
            {
              locationStore_.insert({targetNEMId,{location.getPosition(),
                                                  location.getOrientation(),
                                                  location.getVelocity()}});
            }
        }
    }
}


std::pair<EMANE::LocationInfo,bool> EMANE::LocationManager::getLocationInfo(NEMId remoteNEMId)
{
  if(localPOV_.isValid())
    {
      auto cacheIter = locationInfoCache_.find(remoteNEMId);

      if(cacheIter != locationInfoCache_.end())
        {
          return {cacheIter->second,true};
        }
      else
        {
          auto iter = locationStore_.find(remoteNEMId);

          if(iter != locationStore_.end())
            {
              LocationInfo locationInfo{localPOV_,iter->second,++u64CacheSequenceNumber_};

              locationInfoCache_[remoteNEMId] = locationInfo;

              return {locationInfo,true};
            }
        }
    }

  return {LocationInfo{},false};
}

const EMANE::PositionOrientationVelocity & EMANE::LocationManager::getLocalPOV() const
{
  return localPOV_;
}
