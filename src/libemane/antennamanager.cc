/*
 * Copyright (c) 2020 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "antennamanager.h"

EMANE::AntennaManager::AntennaInfo::AntennaInfo():
  u64UpdateSequence_{},
  antenna_{},
  pPattern_{},
  pBlockage_{},
  placement_{}{}

EMANE::AntennaManager::AntennaManager():
  store_{},
  u64UpdateSequence_{}{}


void EMANE::AntennaManager::update(const Events::AntennaProfiles & antennaProfiles)
{
  ++u64UpdateSequence_;

  // updates coming in as events are for radio models using the
  // original (compat 1) antenna profile event mechanisms for
  // selecting and pointing a single antenna. Compat 1 single
  // antenna uses the DEFAULT_ANTENNA_INDEX.
  for(const auto & antennaProfile : antennaProfiles)
    {
      auto target = Antenna::createProfileDefined(DEFAULT_ANTENNA_INDEX,
                                                  {antennaProfile.getAntennaProfileId(),
                                                   antennaProfile.getAntennaAzimuthDegrees(),
                                                   antennaProfile.getAntennaElevationDegrees()});

      // store all antenna event info so you can construct a
      // compat 2 tx antenna entry. In compat 2, each OTA message
      // contains one or more transmit antennas w/ profile and
      // pointing info.
      //
      // A radio model can send a tx antenna for downstream use
      // with no pointing to indicate that compat 1 style antenna
      // profiles are in use.
      defaultEventPointingStore_[antennaProfile.getNEMId()] = target.getPointing().first;

      update(antennaProfile.getNEMId(),target);
    }
}

void EMANE::AntennaManager::update(NEMId nemId, const Antenna & antenna)
{
  ++u64UpdateSequence_;

  auto target = antenna;

  auto iterNemStore = store_.find(nemId);

  if(iterNemStore == store_.end())
    {
      iterNemStore =
        store_.emplace(nemId,AntennaStore{}).first;
    }

  auto iterAntenna = iterNemStore->second.find(target.getIndex());

  if(iterAntenna == iterNemStore->second.end())
    {
      iterAntenna = iterNemStore->second.emplace(target.getIndex(),
                                                 AntennaInfo{}).first;

      iterAntenna->second.u64UpdateSequence_ = u64UpdateSequence_;
    }

  if(iterAntenna->second.antenna_ != target)
    {
      if(target.isProfileDefined())
        {
          if(!target.getPointing().second && !target.getIndex())
            {
              auto iterDefaultPointing = defaultEventPointingStore_.find(nemId);

              if(iterDefaultPointing != defaultEventPointingStore_.end())
                {
                  target.setPointing(iterDefaultPointing->second);
                }
            }

          const auto & currentPointing = iterAntenna->second.antenna_.getPointing();
          const auto & targetPointing = target.getPointing();

          if(targetPointing.first.getProfileId() != currentPointing.first.getProfileId())
            {
              const auto ret =
                AntennaProfileManifest::instance()->getProfileInfo(targetPointing.first.getProfileId());

              if(ret.second)
                {
                  iterAntenna->second.pPattern_ = std::get<0>(ret.first);
                  iterAntenna->second.pBlockage_ = std::get<1>(ret.first);
                  iterAntenna->second.placement_ = std::get<2>(ret.first);
                }
              else
                {
                  // unknown profile
                  iterAntenna->second.pPattern_ = nullptr;
                  iterAntenna->second.pBlockage_ = nullptr;
                  iterAntenna->second.placement_ = {};
                }
            }
        }
      else
        {
          iterAntenna->second.pPattern_ = nullptr;
          iterAntenna->second.pBlockage_ = nullptr;
          iterAntenna->second.placement_ = {};
        }

      iterAntenna->second.u64UpdateSequence_ = u64UpdateSequence_;
      iterAntenna->second.antenna_ = target;
    }
}

std::pair<const EMANE::AntennaManager::AntennaInfo &, bool>
EMANE::AntennaManager::getAntennaInfo(NEMId nemId,
                                      AntennaIndex antennaIndex) const
{
  static AntennaInfo empty{};

  auto iterNemStore = store_.find(nemId);

  if(iterNemStore != store_.end())
    {
      auto iterAntenna = iterNemStore->second.find(antennaIndex);

      if(iterAntenna != iterNemStore->second.end())
        {
          return {iterAntenna->second,true};
        }
    }

  return {empty,false};
}

void EMANE::AntennaManager::remove(NEMId nemId,
                                   AntennaIndex antennaIndex)
{
  auto iterNemStore = store_.find(nemId);

  if(iterNemStore != store_.end())
    {
      iterNemStore->second.erase(antennaIndex);
    }
}
