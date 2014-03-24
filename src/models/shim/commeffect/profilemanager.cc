/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2009-2010 - DRS CenGen, LLC, Columbia, Maryland
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
 * * Neither the name of DRS CenGen, LLC nor the names of its
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

#include "profilemanager.h"
#include "filterreader.h"

EMANE::Models::CommEffect::ProfileManager::ProfileManager(EMANE::NEMId id,
                                                          EMANE::PlatformServiceProvider * pPlatformService): 
  id_{id},
  pPlatformService_{pPlatformService}
{}


EMANE::Models::CommEffect::ProfileManager::~ProfileManager()
{} 

std::pair<EMANE::Events::CommEffect,bool>
EMANE::Models::CommEffect::ProfileManager::getProfileData(const void* buf,
                                                          size_t len,
                                                          EMANE::NEMId src) const
{
  // check commeffect filter rules
  for(const auto & filter : filters_)
    {
      if(filter.match(buf, len) == 1)
        {
          return std::make_pair(filter.getEffect(),true);
        }
    }

  // search for profile data based on NEM src id
  const auto itr = profileDataMap_.find(src);

  if(itr != profileDataMap_.end())
    {
      return std::make_pair(itr->second,true);
    }
  else
    {
      return std::make_pair(Events::CommEffect{0,{},{},0,0,0,0},false);
    }
}



int EMANE::Models::CommEffect::ProfileManager::load(const char * pzFileName)
{
  try
    {
      filters_ = FilterReader::load(pzFileName);
    }
  catch(...)
    {
      return -1;
    }

  return 0;
}

int EMANE::Models::CommEffect::ProfileManager::load(const Events::CommEffects & effects)
{
  // load entries
  for(const auto & effect : effects)
    {
      if(effect.getNEMId() != 0)
        {
          // action insert or add
          std::string sAction;

          // try to insert profile data entry
          auto result = 
            profileDataMap_.insert(std::make_pair(effect.getNEMId(), effect));

          if(result.second == true)
            {
              // new entry
              sAction = "added";
            }
          else
            {
              // entry exists
              sAction = "updated";

              // update profile data
              result.first->second = effect;
            }
          
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  DEBUG_LEVEL,
                                  "SHIMI %03hu ProfileManager::%s: %s entry src: %03hu latency: %lf,"
                                  " jitter: %lf, loss: %.2f%%, dups: %.2f%%, ucbitrate %ju, bcbitrate %ju",
                                  id_,
                                  __func__,
                                  sAction.c_str(),
                                  effect.getNEMId(),
                                  std::chrono::duration_cast<DoubleSeconds>(effect.getLatency()).count(),
                                  std::chrono::duration_cast<DoubleSeconds>(effect.getJitter()).count(),
                                  effect.getProbabilityLoss(),
                                  effect.getProbabilityDuplicate(),
                                  effect.getUnicastBitRate(),
                                  effect.getBroadcastBitRate());
        }
    }

  return 0;
}
