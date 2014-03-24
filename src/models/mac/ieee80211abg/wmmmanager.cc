/*
 * Copyright (c) 2013 - Adjacent Link, LLC, Bridgewater New Jersey
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

#include "wmmmanager.h"
#include "maclayer.h"
#include "utils.h"

#include <ace/OS_NS_time.h>



EMANE::Models::IEEE80211ABG::WMMManager::WMMManager(NEMId id, 
                                                    PlatformServiceProvider * pPlatformService, 
                                                    MACLayer *pMACLayer):
  id_{id}, 
  pPlatformService_{pPlatformService},
  pMACLayler_{pMACLayer},
  u8NumCategories_{1}
{
   totalUtilizationVector_.resize(u8NumCategories_);
   localUtilizationVector_.resize(u8NumCategories_);

   // reset 
   resetCounters();
}



EMANE::Models::IEEE80211ABG::WMMManager::~WMMManager()
{ }




void
EMANE::Models::IEEE80211ABG::WMMManager::updateTotalActivity(const std::uint8_t u8Category,
                                                             const Microseconds & durationMicroseconds)
{
   if(u8Category > u8NumCategories_)
    {
      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                             ERROR_LEVEL,
                             "MACI %03hu %s::%s: invalid category %hhu, max is %hhu, ignore",
                             id_,
                             "WMMManager", 
                             __func__, 
                             u8Category, 
                             u8NumCategories_);
    }
   else
    {
      // total sum for this index
      totalUtilizationVector_[u8Category] += durationMicroseconds;

      // save the total
      totalUtilizationMicroseconds_ += durationMicroseconds;

      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                             DEBUG_LEVEL, 
                             "MACI %03hu %s::%s: category %hhu, duration %lf",
                             id_, 
                             "WMMManager", 
                             __func__,
                             u8Category, 
                             std::chrono::duration_cast<DoubleSeconds>(durationMicroseconds).count());
    }
}



void
EMANE::Models::IEEE80211ABG::WMMManager::updateLocalActivity(const std::uint8_t u8Category,
                                                             const Microseconds & durationMicroseconds)
{
   if(u8Category > u8NumCategories_)
    {
      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                             ERROR_LEVEL, 
                             "MACI %03hu %s::%s: invalid category %hhu, max is %hhu, ignore",
                             id_,
                             "WMMManager", 
                             __func__, 
                             u8Category,
                             u8NumCategories_);
    }
   else
    {
      // local sum for this index
      localUtilizationVector_[u8Category] += durationMicroseconds;

      // total sum for this index
      totalUtilizationVector_[u8Category] += durationMicroseconds;

      // save the total
      totalUtilizationMicroseconds_ += durationMicroseconds;

      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                             DEBUG_LEVEL,
                             "MACI %03hu %s::%s: category %hhu, duration %lf",
                              id_,
                              "WMMManager",
                              __func__, 
                              u8Category, 
                              std::chrono::duration_cast<DoubleSeconds>(durationMicroseconds).count());
    }
}



void
EMANE::Models::IEEE80211ABG::WMMManager::setNumCategories(const std::uint8_t u8NumCategories)
{
   if(u8NumCategories_ != u8NumCategories)
    {
      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                             DEBUG_LEVEL, 
                             "MACI %03hu %s::%s: change num categories from %hhu to %hhu", 
                             id_, 
                             "WMMManager",
                             __func__,
                             u8NumCategories_,
                             u8NumCategories);

      // resize
      totalUtilizationVector_.resize(u8NumCategories);
      localUtilizationVector_.resize(u8NumCategories);

      u8NumCategories_ = u8NumCategories;

      // reset ALL values
      resetCounters();
   }
}


void
EMANE::Models::IEEE80211ABG::WMMManager::resetCounters()
{
   for(auto & iter : totalUtilizationVector_)
    {
      iter = Microseconds::zero();
    }

   for(auto & iter : localUtilizationVector_)
    {
      iter = Microseconds::zero();
    }

    totalUtilizationMicroseconds_ = Microseconds::zero();
}



EMANE::Models::IEEE80211ABG::WMMManager::UtilizationRatioVector 
EMANE::Models::IEEE80211ABG::WMMManager::getUtilizationRatios(const Microseconds & deltaTMicroseconds)
{
  // initialize to <total = 0.0, local = 0.0>
  UtilizationRatioVector vec(u8NumCategories_, UtilizationRatioPair(0.0f, 0.0f));

  // check for divide by 0
  if((totalUtilizationMicroseconds_ > Microseconds::zero()) && (deltaTMicroseconds > Microseconds::zero()))
   {
     // get activity ratio (used / interval)
     float fActivityRatio{getRatio(totalUtilizationMicroseconds_, deltaTMicroseconds)};

     // clamp it 
     if(fActivityRatio > 1.0f)
      {
        fActivityRatio = 1.0f;
      }

     // get the utilization ratio(s)
     for(std::uint8_t u8Category = 0; u8Category < u8NumCategories_; ++u8Category)
      {
        // set total ratio
        vec[u8Category].first = getRatio(totalUtilizationVector_[u8Category], totalUtilizationMicroseconds_) * fActivityRatio;

        // check for divide by 0
        if(totalUtilizationVector_[u8Category] > Microseconds::zero())
         {
           // set local
           vec[u8Category].second = getRatio(localUtilizationVector_[u8Category], totalUtilizationVector_[u8Category]);
         }
        else
         {
           vec[u8Category].second = 0.0f;
         }

        LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                               DEBUG_LEVEL,
                               "MACI %03hu %s::%s: category %hhu, [total_bw %lf, ratio %f], "
                               "usage [total %f, %3.2f%%]", 
                               id_,
                               "WMMManager",
                               __func__, 
                               u8Category, 
                               vec[u8Category].first, 
                               vec[u8Category].second,
                               std::chrono::duration_cast<DoubleSeconds>(totalUtilizationMicroseconds_).count(),
                               fActivityRatio * 100.0f);
      }
   }

  // clear counters
  resetCounters();

  return vec;
}
