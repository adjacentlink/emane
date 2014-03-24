/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater New Jersey
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

#ifndef EMANEMODELSIEEE80211ABGWMMMANAGER_HEADER_
#define EMANEMODELSIEEE80211ABGWMMMANAGER_HEADER_


#include "emane/types.h"
#include "emane/platformserviceprovider.h"

#include <vector>

namespace EMANE
{
  namespace Models
   {
     namespace IEEE80211ABG
      {
        class MACLayer;

      /**
       * @class  WMMManager
       *
       * @brief Defines the WMM manager
       *
       */
        class WMMManager
        {
         public:
           typedef std::pair<float, float> UtilizationRatioPair; // <total, local>

           typedef std::vector<UtilizationRatioPair>  UtilizationRatioVector;

           WMMManager(NEMId id, PlatformServiceProvider * pPlatformService, MACLayer *pMACLayer);

           ~WMMManager();

           void updateTotalActivity(std::uint8_t u8Category, const Microseconds & durationMicroseconds);
 
           void updateLocalActivity(std::uint8_t u8Category, const Microseconds & durationMicroseconds);

           void setNumCategories(const std::uint8_t u8NumCategories);

           UtilizationRatioVector getUtilizationRatios(const Microseconds & deltaTMicroseconds);

         private:
           typedef std::vector<Microseconds> UtilizationVector;

           NEMId id_;

           PlatformServiceProvider * pPlatformService_;

           MACLayer *pMACLayler_;

           UtilizationVector localUtilizationVector_;

           UtilizationVector totalUtilizationVector_;

           Microseconds totalUtilizationMicroseconds_;

           std::uint8_t u8NumCategories_;

           void resetCounters();
         };
      }
   }
}
#endif    //EMANEMODELSIEEE80211ABG_WMMMANAGER_HEADER_
