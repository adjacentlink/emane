/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2010-2012 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANEMODELSIEEE802ABGNEIGHBORENTRY_HEADER_
#define EMANEMODELSIEEE802ABGNEIGHBORENTRY_HEADER_

#include "emane/types.h"

#include "neighbortype.h"
#include "msgtypes.h"

#include <map>
#include <set>


namespace EMANE
 {
  namespace Models
   {
     namespace IEEE80211ABG
       {
        /**
         *
         * @class  NeighborEntry
         *
         * @brief Defines a 1 hop nbr and its bandwidth utilization
         *
         */
        class NeighborEntry
        {
        public:
         /**
          *
          * constructor
          *
          */
          NeighborEntry();

         /**
          *
          * updates the bandwidth utilization parameters
          *
          * @param utilizationMicroseconds the bandwidth (duration) of channel activity
          * @param type                    the msg type (unicast, unicast rts/cts, broadcast)
          * @param activityTime            the absolute time that the activity
          * @param fRxPowerMilliWatts      the rx power in milliwatts
          * @param numPackets              the number of packets associated with the activity default is 1
          *
          */
          void updateChannelActivity(const Microseconds & utilizationMicroseconds, 
                                     std::uint8_t type,
                                     const TimePoint & activityTime, 
                                     float fRxPowerMilliWatts, 
                                     size_t numPackets = 1);

         /**
          *
          * resets the bandwidth utilization parameter(s)
          *
          * @param msgTypeMask  the msg type (unicast, unicast rts/cts, broadcast)
          *
          */
          void resetUtilization(std::uint8_t msgTypeMask);

          void resetUtilization();

          EMANE::TimePoint getLastActivityTime() const;

         /**
          *
          * gets the bandiwdth utilzation (duration) time
          *
          * @param msgTypeMask  the msg type (unicast, unicast rts/cts, broadcast)
          *
          * @return the bandiwdth utilzation
          *
          */
          Microseconds getUtilizationMicroseconds(std::uint8_t msgTypeMask) const;

         /**
          *
          * gets the number of utilization packets
          *
          * @param msgTypeMask  the msg type (unicast, unicast rts/cts, broadcast)
          *
          * @return the number of utilization packets
          *
          */
          size_t getNumberOfPackets(std::uint8_t msgTypeMask) const;


         /**
          *
          * gets the utilization rx power
          *
          * @param msgTypeMask  the msg type (unicast, unicast rts/cts, broadcast)
          *
          * @return the utilization rx power sum
          *
          */
          float getRxPowerMilliWatts(std::uint8_t msgTypeMask) const;

          float getEstimatedNumCommonNeighbors() const;

          void setEstimatedNumCommonNeighbors(float num);

          float getHiddenChannelActivity() const;

          void setHiddenChannelActivity(float fActivity);

          void setAverageHiddenRxPowerMilliWatts(float dAverageHiddenRxPowerMilliWatts);

          float getAverageHiddenRxPowerMilliWatts() const;

          void setAverageCommonRxPowerMilliWatts(float dAverageCommonRxPowerMilliWatts);

          float getAverageCommonRxPowerMilliWatts() const;

          void setOneHopNeighbors(const NbrSet & nbrs);

          void setHiddenNeighbors(const NbrSet & nbrs);

          void setCommonNeighbors(const NbrSet & nbrs);

          const NbrSet & getOneHopNeighbors() const;

          const NbrSet & getCommonNeighbors() const;

          const NbrSet & getHiddenNeighbors() const;

          bool isOneHopNbr(EMANE::NEMId id) const;

          bool isCommonNbr(EMANE::NEMId id) const;

          bool isHiddenNbr(EMANE::NEMId id) const;

        private:
        /**
         *
         * @brief Defines a 1 hop nbr bandwidth utilization
         *
         */

          struct Utilization
            {
               Microseconds   totalUtilizationMicroseconds_;
               size_t         totalNumPackets_;
               float          fTotalRxPowerMilliWatts_;

               Utilization() :
                totalUtilizationMicroseconds_{},
                totalNumPackets_{},
                fTotalRxPowerMilliWatts_{}
               { }

               void reset()
                 {
                   totalUtilizationMicroseconds_ = Microseconds::zero();
                   totalNumPackets_         = 0;
                   fTotalRxPowerMilliWatts_ = 0;
                 }

               void update(size_t numPackets, const Microseconds & utilizationMicroseconds, float fRxPowerMilliWatts)
                 {
                   totalNumPackets_ += numPackets;

                   totalUtilizationMicroseconds_ += utilizationMicroseconds;

                   fTotalRxPowerMilliWatts_ += fRxPowerMilliWatts;
                }
            };

          using UtilizationTypeMap = std::map<std::uint8_t, Utilization>;

          UtilizationTypeMap utilizationTypeMap_;

          TimePoint lastActivityTime_;

          NbrSet oneHopNbrSet_;

          NbrSet commonNbrSet_;

          NbrSet hiddenNbrSet_;

          float fEstimatedNumCommonNeighbors_;

          float fHiddenChannelActivity_;

          float fAverageHiddenRxPowerMilliWatts_;

          float fAverageCommonRxPowerMilliWatts_;
        };
     }
  } 
}
#endif  //EMANEMODELSIEEE802ABGNEIGHBORENTRY_HEADER_
