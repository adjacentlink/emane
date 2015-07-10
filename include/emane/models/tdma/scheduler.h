/*
 * Copyright (c) 2015 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANETDMARADIOMODELSCHEDULER_HEADER_
#define EMANETDMARADIOMODELSCHEDULER_HEADER_

#include "emane/component.h"
#include "emane/platformserviceuser.h"
#include "emane/runningstatemutable.h"
#include "emane/upstreampacket.h"

#include "emane/models/tdma/types.h"
#include "emane/models/tdma/scheduleruser.h"

namespace EMANE
{
  namespace Models
  {
    namespace TDMA
    {
      /**
       * @class Scheduler
       *
       * @brief Scheduler interface used by BaseModel to communicate
       * with a scheduler module
       */
      class Scheduler : public Component,
                        public PlatformServiceUser,
                        public RunningStateMutable
      {
      public:
        /**
         * Destroys an instance.
         */
        virtual ~Scheduler(){};

        /**
         * Gets the RxSlotInfo for a specified time.
         *
         * @param timePoint Current time
         *
         * @return A pair holding the RxSlotInfo and a bool set to
         * true if the slot is a valid receive slot.
         */
        virtual std::pair<RxSlotInfo,bool>
        getRxSlotInfo(const TimePoint & timePoint) const = 0;

        /**
         * Gets the transmit slot info for a specified number of
         * multiframes occurring on or after a specified time.
         *
         * @param timePoint Start time of transmit opportunity search
         * @param multiframes Number of mulitframes worth of transmit
         * opportunities to return
         *
         * @return A pair holding a list of TxSlotInfo entries and the
         * next time to use when requesting more transmit
         * opportunities.
         *
         * @note Using the returned time value during the next request
         * allows you to determine the number of missed opportunities
         * due to system timing/resource issues.
         */
        virtual std::pair<TxSlotInfos,TimePoint>
        getTxSlotInfo(const TimePoint & timePoint, int multiframes) const = 0;


        /**
         * Gets the slot info for a specified absolute slot index
         *
         * @param u64AbsoluteSlotIndex Absolute slot index
         *
         * @return Slot information
         */
        virtual SlotInfo getSlotInfo(std::uint64_t u64AbsoluteSlotIndex) const = 0;


        /**
         * Gets the slot info for a specified time
         *
         * @param timePoint Time of desired slot info
         *
         * @return Slot information
         */
        virtual SlotInfo getSlotInfo(const TimePoint & timePoint) const = 0;


       /**
         * Process a %Scheduler message received over-the-air.
         *
         * @param pkt Received UpstreamPacket
         * @param packetMetaInfo Meta information associated with
         * received packet
         *
         */
        virtual void processSchedulerPacket(UpstreamPacket & pkt,
                                            const PacketMetaInfo & packetMetaInfo) = 0;

        /**
         * Process packet information for a received over-the-air data
         * packet.
         *
         * @param packetMetaInfo Meta information associated with
         * received packet
         */
        virtual void processPacketMetaInfo(const PacketMetaInfo & packetMetaInfo) = 0;


      protected:
        SchedulerUser * pSchedulerUser_;
        NEMId id_;

        /**
         * Creates an instance.
         */
        Scheduler(NEMId id,
                  PlatformServiceProvider * pPlatformServiceProvider,
                  SchedulerUser * pSchedulerUser):
          PlatformServiceUser{pPlatformServiceProvider},
          pSchedulerUser_{pSchedulerUser},
          id_{id}{}
      };
    }
  }
}

#endif // EMANETDMARADIOMODELSCHEDULER_HEADER_
