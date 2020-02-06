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

#ifndef EMANETDMARADIOMODELEVENTSCHEDULER_HEADER_
#define EMANETDMARADIOMODELEVENTSCHEDULER_HEADER_

#include "emane/models/tdma/scheduler.h"
#include "emane/events/slotinfo.h"
#include "emane/events/slotstructure.h"
#include "emane/statisticnumeric.h"
#include "emane/models/frameworkphy/eventtablepublisher.h"
#include "slotter.h"

namespace EMANE
{
  namespace Models
  {
    namespace TDMA
    {
      /**
       * @class EventScheduler
       *
       * @brief Reference Scheduler implementation
       *
       * Implementation receives a %TDMA schdule via an event.
       */
      class EventScheduler : public Scheduler
      {
      public:
        EventScheduler(NEMId id,
                       PlatformServiceProvider * pPlatformServiceProvider,
                       SchedulerUser * pSchedulerUser);

        ~EventScheduler();

        void initialize(Registrar & registrar) override;

        void configure(const ConfigurationUpdate & update) override;

        void start() override;

        void postStart() override;

        void stop() override;

        void destroy() throw() override;

        void processEvent(const EventId & eventId,
                          const Serialization & serialization) override;

        std::pair<RxSlotInfo,bool> getRxSlotInfo(const TimePoint & timePoint) const override;

        std::pair<TxSlotInfos,TimePoint> getTxSlotInfo(const TimePoint & timePoint,
                                                       int multiframes) const override;

        SlotInfo getSlotInfo(std::uint64_t u64AbsoluteSlotIndex) const override;

        SlotInfo getSlotInfo(const TimePoint & timePoint) const override;

        void processSchedulerPacket(UpstreamPacket & pkt,
                                    const PacketMetaInfo & packetMetaInfo) override;

        void processPacketMetaInfo(const PacketMetaInfo & packetMetaInfo) override;

      private:
        Events::SlotInfos slotInfos_;
        Events::SlotStructure structure_;
        EventTablePublisher eventTablePublisher_;
        Slotter slotter_;
        mutable bool bWaitingFirstTxSlotInfoRequest_;
        Frequencies frequencies_;
        StatisticNumeric<std::uint64_t> * pNumScheduleRejectSlotIndexOutOfRange_;
        StatisticNumeric<std::uint64_t> * pNumScheduleRejectFrameIndexOutOfRange_;
        StatisticNumeric<std::uint64_t> * pNumScheduleRejectUpdateBeforeFull_;
        StatisticNumeric<std::uint64_t> * pNumScheduleRejectOther_;
        StatisticNumeric<std::uint64_t> * pNumScheduleFullAccept_;
        StatisticNumeric<std::uint64_t> * pNumScheduleUpdateAccept_;

        void flushSchedule();
      };
    }
  }
}

#endif // EMANETDMARADIOMODELEVENTSCHEDULER_HEADER_
