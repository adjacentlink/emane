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

#ifndef EMANETDMABASEMODELIMPL_HEADER_
#define EMANETDMABASEMODELIMPL_HEADER_

#include "emane/maclayerimpl.h"
#include "emane/flowcontrolmanager.h"
#include "emane/models/tdma/basemodel.h"
#include "emane/models/tdma/scheduler.h"
#include "emane/models/tdma/queuemanager.h"

#include "slotstatustablepublisher.h"
#include "receivemanager.h"
#include "packetstatuspublisherimpl.h"

namespace EMANE
{
  namespace Models
  {
    namespace TDMA
    {
      class BaseModel::Implementation : public MACLayerImplementor,
                                        public SchedulerUser
      {
      public:
        Implementation(NEMId id,
                       PlatformServiceProvider *pPlatformServiceProvider,
                       RadioServiceProvider * pRadioServiceProvider,
                       Scheduler * pScheduler,
                       QueueManager * pQueueManager,
                       MACLayerImplementor * pRadioModel);

        ~Implementation();

        void initialize(Registrar & registrar) override;

        void configure(const ConfigurationUpdate & update) override;

        void start() override;

        void postStart() override;

        void stop() override;

        void destroy() throw() override;

        void processUpstreamControl(const ControlMessages & msgs) override;


        void processUpstreamPacket(const CommonMACHeader & hdr,
                                   UpstreamPacket & pkt,
                                   const ControlMessages & msgs) override;

        void processDownstreamControl(const ControlMessages & msgs) override;


        void processDownstreamPacket(DownstreamPacket & pkt,
                                     const ControlMessages & msgs) override;


        void processEvent(const EventId &, const Serialization &) override;

        void processTimedEvent(TimerEventId eventId,
                               const TimePoint & expireTime,
                               const TimePoint & scheduleTime,
                               const TimePoint & fireTime,
                               const void * arg) override;

        void processConfiguration(const ConfigurationUpdate & update) override;

        void notifyScheduleChange(const Frequencies & frequencies,
                                  std::uint64_t u64BandwidthHz,
                                  const Microseconds & slotDuration,
                                  const Microseconds & slotOverhead) override;


        void processSchedulerPacket(DownstreamPacket & pkt) override;

        void processSchedulerControl(const ControlMessages & msgs) override;

        QueueInfos getPacketQueueInfo() const override;

      private:
        std::unique_ptr<Scheduler> pScheduler_;
        std::unique_ptr<QueueManager> pQueueManager_;
        MACLayerImplementor * pRadioModel_;

        bool bFlowControlEnable_;
        std::uint16_t u16FlowControlTokens_;
        bool bRadioMetricEnable_;
        Microseconds radioMetricReportIntervalMicroseconds_;
        std::string sPCRCurveURI_;
        TimerEventId transmitTimedEventId_;
        TxSlotInfo pendingTxSlotInfo_;
        TimePoint  nextMultiFrameTime_;
        TxSlotInfos txSlotInfos_;
        Microseconds slotDuration_;
        Microseconds slotOverhead_;
        SlotStatusTablePublisher slotStatusTablePublisher_;
        std::uint64_t u64SequenceNumber_;
        Frequencies frequencies_;
        std::uint64_t u64BandwidthHz_;
        PacketStatusPublisherImpl packetStatusPublisher_;
        ReceiveManager receiveManager_;
        FlowControlManager flowControlManager_;
        void sendDownstreamPacket(double dSlotRemainingRatio);

        bool processTxOpportunity();
      };
    }
  }
}

#endif // EMANETDMABASEMODELIMPL_HEADER_
