/*
 * Copyright (c) 2015,2023 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANE_MODELS_BENTPIPE_QUEUEMANAGER_HEADER_
#define EMANE_MODELS_BENTPIPE_QUEUEMANAGER_HEADER_

#include "types.h"
#include "queueinfo.h"
#include "transponderconfiguration.h"
#include "messagecomponent.h"
#include "queue.h"
#include "queuestatuspublisher.h"
#include "packetstatuspublisher.h"

#include "emane/component.h"
#include "emane/platformserviceuser.h"
#include "emane/runningstatemutable.h"
#include "emane/downstreampacket.h"

namespace EMANE
{
  namespace Models
  {
    namespace BentPipe
    {
      class QueueManager : public Component,
                           public RunningStateMutable
      {
      public:
        QueueManager(NEMId id,
                     PlatformServiceProvider * pPlatformServiceProvider,
                     PacketStatusPublisher * pPacketStatusPublisher);

        ~QueueManager();

        void initialize(Registrar & registrar) override;

        void configure(const ConfigurationUpdate & update) override;

        void start() override;

        void postStart() override;

        void addQueue(TransponderIndex transponderIndex);

        void removeQueue(TransponderIndex transponderIndex);

        void stop() override;

        void destroy() throw() override;

        size_t enqueue(TransponderIndex transponderIndex, DownstreamPacket && pkt);

        std::tuple<MessageComponents,
                   size_t>
        dequeue(TransponderIndex transponderIndex,
                size_t length);

        QueueInfos getPacketQueueInfo() const;

        static constexpr const char * CONFIG_PREFIX{"queue."};

      private:
        NEMId id_;
        PlatformServiceProvider * pPlatformService_;
        PacketStatusPublisher * pPacketStatusPublisher_;
        std::uint16_t u16QueueDepth_;
        bool bAggregationEnable_;
        bool bFragmentationEnable_;

        using Queues = std::map<TransponderIndex,std::unique_ptr<Queue>>;
        Queues queues_;

        QueueStatusPublisher queueStatusPublisher_;
      };
    }
  }
}

#endif // EMANE_MODELS_BENTPIPE_QUEUEMANAGER_HEADER_
