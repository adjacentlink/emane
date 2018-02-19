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

#ifndef EMANEMODELSTDMAQUEUEMANAGER_HEADER_
#define EMANEMODELSTDMAQUEUEMANAGER_HEADER_

#include "emane/component.h"
#include "emane/platformserviceuser.h"
#include "emane/runningstatemutable.h"
#include "emane/downstreampacket.h"
#include "emane/models/tdma/types.h"
#include "emane/models/tdma/messagecomponent.h"
#include "emane/models/tdma/packetstatuspublisheruser.h"

#include <tuple>

namespace EMANE
{
  namespace Models
  {
    namespace TDMA
    {
      /**
       * @class QueueManager
       *
       * @brief %Queue management interface used by BaseModel
       */
      class QueueManager : public Component,
                           public PlatformServiceUser,
                           public RunningStateMutable,
                           public PacketStatusPublisherUser
      {
      public:
         /**
         * Destroys an instance.
         */
        virtual ~QueueManager(){};

        /**
         * Enqueues a packet in to a specified queue.
         *
         * @param u8QueueIndex %Queue index of target queue
         * @param pkt Packet to enqueue
         *
         * @return The number of packets dropped as a result of this
         * enqueue. This value would be non-zero in the case of
         * overflow.
         */
        virtual
        size_t enqueue(std::uint8_t u8QueueIndex, DownstreamPacket && pkt) = 0;


        /**
         * Dequeues a specified number of bytes from a specified queue.
         *
         * @param u8QueueIndex %Queue index of target queue @param
         * length length to dequeue in bytes
         * @param destination NEM destination to dequeue, 0 indicates
         * any NEM.
         *
         * @return A tuple containing a list of MessageComponent
         * entries and the the total size in bytes across all entries.
         *
         * @note Unless restricted by configuration, the dequeue
         * operation will search multiple queues (highest priority
         * first) in order to return the specified number of bytes. If
         * fragmentation is disabled, all packets in the initially
         * specified queue that are larger than the requested size
         * will be discarded.
         */
        virtual std::tuple<EMANE::Models::TDMA::MessageComponents,
                           size_t>
        dequeue(std::uint8_t u8QueueIndex,
                size_t length,
                NEMId destination) = 0;


        /**
         * Gets queue status.
         *
         * @return A list of QueueInfo entries.
         */
        virtual QueueInfos getPacketQueueInfo() const = 0;

        virtual double getImplQSDelay() = 0;

      protected:
        NEMId id_;

        /**
         * Creates an instance.
         */
        QueueManager(NEMId id,
                     PlatformServiceProvider * pPlatformServiceProvider):
          PlatformServiceUser{pPlatformServiceProvider},
          id_{id}{}

      private:
        void processEvent(const EventId &, const Serialization &) final{};

        void processTimedEvent(TimerEventId,
                               const TimePoint &,
                               const TimePoint &,
                               const TimePoint &,
                               const void *) final {};

      };
    }
  }
}

#endif // EMANEMODELSTDMAQUEUEMANAGER_HEADER_
