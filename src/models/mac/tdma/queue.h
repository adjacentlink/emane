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

#ifndef EMANEMODELSTDMAQUEUE_HEADER_
#define EMANEMODELSTDMAQUEUE_HEADER_

#include "emane/types.h"
#include "emane/downstreampacket.h"
#include "emane/models/tdma/messagecomponent.h"

#include <cstdint>
#include <map>

namespace EMANE
{
  namespace Models
  {
    namespace TDMA
    {
      /**
       * @class Queue
       *
       * @brief Downstream packet queue with both FIFO and
       * By-Destination FIFO dequeue mechanisms.
       */
      class Queue
      {
      public:
        Queue();

        void initialize(std::uint16_t u16QueueDepth,bool bFragment,bool bIsControl);


        std::pair<std::unique_ptr<DownstreamPacket>,bool>
        enqueue(DownstreamPacket && pkt);

        std::tuple<MessageComponents,
                   size_t,
                   std::list<std::unique_ptr<DownstreamPacket>>>
          dequeue(size_t requestedBytes,NEMId destination,bool bDrop);

        // packets, bytes
        std::tuple<size_t,size_t> getStatus() const;

      private:
        class MetaInfo
        {
        public:
          size_t index_{};
          size_t offset_{};
        };
        using PacketQueue = std::map<std::uint64_t,
                                     std::pair<DownstreamPacket *,MetaInfo *>>;
        PacketQueue queue_;
        std::map<NEMId,PacketQueue> destQueue_;
        std::uint16_t u16QueueDepth_;
        bool bFragment_;
        std::uint64_t u64Counter_;
        size_t currentBytes_;
        bool bIsControl_;

        std::pair<MessageComponent,size_t> fragmentPacket(DownstreamPacket * pPacket,
                                                          MetaInfo * pMetaInfo,
                                                          std::uint64_t u64Sequence,
                                                          size_t bytes);
      };
    }
  }
}

#endif // EMANEMODELSTDMAQUEUE_HEADER_
