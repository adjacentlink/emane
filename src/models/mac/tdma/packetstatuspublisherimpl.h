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

#ifndef EMANEMODELSTDMAPACKETSTATUSPUBLISHERIMPL_HEADER_
#define EMANEMODELSTDMAPACKETSTATUSPUBLISHERIMPL_HEADER_

#include "emane/statisticregistrar.h"
#include "emane/models/tdma/packetstatuspublisher.h"

#include <array>
#include <mutex>
#include <map>

namespace EMANE
{
  namespace Models
  {
    namespace TDMA
    {
      /**
       * @class PacketStatusPublisherImpl
       *
       * @brief Implementation of PacketStatusPublisher
       */
      class PacketStatusPublisherImpl : public PacketStatusPublisher
      {
      public:
        PacketStatusPublisherImpl();

        ~PacketStatusPublisherImpl();

        void registerStatistics(StatisticRegistrar & registrar);

        void inbound(NEMId src,
                     const MessageComponent & component,
                     InboundAction action) override;

        void inbound(NEMId src,
                     const MessageComponents & components,
                     InboundAction action) override;

        void inbound(NEMId src,
                     NEMId dst,
                     Priority priority,
                     size_t size,
                     InboundAction action) override;

        void outbound(NEMId src,
                      NEMId dst,
                      Priority priority,
                      size_t size,
                      OutboundAction action) override;

        void outbound(NEMId src,
                      const MessageComponents & components,
                      OutboundAction action) override;

      private:
        enum {QUEUE_COUNT = 5};

        using PacketAcceptInfo = std::map<NEMId,std::tuple<std::uint64_t, // bytes Tx
                                                           std::uint64_t>>; // bytes Rx

        using PacketDropInfo = std::map<NEMId,std::tuple<std::uint64_t, // SINR
                                                         std::uint64_t, // Reg Id
                                                         std::uint64_t, // Dst MAC
                                                         std::uint64_t, // Queue Overflow
                                                         std::uint64_t, // Bad Control
                                                         std::uint64_t, // Bad Spectrum Query
                                                         std::uint64_t, // Flow Control
                                                         std::uint64_t, // Too Big
                                                         std::uint64_t, // Slot Error
                                                         std::uint64_t>>; // Miss Fragment

        using TableArray = std::array<StatisticTable<NEMId> *,QUEUE_COUNT>;

        TableArray broadcastAcceptTables_;
        TableArray broadcastDropTables_;

        TableArray unicastAcceptTables_;
        TableArray unicastDropTables_;

        using AcceptInfoArrary = std::array<PacketAcceptInfo,QUEUE_COUNT>;
        using DropInfoArrary = std::array<PacketDropInfo,QUEUE_COUNT>;

        AcceptInfoArrary broadcastAcceptInfos_;
        DropInfoArrary  broadcastDropInfos_;

        AcceptInfoArrary unicastAcceptInfos_;
        DropInfoArrary  unicastDropInfos_;

        std::mutex mutexBroadcastPacketAcceptTable_;
        std::mutex mutexUnicastPacketAcceptTable_;
        std::mutex mutexBroadcastPacketDropTable_;
        std::mutex mutexUnicastPacketDropTable_;
      };
    }
  }
}

#endif // EMANEMODELSTDMAPACKETSTATUSPUBLISHERIMPL_HEADER_
