/*
 * Copyright (c) 2015-2016,2023 - Adjacent Link LLC, Bridgewater,
 *  New Jersey
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

#ifndef EMANE_MODELS_BENTPIPE_PACKETSTATUSPUBLISHER_HEADER_
#define EMANE_MODELS_BENTPIPE_PACKETSTATUSPUBLISHER_HEADER_

#include "messagecomponent.h"
#include "emane/types.h"
#include "emane/statisticregistrar.h"

#include <array>
#include <mutex>
#include <map>

namespace EMANE
{
  namespace Models
  {
    namespace BentPipe
    {
      class PacketStatusPublisher
      {
      public:
        PacketStatusPublisher();

        ~PacketStatusPublisher();

        void registerStatistics(StatisticRegistrar & registrar);

        enum class InboundAction
          {
            ACCEPT_GOOD, /**< Accepted and sent upstream*/
            DROP_BAD_CONTROL,/**< Dropped bad control message*/
            DROP_MISS_FRAGMENT,/**< Dropped one or more fragments missing*/
            DROP_SPECTRUM_SERVICE,/**< Dropped due to Spectrum service query error*/
            DROP_SINR,/**< Dropped due to SINR*/
            DROP_REGISTRATION_ID,/**< Dropped not for this radio model*/
            DROP_DESTINATION_MAC,/**< Dropped not for this NEM*/
            DROP_TOO_LONG,/**< Dropped packet propagation plus duration more than a slot*/
            DROP_BAD_CURVE,
            DROP_LOCK,
            DROP_RX_OFF,
          };

        /**
         * OUtbound packet (downstream> status codes
         */
        enum class OutboundAction
          {
            ACCEPT_GOOD, /**< Accepted and sent downstream */
            DROP_TOO_BIG, /**< Dropped too big and fragmentation disabled */
            DROP_OVERFLOW, /**< Dropped queue overflow */
            DROP_FLOW_CONTROL, /**< Dropped flow control error */
            DROP_TX_OFF,
          };


        void inbound(NEMId src,
                     const MessageComponent & component,
                     InboundAction action);

        void inbound(NEMId src,
                     const MessageComponents & components,
                     InboundAction action);

        void inbound(NEMId src,
                     NEMId dst,
                     size_t size,
                     InboundAction action);

        void outbound(NEMId src,
                      NEMId dst,
                      size_t size,
                      OutboundAction action);

        void outbound(NEMId src,
                      const MessageComponents & components,
                      OutboundAction action);

      private:
        enum {QUEUE_COUNT = 1};

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
                                                         std::uint64_t, // Too Long
                                                         std::uint64_t, // Frequency
                                                         std::uint64_t, // Slot Error
                                                         std::uint64_t,// Miss Fragment
                                                         std::uint64_t, // Bad Curve
                                                         std::uint64_t>>;  // Lock

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

#endif // EMANE_MODELS_BENTPIPE_PACKETSTATUSPUBLISHER_HEADER_
