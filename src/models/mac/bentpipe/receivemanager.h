/*
 * Copyright (c) 2015,2017-2018,2023 - Adjacent Link LLC, Bridgewater,
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

#ifndef EMANE_MODELS_BENTPIPE_RECEIVEMANAGER_HEADER_
#define EMANE_MODELS_BENTPIPE_RECEIVEMANAGER_HEADER_

#include "transponderconfiguration.h"
#include "transponderpackettransport.h"
#include "bentpipemessage.h"
#include "packetstatuspublisher.h"
#include "neighborstatuspublisher.h"
#include "pcrmanager.h"
#include "bentpipemessage.pb.h"

#include "emane/types.h"
#include "emane/upstreampacket.h"
#include "emane/radioserviceprovider.h"
#include "emane/platformserviceprovider.h"
#include "emane/frequencysegment.h"
#include "emane/utils/randomnumberdistribution.h"

#include <tuple>

namespace EMANE
{
  namespace Models
  {
    namespace BentPipe
    {
      /**
       * @class ReceiveManager
       *
       * @brief Manages all receive side message processing
       *
       * Support aggregate message components and fragmentation
       * reassembly. Handles aggregation and fragmentation of inbound
       * messages even when the radio model is configured to not
       * aggregate or fragment transmissions.
       */
      class ReceiveManager
      {
      public:
        ReceiveManager(NEMId id,
                       TransponderIndex transponderIndex,
                       TransponderPacketTransport * pTransponderPacketTransport,
                       PlatformServiceProvider * pPlatformLogService,
                       RadioServiceProvider * pRadioService,
                       PacketStatusPublisher * pPacketStatusPublisher,
                       NeighborStatusPublisher * pNeighborStatusPublisher,
                       bool bProcess,
                       PCRManager * pPCRManager,
                       AntennaIndex rxAntennaIndex,
                       const std::chrono::seconds & fragmentCheckThreshold,
                       const std::chrono::seconds & fragmentTimeoutThreshold);

        void enqueue(BentPipeMessage && bentPipeMessage,
                     const PacketInfo & pktInfo,
                     size_t length,
                     const TimePoint & startOfReception,
                     const FrequencySegments & frequencySegments,
                     const Microseconds & span,
                     const TimePoint & beginTime,
                     std::uint64_t u64PacketSequence);

        void process();

      private:
        NEMId id_;
        TransponderIndex transponderIndex_;
        TransponderPacketTransport * pTransponderPacketTransport_;
        PlatformServiceProvider * pPlatformService_;
        RadioServiceProvider * pRadioService_;
        PacketStatusPublisher * pPacketStatusPublisher_;
        NeighborStatusPublisher * pNeighborStatusPublisher_;
        bool bProcess_;
        PCRManager * pPCRManager_;
        AntennaIndex rxAntennaIndex_;

        using FrameInfo = std::tuple<BentPipeMessage,
                                     PacketInfo,
                                     size_t,
                                     TimePoint, //sor
                                     FrequencySegments,
                                     Microseconds, // span
                                     TimePoint,
                                     std::uint64_t>; // sequence number

        using PendingQueue = std::map<TimePoint,FrameInfo>;
        PendingQueue pendingQueue_;
        TimePoint lastEndOfReception_;

        Utils::RandomNumberDistribution<std::mt19937,
                                        std::uniform_real_distribution<float>> distribution_;

        std::chrono::seconds fragmentCheckThreshold_;
        std::chrono::seconds fragmentTimeoutThreshold_;

        using FragmentKey = std::tuple<NEMId,std::uint64_t>;
        using FragmentParts = std::map<size_t,std::vector<std::uint8_t>>;
        using FragmentInfo = std::tuple<std::set<size_t>,
                                        FragmentParts,
                                        TimePoint, // last fragment time
                                        NEMId, // destination
                                        size_t>; // total number of fragments
        using FragmentStore = std::map<FragmentKey,FragmentInfo>;

        FragmentStore fragmentStore_;
        TimePoint lastFragmentCheckTime_;
        TimePoint nextEoRCheckTime_;

        ReceiveManager(const ReceiveManager &) = delete;

        ReceiveManager & operator=(const ReceiveManager &) = delete;
      };
    }
  }
}

#endif // EMANE_MODELS_BENTPIPE_RECEIVEMANAGER_HEADER_
