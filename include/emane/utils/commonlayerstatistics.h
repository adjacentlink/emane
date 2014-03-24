/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANEUTILSCOMMONLAYERSTATISTICS_HEADER_
#define EMANEUTILSCOMMONLAYERSTATISTICS_HEADER_

#include "emane/downstreampacket.h"
#include "emane/upstreampacket.h"
#include "emane/statisticregistrar.h"
#include "emane/types.h"

#include <memory>

namespace EMANE
{
  namespace Utils
  {
    /**
     * @class CommonLayerStatistics
     *
     * @brief Common NEM layer statistics and drop reason tables
     */
    class CommonLayerStatistics
    {
    public:
      /**
       * Creates a CommonLayerStatistics instance
       *
       * @param unicastDropTableLabels Unicast drop table row labels
       * @param broadcastDropTableLabels Broadcast drop table row labels.
       * Defaults to Unicast drop table labels.
       * @param sInstance String to append to all statistic names
       */
      CommonLayerStatistics(const StatisticTableLabels & unicastDropTableLabels,
                            const StatisticTableLabels & broadcastDropTableLabels = {},
                            const std::string & sInstance = {});

      /**
       * Destroys an instance
       */
      ~CommonLayerStatistics();

      /**
       * Registers all statistics and tables
       *
       * @param statisticRegistrar Statistic registrar reference
       *
       * @throw RegistrarException when a error occurs during
       * registration.
       */
      void registerStatistics(StatisticRegistrar & statisticRegistrar);

      /**
       * Processes statistics for an inbound upstream packet 
       *
       * @param pkt Inbound packet
       */
      void processInbound(const UpstreamPacket & pkt);


      /**
       * Processes statistics for an outbound upstream packet
       *
       * @param pkt Inbound packet
       * @param delay Processing duration in microseconds
       * @param dropCode Drop code label index. Default is no drop.
       */
      void processOutbound(const UpstreamPacket & pkt, Microseconds delay, size_t dropCode = {});

      /**
       * Processes statistics for an inbound downstream packet 
       *
       * @param pkt Outbound packet
       */
      void processInbound(const DownstreamPacket & pkt);

      /**
       * Processes statistics for an outbound downstream packet 
       *
       * @param pkt Inbound packet
       * @param delay Processing duration in microseconds
       * @param dropCode Drop code label index. Default is no drop.
       * @param bSelfGenerated Flag indicating whether the outbound
       * packet was self generated
       */
      void processOutbound(const DownstreamPacket & pkt, 
                           Microseconds delay,
                           size_t dropCode = {}, 
                           bool bSelfGenerated = {});

    private:
      class Implementation;
      std::unique_ptr<Implementation> pImpl_;

      CommonLayerStatistics(const CommonLayerStatistics &) = delete;
      CommonLayerStatistics & operator = (const CommonLayerStatistics &) = delete;
    };
  }
}


#endif // EMANEUTILSCOMMONLAYERSTATISTICS_HEADER_
