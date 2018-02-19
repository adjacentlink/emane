/*
 * Copyright (c) 2016-2017 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANEOTASTATISTICPUBLISHER_HEADER_
#define EMANEOTASTATISTICPUBLISHER_HEADER_

#include "emane/types.h"
#include "emane/statisticnumeric.h"
#include "emane/statistictable.h"
#include "emane/statisticregistrar.h"

#include <uuid.h>
#include <mutex>
#include <string>
#include <tuple>
#include <map>

namespace EMANE
{
  class OTAStatisticPublisher
  {
  public:
    OTAStatisticPublisher();

    enum class Type
      {
        TYPE_UPSTREAM_PACKET_SUCCESS,
        TYPE_DOWNSTREAM_PACKET_SUCCESS,
        TYPE_UPSTREAM_PACKET_DROP_MISSING_PARTS,
      };

    void update(Type type,
                const uuid_t & uuid,
                NEMId nemId);

    void setRowLimit(size_t rows);

  private:
    using PacketCountTableKey = std::pair<std::string,NEMId>;

    using PacketCountInfo =
      std::map<PacketCountTableKey,
               std::tuple<std::uint64_t, // packets Tx
                          std::uint64_t, // packets Rx
                          std::uint64_t>>; // packets Rx drop missing
                                           // parts

    StatisticNumeric<std::uint64_t> * pNumOTAChannelDownstreamPackets_;
    StatisticNumeric<std::uint64_t> * pNumOTAChannelUpstreamPackets_;
    StatisticNumeric<std::uint64_t> * pNumOTAChannelUpstreamPacketsDroppedMissingPart_;

    StatisticTable<PacketCountTableKey> * pPacketCountTable_;

    PacketCountInfo packetCountInfo_;

    std::mutex mutexPacketCountTable_;

    size_t rowLimit_;
  };
}

#endif //EMANEOTASTATISTICPUBLISHER_HEADER_
