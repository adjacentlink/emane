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

#ifndef EMANETDMARADIOMODELTYPES_HEADER_
#define EMANETDMARADIOMODELTYPES_HEADER_

#include "emane/types.h"

#include <vector>
#include <set>
#include <list>

namespace EMANE
{
  namespace Models
  {
    namespace TDMA
    {
      using SlotIndex = uint64_t;
      using FrameIndex = uint64_t;

      struct PacketMetaInfo
      {
        NEMId source_;
        SlotIndex slot_;
        double dRxPowerdBm_;
        double dSINR_;
        std::uint64_t u64DataRatebps_;
      };

      struct QueueInfo
      {
        std::uint8_t u8QueueId_;
        std::uint64_t u64Packets_;
        std::uint64_t u64Bytes_;
      };

      struct TxSlotInfo
      {
        std::uint64_t u64AbsoluteSlotIndex_;
        std::uint32_t u32RelativeIndex_;
        std::uint32_t u32RelativeSlotIndex_;
        std::uint32_t u32RelativeFrameIndex_;
        TimePoint timePoint_;
        std::uint64_t u64FrequencyHz_;
        std::uint64_t u64DataRatebps_;
        std::uint8_t  u8QueueId_;
        double dPowerdBm_;
        NEMId destination_;
      };

      struct RxSlotInfo
      {
        SlotIndex u64AbsoluteSlotIndex_;
        std::uint32_t u32RelativeIndex_;
        std::uint32_t u32RelativeSlotIndex_;
        std::uint32_t u32RelativeFrameIndex_;
        TimePoint timePoint_;
        std::uint64_t u64FrequencyHz_;
      };

      struct SlotInfo
      {
        enum class Type{TX,RX,IDLE};
        SlotIndex u64AbsoluteSlotIndex_;
        std::uint32_t u32RelativeIndex_;
        std::uint32_t u32RelativeSlotIndex_;
        std::uint32_t u32RelativeFrameIndex_;
        TimePoint timePoint_;
        Type type_;
      };

      using QueueInfos = std::vector<QueueInfo>;

      using TxSlotInfos = std::list<TxSlotInfo>;

      using Frequencies = std::set<std::uint64_t>;
    }
  }
}

#endif // EMANETDMARADIOMODELTYPES_HEADER_
