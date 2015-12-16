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

#ifndef EMANEEVENTSSLOTINFO_HEADER_
#define EMANEEVENTSSLOTINFO_HEADER_

#include "emane/types.h"
#include <vector>

namespace EMANE
{
  namespace Events
  {
    class SlotInfo
    {
    public:
      enum class Type{TX,RX,IDLE};

      SlotInfo(Type type,
               std::uint32_t u32FrameIndex,
               std::uint32_t u32SlotIndex,
               std::uint64_t u64FrequencyHz = 0,
               std::uint64_t u64DataRatebps_ = 0,
               std::uint8_t u8ServiceClass_ = 0,
               double dPowerdBm_ = 0,
               NEMId destination = 0);


      Type getType() const;

      std::uint32_t getFrameIndex() const;

      std::uint32_t getSlotIndex() const;

      std::uint64_t getFrequency() const;

      std::uint64_t getDataRate() const;

      std::uint8_t getServiceClass() const;

      double getPower() const;

      NEMId getDestination() const;

    private:
      Type type_;
      std::uint32_t u32FrameIndex_;
      std::uint32_t u32SlotIndex_;
      std::uint64_t u64FrequencyHz_;
      std::uint64_t u64DataRatebps_;
      std::uint8_t u8ServiceClass_;
      double dPowerdBm_;
      NEMId destination_;
    };

    using SlotInfos = std::vector<SlotInfo>;
  }
}

#include "emane/events/slotinfo.inl"

#endif // EMANEEVENTSSLOTINFO_HEADER_
