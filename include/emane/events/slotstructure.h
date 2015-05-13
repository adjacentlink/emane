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

#ifndef EMANEEVENTSSLOTSTRUCTURE_HEADER_
#define EMANEEVENTSSLOTSTRUCTURE_HEADER_

#include "emane/types.h"
#include <set>

namespace EMANE
{
  namespace Events
  {
    class SlotStructure
    {
    public:
      SlotStructure(std::uint64_t u64BandwidthHz,
                    std::uint32_t u32FramePerMultiFrame,
                    std::uint32_t u32SlotsPerFrame,
                    const Microseconds & slotDuration,
                    const Microseconds & slotOverhead);

      SlotStructure();

      std::uint64_t getBandwidth() const;

      std::uint32_t getFramesPerMultiFrame() const;

      std::uint32_t getSlotsPerFrame() const;

      const Microseconds & getSlotDuration() const;

      const Microseconds & getSlotOverhead() const;

    private:
      std::uint64_t u64BandwidthHz_;
      std::uint32_t u32FramePerMultiFrame_;
      std::uint32_t u32SlotsPerFrame_;
      Microseconds slotDuration_;
      Microseconds slotOverhead_;

    };
  }
}

#include "emane/events/slotstructure.inl"

#endif // EMANEEVENTSSLOTSTRUCTURE_HEADER_
