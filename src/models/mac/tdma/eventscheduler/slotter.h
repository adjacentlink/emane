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

#ifndef EMANEMODELSTDMASLOTTER_HEADER_
#define EMANEMODELSTDMASLOTTER_HEADER_

#include "emane/types.h"

namespace EMANE
{
  namespace Models
  {
    namespace TDMA
    {
      class Slotter
      {
      public:
        Slotter();

        void reset(const EMANE::Microseconds & slotSizeMicroseconds,
                   std::uint64_t u32SlotsPerFrame,
                   std::uint64_t u32FramesPerMultiFrame);

        TimePoint getMultiFrameTime(std::uint64_t u64MultiFrameIndex) const;

        TimePoint getFrameTime(std::uint64_t u64FrameIndex) const;

        TimePoint getSlotTime(std::uint64_t u64SlotIndex) const;

        std::tuple<std::uint64_t,std::uint64_t,std::uint64_t>
        getAbsoluteIndex(const EMANE::TimePoint & timePoint) const;

        std::tuple<std::uint32_t,std::uint32_t>
        getRelativeIndex(std::uint64_t u64SlotIndex) const;

      private:
        std::uint64_t u64SlotSizeMicroseconds_;
        std::uint64_t u32SlotsPerFrame_;
        std::uint64_t u32FramesPerMultiFrame_;

      };
    }
  }
}

#include "slotter.inl"

#endif // EMANEMODELSTDMASLOTTER_HEADER_
