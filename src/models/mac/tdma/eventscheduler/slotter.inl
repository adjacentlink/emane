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

inline
EMANE::Models::TDMA::Slotter::Slotter():
  u64SlotSizeMicroseconds_{},
  u32SlotsPerFrame_{},
  u32FramesPerMultiFrame_{}{}

inline
void EMANE::Models::TDMA::Slotter::reset(const Microseconds & slotSizeMicroseconds,
                                         std::uint64_t u32SlotsPerFrame,
                                         std::uint64_t u32FramesPerMultiFrame)
{
  u64SlotSizeMicroseconds_ = slotSizeMicroseconds.count();
  u32SlotsPerFrame_ = u32SlotsPerFrame;
  u32FramesPerMultiFrame_ = u32FramesPerMultiFrame;
}

inline
EMANE::TimePoint
EMANE::Models::TDMA::Slotter::getMultiFrameTime(std::uint64_t u64MultiFrameIndex) const
{
  return TimePoint{Microseconds{u64MultiFrameIndex *
        u32FramesPerMultiFrame_ *
        u32SlotsPerFrame_ *
        u64SlotSizeMicroseconds_}};
}

inline
EMANE::TimePoint EMANE::Models::TDMA::Slotter::getFrameTime(std::uint64_t u64FrameIndex) const
{
  return TimePoint{Microseconds{u64FrameIndex *
        u32SlotsPerFrame_ *
        u64SlotSizeMicroseconds_}};
}

inline
EMANE::TimePoint EMANE::Models::TDMA::Slotter::getSlotTime(std::uint64_t u64SlotIndex) const
{
  return TimePoint{Microseconds{u64SlotIndex *
        u64SlotSizeMicroseconds_}};
}

inline
std::tuple<std::uint64_t,std::uint64_t,std::uint64_t>
EMANE::Models::TDMA::Slotter::getAbsoluteIndex(const EMANE::TimePoint & timePoint) const
{
  std::uint64_t count = std::chrono::duration_cast<Microseconds>(timePoint.time_since_epoch()).count();

  std::uint64_t slotIndex{count / u64SlotSizeMicroseconds_};

  std::uint64_t frameIndex{slotIndex / u32SlotsPerFrame_};

  std::uint64_t multiFrameIndex{frameIndex / u32FramesPerMultiFrame_};

  return std::make_tuple(slotIndex,frameIndex,multiFrameIndex);
}

inline
std::tuple<std::uint32_t,std::uint32_t>
EMANE::Models::TDMA::Slotter::getRelativeIndex(std::uint64_t u64SlotIndex) const
{
  return std::make_tuple(u64SlotIndex % u32SlotsPerFrame_,
                         u64SlotIndex / u32SlotsPerFrame_ % u32FramesPerMultiFrame_);
}
