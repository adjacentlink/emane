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

#include "emane/types.h"
#include <set>

inline
EMANE::Events::SlotStructure::SlotStructure(std::uint64_t u64BandwidthHz,
                                            std::uint32_t u32FramePerMultiFrame,
                                            std::uint32_t u32SlotsPerFrame,
                                            const Microseconds & slotDuration,
                                            const Microseconds & slotOverhead):
  u64BandwidthHz_{u64BandwidthHz},
  u32FramePerMultiFrame_{u32FramePerMultiFrame},
  u32SlotsPerFrame_{u32SlotsPerFrame},
  slotDuration_{slotDuration},
  slotOverhead_{slotOverhead}{}

inline
EMANE::Events::SlotStructure::SlotStructure():
  u64BandwidthHz_{},
  u32FramePerMultiFrame_{},
  u32SlotsPerFrame_{},
  slotDuration_{},
  slotOverhead_{}{}

inline
std::uint64_t EMANE::Events::SlotStructure::getBandwidth() const
{
  return u64BandwidthHz_;
}

inline
std::uint32_t EMANE::Events::SlotStructure::getFramesPerMultiFrame() const
{
  return u32FramePerMultiFrame_;
}

inline
std::uint32_t EMANE::Events::SlotStructure::getSlotsPerFrame() const
{
  return u32SlotsPerFrame_;;
}

inline
const EMANE::Microseconds & EMANE::Events::SlotStructure::getSlotDuration() const
{
  return slotDuration_;
}

inline
const EMANE::Microseconds & EMANE::Events::SlotStructure::getSlotOverhead() const
{
  return slotOverhead_;
}
