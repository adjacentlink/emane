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
EMANE::Events::SlotInfo::SlotInfo(Type type,
                                   std::uint32_t u32FrameIndex,
                                  std::uint32_t u32SlotIndex,
                                  std::uint64_t u64FrequencyHz,
                                  std::uint64_t u64DataRatebps,
                                  std::uint8_t u8ServiceClass,
                                  double dPowerdBm,
                                  NEMId destination):
  type_{type},
  u32FrameIndex_{u32FrameIndex},
  u32SlotIndex_{u32SlotIndex},
  u64FrequencyHz_{u64FrequencyHz},
  u64DataRatebps_{u64DataRatebps},
  u8ServiceClass_{u8ServiceClass},
  dPowerdBm_{dPowerdBm},
  destination_{destination}{}

inline
EMANE::Events::SlotInfo::Type EMANE::Events::SlotInfo::getType() const
{
  return type_;
}

inline
std::uint32_t EMANE::Events::SlotInfo::getFrameIndex() const
{
  return u32FrameIndex_;
}

inline
std::uint32_t EMANE::Events::SlotInfo::getSlotIndex() const
{
  return u32SlotIndex_;
}

inline
std::uint64_t EMANE::Events::SlotInfo::getFrequency() const
{
  return u64FrequencyHz_;
}

inline
std::uint64_t EMANE::Events::SlotInfo::getDataRate() const
{
  return u64DataRatebps_;
}

inline
std::uint8_t EMANE::Events::SlotInfo::getServiceClass() const
{
  return u8ServiceClass_;
}

inline
double EMANE::Events::SlotInfo::getPower() const
{
  return dPowerdBm_;
}

inline
EMANE::NEMId EMANE::Events::SlotInfo::getDestination() const
{
  return destination_;
}

