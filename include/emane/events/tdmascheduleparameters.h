/*
 * Copyright (c) 2025 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANEEVENETSTDMASECHDULEPARAMETERS_HEADER_
#define EMANEEVENETSTDMASECHDULEPARAMETERS_HEADER_

#include "emane/types.h"

#include <optional>
#include <variant>
#include <map>

namespace EMANE
{
  namespace Events
  {
    namespace TDMASchedule
    {
      struct MultiFrameParameters
      {
        std::optional<std::uint64_t> u64FrequencyHz_;
        std::optional<double> dPowerdBm_;
        std::optional<std::uint8_t> u8ServiceClass_;
        std::optional<std::uint64_t> u64Dataratebps_;

        void reset()
        {
          u64FrequencyHz_.reset();
          dPowerdBm_.reset();
          u8ServiceClass_.reset();
          u64Dataratebps_.reset();
        }

      };

      struct FrameParameters
      {
        std::optional<std::uint64_t> u64FrequencyHz_;
        std::optional<double> dPowerdBm_;
        std::optional<std::uint8_t> u8ServiceClass_;
        std::optional<std::uint64_t> u64Dataratebps_;
      };

      struct TxSlotParameters
      {
        std::optional<std::uint64_t> u64FrequencyHz_;
        std::optional<double> dPowerdBm_;
        std::optional<std::uint8_t> u8ServiceClass_;
        std::optional<std::uint64_t> u64Dataratebps_;
        std::optional<NEMId> destination_;
      };

      struct RxSlotParameters
      {
        std::optional<std::uint64_t> u64FrequencyHz_;
      };

      struct IdleSlotParameters
      {};

      struct SlotParameters
      {
        using Parameters = std::variant<TxSlotParameters,
                                        RxSlotParameters,
                                        IdleSlotParameters>;
        Parameters parameters_;
      };

      using SlotParameterMap =
        std::map<std::uint32_t, // slot index
                 SlotParameters>;

      using FrameSlotMap =
        std::map<std::uint32_t, // frame index
                 SlotParameterMap>;

      using ScheduleNEMMap =
        std::map<NEMId,
                 FrameSlotMap>;

      using FrameParameterMap =
        std::map<std::uint32_t, // frame index
                 FrameParameters>;
    }
  }
}

#endif // EMANEEVENETSTDMASECHDULEPARAMETERS_HEADER_
