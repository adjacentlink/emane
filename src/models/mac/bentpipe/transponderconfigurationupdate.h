/*
 * Copyright (c) 2023 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANE_MODELS_BENTPIPE_TRANSPONDERCONFIGURATIONUPDATE_HEADER_
#define EMANE_MODELS_BENTPIPE_TRANSPONDERCONFIGURATIONUPDATE_HEADER_

#include "types.h"
#include "emane/types.h"

#include <optional>
#include <map>

namespace EMANE
{
  namespace Models
  {
    namespace BentPipe
    {
      class TransponderConfigurationUpdate
      {
      public:
        TransponderConfigurationUpdate(TransponderIndex transponderIndex);

        TransponderConfigurationUpdate(const TransponderConfigurationUpdate &) = default;

        void setReceiveFrequencyHz(std::uint64_t u64FrequencyHz);
        void setReceiveEnable(bool bEnable);
        void setPCRCurveIndex(PCRCurveIndex index);
        void setTransmitFrequencyHz(std::uint64_t u64FrequencyHz);
        void setTransmitDataRatebps(std::uint64_t u64Dataratebps);
        void setTransmitPowerdBm(double dPowerdBm);
        void setTransmitUbendDelay(const Microseconds & delay);
        void setTransmitUbendJitter(const Microseconds & jitter);
        void setTransmitSlotsPerFrame(std::uint16_t u16TransmitSlotsPerFrame);
        void setTransmitSlotSize(const Microseconds & slotSize);
        void setTransmitSlots(const TransmitSlots & slots);
        void setTransmitMTUBytes(std::uint64_t u64MTUBytes);
        void setTransmitEnable(bool bEnable);

        TransponderIndex getTransponderIndex() const;
        const std::optional<std::uint64_t> & getReceiveFrequencyHz() const;
        const std::optional<bool> & getReceiveEnable() const;
        const std::optional<PCRCurveIndex> & getPCRCurveIndex() const;
        const std::optional<std::uint64_t> & getTransmitFrequencyHz() const;
        const std::optional<std::uint64_t> & getTransmitDataRatebps() const;
        const std::optional<double> & getTransmitPowerdBm() const;
        const std::optional<Microseconds> & getTransmitUbendDelay() const;
        const std::optional<Microseconds> & getTransmitUbendJitter() const;
        const std::optional<std::uint16_t> & getTransmitSlotsPerFrame() const;
        const std::optional<Microseconds> & getTransmitSlotSize() const;
        const std::optional<TransmitSlots> & getTransmitSlots() const;
        const std::optional<std::uint64_t> & getTransmitMTUBytes() const;
        const std::optional<bool> & getTransmitEnable() const;

      private:
        TransponderIndex transponderIndex_;
        std::optional<std::uint64_t> u64ReceiveFrequencyHz_;
        std::optional<PCRCurveIndex> curveIndex_;
        std::optional<std::uint64_t> u64TransmitFrequencyHz_;
        std::optional<std::uint64_t> u64TransmitDataRatebps_;
        std::optional<double> dTransmitPowerdBm_;
        std::optional<Microseconds> transmitUbendDelay_;
        std::optional<Microseconds>  transmitUbendJitter_;
        std::optional<std::uint16_t> u16TransmitSlotsPerFrame_;
        std::optional<Microseconds> transmitSlotSize_;
        std::optional<TransmitSlots> transmitSlots_;
        std::optional<std::uint64_t> u64TransmitMTUBytes_;
        std::optional<bool> bReceiveEnable_;
        std::optional<bool> bTransmitEnable_;
      };

      using TransponderConfigurationUpdates =
        std::map<TransponderIndex,
                 TransponderConfigurationUpdate>;
    }
  }
}

#endif // EMANE_MODELS_BENTPIPE_TRANSPONDERCONFIGURATIONUPDATE_HEADER_
