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

#ifndef EMANE_MODELS_BENTPIPE_TRANSPONDERCONFIGURATION_HEADER_
#define EMANE_MODELS_BENTPIPE_TRANSPONDERCONFIGURATION_HEADER_

#include "types.h"
#include "pcrmanager.h"
#include "emane/types.h"

#include <map>
#include <set>

namespace EMANE
{
  namespace Models
  {
    namespace BentPipe
    {
      class TransponderConfiguration
      {
      public:
        TransponderConfiguration(TransponderIndex transponderIndex);

        TransponderConfiguration(const TransponderConfiguration &) = default;

        void setReceiveFrequencyHz(std::uint64_t u64FrequencyHz);
        void setReceiveBandwidthHz(std::uint64_t u64BandwidthHz);
        void setReceiveAntennaIndex(AntennaIndex antennaIndex);
        void setPCRCurveIndex(PCRCurveIndex index);
        void setReceiveAction(ReceiveAction action);
        void setReceiveEnable(bool bEnable);
        void setTransmitFrequencyHz(std::uint64_t u64FrequencyHz);
        void setTransmitBandwidthHz(std::uint64_t u64BandwidthHz);
        void setTransmitDataRatebps(std::uint64_t u64Dataratebps);
        void setTransmitAntennaIndex(AntennaIndex antennaIndex);
        void setTransmitMTUBytes(std::uint64_t u64MTUBytes);
        void setTransmitPowerdBm(double dPowerdBm);
        void setTransmitUbendDelay(const Microseconds & delay);
        void setTransmitProcessTOS(const TOSSet & tosSet);
        void setTransmitSlotsPerFrame(std::uint16_t u16TransmitSlotsPerFrame);
        void setTransmitSlotSize(const Microseconds & slotSize);
        void setTransmitSlots(const TransmitSlots & slots);
        void setTransmitEnable(bool bEnable);

        TransponderIndex getTransponderIndex() const;
        std::uint64_t getReceiveFrequencyHz() const;
        std::uint64_t getReceiveBandwidthHz() const;
        AntennaIndex getReceiveAntennaIndex() const;
        PCRCurveIndex getPCRCurveIndex() const;
        ReceiveAction getReceiveAction() const;
        bool getReceiveEnable() const;
        std::uint64_t getTransmitFrequencyHz() const;
        std::uint64_t getTransmitBandwidthHz() const;
        std::uint64_t getTransmitDataRatebps() const;
        AntennaIndex getTransmitAntennaIndex() const;
        std::uint64_t getTransmitMTUBytes() const;

        double getTransmitPowerdBm() const;
        const Microseconds & getTransmitUbendDelay() const;
        const TOSSet & getTransmitProcessTOS() const;
        std::uint16_t getTransmitSlotsPerFrame() const;
        const Microseconds & getTransmitSlotSize() const;
        const TransmitSlots & getTransmitSlots() const;
        bool getTransmitEnable() const;

      private:
        TransponderIndex transponderIndex_;
        std::uint64_t u64ReceiveFrequencyHz_;
        std::uint64_t u64ReceiveBandwidthHz_;
        AntennaIndex  receiveAntennaIndex_;
        PCRCurveIndex curveIndex_;
        ReceiveAction receiveAction_;
        std::uint64_t u64TransmitFrequencyHz_;
        std::uint64_t u64TransmitBandwidthHz_;
        std::uint64_t u64TransmitDataRatebps_;
        AntennaIndex  transmitAntennaIndex_;
        double dTransmitPowerdBm_;
        Microseconds transmitUbendDelay_;
        TOSSet transmitProcessTOS_;
        std::uint16_t u16TransmitSlotsPerFrame_;
        Microseconds transmitSlotSize_;
        TransmitSlots transmitSlots_;
        std::uint64_t u64TransmitMTUBytes_;
        bool bReceiveEnable_;
        bool bTransmitEnable_;
      };

      using TransponderConfigurations =
        std::map<TransponderIndex,
                 TransponderConfiguration>;
    }
  }
}

#endif // EMANE_MODELS_BENTPIPE_TRANSPONDERCONFIGURATION_HEADER_
