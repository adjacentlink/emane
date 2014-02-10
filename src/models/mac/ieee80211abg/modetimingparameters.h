/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2008 - DRS CenGen, LLC, Columbia, Maryland
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
 * * Neither the name of DRS CenGen, LLC nor the names of its
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

#ifndef EMANEMODELSIEEE802ABGMODETIMINGPARAMTERS_HEADER_
#define EMANEMODELSIEEE802ABGMODETIMINGPARAMTERS_HEADER_

#include "emane/types.h"

#include "macconfig.h"

#include <map>


namespace EMANE
 {
  namespace Models
   {
    namespace IEEE80211ABG
     {
       class MACLayer;

       /**
        *
        * @brief class used to define timing parameters
        *
        */
          class ModeTimingParameters
          {
            private:
              const MACConfig & macConfig_;

            /**
            *
            * @brief structure used to define timing parameters for each mode
            *
            */
            struct TimingParams
            {
              std::uint16_t u16RtsBitLength_;          // rts bit length
              std::uint16_t u16CtsBitLength_;          // cts bit length
              std::uint16_t u16AckBitLength_;          // ack bit length
              Microseconds  slotMicroseconds_;         // slot duration
              Microseconds  sifsMicroseconds_;         // sifs duration
              Microseconds  preambleMicroseconds_;     // preamble duration

                TimingParams(): 
                 u16RtsBitLength_{},
                 u16CtsBitLength_{},
                 u16AckBitLength_{}, 
                 slotMicroseconds_{},
                 sifsMicroseconds_{}, 
                 preambleMicroseconds_{}
              { }
            };

            TimingParams timingParams_[MODULATION_TYPE_INDEX_MAX + 1];

            std::uint16_t getRtsBitLength(MODULATION_TYPE) const;

            std::uint16_t getCtsBitLength(MODULATION_TYPE) const;

            std::uint16_t getAckBitLength(MODULATION_TYPE) const;


            Microseconds getSifsMicroseconds(MODULATION_TYPE) const;

            Microseconds getPreambleMicroseconds(MODULATION_TYPE) const;

            Microseconds getPropagationMicroseconds(std::uint32_t) const;


            Microseconds getBroadcastMessageDurationMicroseconds(size_t) const;

            Microseconds getUnicastMessageDurationMicroseconds(size_t) const;

            Microseconds getCtsMessageDurationMicroseconds() const;

            Microseconds getRtsMessageDurationMicroseconds() const;


          public:
            ModeTimingParameters(const MACConfig & macConfig);

            ~ModeTimingParameters();

            bool packetTimedOut(const Microseconds & txOpMicroseconds, const TimePoint &) const;

            int getContentionWindow(std::uint8_t, std::uint8_t) const;

            Microseconds getSlotSizeMicroseconds() const;

            Microseconds getOverheadMicroseconds(std::uint8_t u8Category) const;

            Microseconds getMessageDurationMicroseconds(std::uint8_t, size_t) const;

            Microseconds getDeferIntervalMicroseconds(std::uint8_t) const;

            TimePoint getSotTime() const;
          };
       }
   }
}

#endif //EMANEMODELSIEEE802ABGMODETIMINGPARAMTERS_HEADER_
