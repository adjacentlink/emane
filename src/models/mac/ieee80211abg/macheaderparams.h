/*
 * Copyright (c) 2013 Adjacent Link, LLC, Bridgewater, New Jersey
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

#ifndef EMANEMODELSIEEE802ABGMACHEADERPARAMS_HEADER_
#define EMANEMODELSIEEE802ABGMACHEADERPARAMS_HEADER_

#include "emane/types.h"
#include "emane/utils/netutils.h"

#include "msgtypes.h"
#include "ieee80211abgmacheadermessage.h"

#include <ace/Time_Value.h>

#include <string>

namespace EMANE
 {
  namespace Models
   {
     namespace IEEE80211ABG
       {

        /**
         *
         * @brief high fidelity mac header.
         *
         */
        class MACHeaderParams
         {
           public:
          /**
           *
           * @brief high fidelity mac header initializer.
           *
           */
          MACHeaderParams(MACHeaderMessage &);

          ~MACHeaderParams();

          /**
           *
           * @brief high fidelity mac header initializer.
           *
           * @param type     msg type data, rtc, cts, ack
           * @param retries  retry count
           * @param rate     data rate index
           * @param seq      sequence number
           * @param src      source NEM
           * @param dst      destination NEM
           * @param duration message duration
           *
           */ 
           MACHeaderParams(std::uint8_t type,
                           std::uint8_t retries,
                           std::uint16_t rate,
                           std::uint16_t seq,
                           std::uint16_t src,
                           std::uint16_t dst,
                           const Microseconds & duration);

           std::uint8_t getMessageType() const;

           std::uint8_t getNumRetries() const;

           std::uint16_t getDataRateIndex() const;
 
           std::uint16_t getSequenceNumber() const;

           std::uint16_t getSrcNEM() const;

           std::uint16_t getDstNEM() const;

           Microseconds getDurationMicroseconds() const;

         private:
           std::uint8_t  u8MsgType_;               // msg type
           std::uint8_t  u8NumRetries_;            // retry count

           std::uint16_t u16DataRateIndex_;        // data rate index
           std::uint16_t u16SequenceNumber_;       // sequence number
           std::uint16_t u16SrcNEM_;               // src NEM
           std::uint16_t u16DstNEM_;               // dst NEM

           Microseconds  durationMicroseconds_;    // duration
         };
      }
   }
}

#endif //EMANEMODELSIEEE802ABGMACHEADERPARAMS_HEADER_
