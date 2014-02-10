/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANEMODELSIEEE802ABGMACHEADERMESSAGE_HEDAER_
#define EMANEMODELSIEEE802ABGMACHEADERMESSAGE_HEDAER_


#include <cstdint>
#include <memory>

#include "emane/types.h"
#include "emane/serializable.h"

namespace EMANE
{
  namespace Models
  {
    namespace IEEE80211ABG
    {
      class MACHeaderMessage : public Serializable
      {
      public:
        MACHeaderMessage(std::uint8_t  u8MessageType, 
                         std::uint8_t  u8NumRetries, 
                         std::uint16_t u16DataRateIndex,
                         std::uint16_t u16SequenceNumber,
                         std::uint16_t u16SrcNEM,
                         std::uint16_t u16DstNEM,
                         const Microseconds & duration);

        /**
         * @throw SerializationException
         */
        MACHeaderMessage(const void * p, size_t len);

        ~MACHeaderMessage();

        std::uint8_t getMessageType() const;

        std::uint8_t getNumRetries() const;

        std::uint16_t getDataRateIndex() const;

        std::uint16_t getSequenceNumber() const;

        std::uint16_t getSrcNEM()const;

        std::uint16_t getDstNEM()const;

        Microseconds getDurationMicroseconds() const;

        Serialization serialize() const override;
     
      private:
        class Implementation;

        std::unique_ptr<Implementation> pImpl_;
      
        MACHeaderMessage(const MACHeaderMessage & r) = delete;
      
        MACHeaderMessage & 
        operator=(const MACHeaderMessage &) = delete;
      };
    }
  }
}

#endif //EMANEMODELSIEEE802ABGMACHEADERMESSAGE_HEDAER_
