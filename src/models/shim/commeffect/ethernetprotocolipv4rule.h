/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2009 - DRS CenGen, LLC, Columbia, Maryland
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
 *
 */

#ifndef EMANEMODELSCOMMEFFECTETHERNETPROTOCOLIPV4RULE_HEADER_
#define EMANEMODELSCOMMEFFECTETHERNETPROTOCOLIPV4RULE_HEADER_

#include "ethernetprotocolrule.h"

namespace EMANE
{
  namespace Models
  {
    namespace CommEffect
    {
      class EthernetProtocolIPv4Rule : public EthernetProtocolRule
      {
      public:
        EthernetProtocolIPv4Rule(std::uint32_t u32Src, 
                                 std::uint32_t u32Dst,
                                 std::uint16_t u16Len,
                                 std::uint8_t u8TOS,
                                 std::uint8_t u8TTL,
                                 const IPProtocolRules & rules);
        
        ~EthernetProtocolIPv4Rule();
        
        bool match(const void * buf, std::size_t len, std::uint16_t u16Type) override;
      
      private:
        bool bCare_;
        std::uint32_t u32Src_;
        std::uint32_t u32Dst_;
        std::uint16_t u16Len_;
        std::uint8_t u8TOS_;
        std::uint8_t u8TTL_;
      };
    }
  }
}

#endif // EMANEMODELSCOMMEFFECTETHERNETPROTOCOLIPV4RULE_HEADER_
