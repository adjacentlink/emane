/*
 * Copyright (c) 2013-2015 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "ethernetprotocolipv4rule.h"
#include "emane/net.h"
#include "emane/utils/netutils.h"

#include <functional>


EMANE::Models::CommEffect::EthernetProtocolIPv4Rule::EthernetProtocolIPv4Rule(std::uint32_t u32Src,
                                                                              std::uint32_t u32Dst,
                                                                              std::uint16_t u16Len,
                                                                              std::uint8_t u8TOS,
                                                                              std::uint8_t u8TTL, 
                                                                              const IPProtocolRules & rules) : 
EthernetProtocolRule{EMANE::HTONS(Utils::ETH_P_IPV4), rules},
  bCare_{false},
  u32Src_{u32Src},
  u32Dst_{u32Dst},
  u16Len_{u16Len},
  u8TOS_{u8TOS},
  u8TTL_{u8TTL} 
{ 
  if((u32Src == 0) && (u32Dst == 0) && (u16Len == 0) && (u8TOS == 0) && (u8TTL == 0) && (rules.empty()))
    {
      bCare_ = false;
    }
  else
    {
      bCare_ = true;
    }
}


EMANE::Models::CommEffect::EthernetProtocolIPv4Rule::~EthernetProtocolIPv4Rule() 
{}


bool EMANE::Models::CommEffect::EthernetProtocolIPv4Rule::match(const void * buf,
                                                                std::size_t len,
                                                                std::uint16_t u16Type) 
{ 
  // check type
  if(u16Type != u16Type_)
    {
      return false;
    }

  // check our attributes
  if(bCare_ == false)
    {
      return true;
    }

  // check buf len
  if(len < Utils::IPV4_HEADER_LEN)
    {
      return false;
    }

  const Utils::Ip4Header * pHdr = (Utils::Ip4Header *) buf;

  if(u32Src_ && (pHdr->u32Ipv4src != u32Src_))
    {
      return false;
    }
  if(u32Dst_ && (pHdr->u32Ipv4dst != u32Dst_))
    {
      return false;
    }
  if(u16Len_ && (pHdr->u16Ipv4len != u16Len_))
    {
      return false;
    }
  if(u8TOS_ && (pHdr->u8Ipv4tos != u8TOS_))
    {
      return false;
    }
  if(u8TTL_ && (pHdr->u8Ipv4hops != u8TTL_))
    {
      return false;
    }

  if(rules_.empty())
    {
      // no rules, don't care
      return 1;
    }

  const int offset = ((pHdr->u8Ipv4vhl & 0x0F) << 2);

  return std::find_if(rules_.begin(),
                      rules_.end(),
                      std::bind(&Rule::match,
                                std::placeholders::_1,
                                static_cast<const std::uint8_t *>(buf) + offset,
                                len - offset, pHdr->u8Ipv4proto)) != rules_.end();
}
