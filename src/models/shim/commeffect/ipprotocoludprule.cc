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


#include "ipprotocoludprule.h"
#include "emane/net.h"
#include "emane/utils/netutils.h"


EMANE::Models::CommEffect::IPProtocolUDPRule::IPProtocolUDPRule(std::uint16_t u16SrcPort,
                                                                std::uint16_t u16DstPort) : 
  IPProtocolRule(Utils::IP_PROTO_UDP),
  bCare_{false},
  u16SrcPort_{u16SrcPort},
  u16DstPort_{u16DstPort}
{
  if((u16SrcPort_ == 0) && (u16DstPort_ == 0))
    {
      bCare_ = false;
    }
  else
    {
      bCare_ = true;
    }
}

EMANE::Models::CommEffect::IPProtocolUDPRule::~IPProtocolUDPRule(){}

bool
EMANE::Models::CommEffect::IPProtocolUDPRule::match(const void * buf,
                                                    std::size_t len,
                                                    std::uint16_t u16Type)
{ 
  // check type
  if(u16Type != u8Type_)
    {
      return 0;
    }

  // check our attributes
  if(bCare_ == false)
    {
      return true;
    }

  // check buf len
  if(len < Utils::UDP_HEADER_LEN)
    {
      return false;
    }

  const auto pHdr =
    static_cast<const Utils::UdpHeader *>(buf);

  if(u16SrcPort_ && (pHdr->u16Udpsrc != u16SrcPort_))
    {
      return false;
    }

  if(u16DstPort_ && (pHdr->u16Udpdst != u16DstPort_))
    {
      return false;
    }

  // match
  return true;
}
