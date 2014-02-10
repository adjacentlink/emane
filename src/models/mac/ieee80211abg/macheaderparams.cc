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

#include "macheaderparams.h"
#include "utils.h"

EMANE::Models::IEEE80211ABG::MACHeaderParams::MACHeaderParams(MACHeaderMessage & msg) :
   u8MsgType_{msg.getMessageType()},
   u8NumRetries_{msg.getNumRetries()},
   u16DataRateIndex_{msg.getDataRateIndex()}, 
   u16SequenceNumber_{msg.getSequenceNumber()},
   u16SrcNEM_{msg.getSrcNEM()},
   u16DstNEM_{msg.getDstNEM()},
   durationMicroseconds_{msg.getDurationMicroseconds()}
 { }


EMANE::Models::IEEE80211ABG::MACHeaderParams::MACHeaderParams(std::uint8_t type,
                                                              std::uint8_t retries,
                                                              std::uint16_t rate,
                                                              std::uint16_t seq,
                                                              std::uint16_t src,
                                                              std::uint16_t dst,
                                                              const Microseconds & duration) :
 u8MsgType_{type},
 u8NumRetries_{retries},
 u16DataRateIndex_{rate},
 u16SequenceNumber_{seq},
 u16SrcNEM_{src},
 u16DstNEM_{dst}, 
 durationMicroseconds_{duration}
{ } 

EMANE::Models::IEEE80211ABG::MACHeaderParams::~MACHeaderParams()
{ }


std::uint8_t EMANE::Models::IEEE80211ABG::MACHeaderParams::getMessageType() const
{
  return u8MsgType_;
}

std::uint8_t EMANE::Models::IEEE80211ABG::MACHeaderParams::getNumRetries() const
{
  return u8NumRetries_;
}

std::uint16_t EMANE::Models::IEEE80211ABG::MACHeaderParams::getDataRateIndex() const
{
  return u16DataRateIndex_;
}

std::uint16_t EMANE::Models::IEEE80211ABG::MACHeaderParams::getSequenceNumber() const
{
  return u16SequenceNumber_;
}

std::uint16_t EMANE::Models::IEEE80211ABG::MACHeaderParams::getSrcNEM() const
{
  return u16SrcNEM_;
}

std::uint16_t EMANE::Models::IEEE80211ABG::MACHeaderParams::getDstNEM() const
{
  return u16DstNEM_;
}


EMANE::Microseconds EMANE::Models::IEEE80211ABG::MACHeaderParams::getDurationMicroseconds() const
{
  return durationMicroseconds_;
}



