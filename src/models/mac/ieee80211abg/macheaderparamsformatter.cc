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

#include "macheaderparamsformatter.h"
#include "msgtypes.h"

namespace {
  std::string typeToString(std::uint8_t type)
   {
     switch(type)
      {
        case EMANE::Models::IEEE80211ABG::MSG_TYPE_BROADCAST_DATA:
          return "BROADCAST_DATA";

        case EMANE::Models::IEEE80211ABG::MSG_TYPE_UNICAST_DATA:
          return "UNICAST_DATA";

        case EMANE::Models::IEEE80211ABG::MSG_TYPE_UNICAST_RTS_CTS_DATA:
          return "UNICAST_RTS_CTS_DATA";

        case EMANE::Models::IEEE80211ABG::MSG_TYPE_UNICAST_CTS_CTRL:
          return "UNICAST_CTS_CTRL";

        case EMANE::Models::IEEE80211ABG::MSG_TYPE_NONE:
          return "NONE";
      }

      return "UNKNOWN";
   }
}


EMANE::Models::IEEE80211ABG::MACHeaderParamsFormatter::
MACHeaderParamsFormatter(const MACHeaderParams * pMsg):
  pMsg_{pMsg}
{ }


EMANE::Strings EMANE::Models::IEEE80211ABG::MACHeaderParamsFormatter::operator()() const
{
  Strings sFormat{"macheaderparams:",
       "type",
       typeToString(pMsg_->getMessageType()) + ",",
       "src",
       std::to_string(pMsg_->getSrcNEM()) + ",",
       "dst",
       std::to_string(pMsg_->getDstNEM()) + ",",
       "duration",
       std::to_string(std::chrono::duration_cast<DoubleSeconds>(pMsg_->getDurationMicroseconds()).count()) + ",",
       "retires",
       std::to_string(pMsg_->getNumRetries()) + ",",
       "data rate index",
       std::to_string(pMsg_->getDataRateIndex()) + ",",
       "seqnum",
       std::to_string(pMsg_->getSequenceNumber())};
       
  return sFormat;
}  
