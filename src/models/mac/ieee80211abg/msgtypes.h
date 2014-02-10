/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2008-2009 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANEMODELSIEEE802ABGMSGTYPES_HEADER_
#define EMANEMODELSIEEE802ABGMSGTYPES_HEADER_


namespace EMANE
 {
  namespace Models
   {
     namespace IEEE80211ABG
       {
         const std::uint8_t MSG_TYPE_NONE                   {0x00};
         const std::uint8_t MSG_TYPE_BROADCAST_DATA         {0x01};
         const std::uint8_t MSG_TYPE_UNICAST_DATA           {0x02};
         const std::uint8_t MSG_TYPE_UNICAST_RTS_CTS_DATA   {0x04};
         const std::uint8_t MSG_TYPE_UNICAST_CTS_CTRL       {0x08};

         const std::uint8_t MSG_TYPE_MASK_BROADCAST         {0x01};
         const std::uint8_t MSG_TYPE_MASK_UNICAST           {0x02};
         const std::uint8_t MSG_TYPE_MASK_ALL_DATA          {0x07};
         const std::uint8_t MSG_TYPE_MASK_ALL               {0x0F};
       }
    }
}

#endif //EMANEMODELSIEEE802ABGMSGTYPES_HEADER_
