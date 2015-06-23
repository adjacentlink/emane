/*
 * Copyright (c) 2015 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANEMODELSTDMAPACKETSTATUSPUBLISHER_HEADER_
#define EMANEMODELSTDMAPACKETSTATUSPUBLISHER_HEADER_

#include "emane/models/tdma/messagecomponent.h"

namespace EMANE
{
  namespace Models
  {
    namespace TDMA
    {
      class PacketStatusPublisher
      {
      public:
        virtual ~PacketStatusPublisher(){}

        enum class InboundAction
        {
          ACCEPT_GOOD,
          DROP_BAD_CONTROL,
          DROP_SLOT_NOT_RX,
          DROP_SLOT_MISSED_RX,
          DROP_MISS_FRAGMENT,
          DROP_SPECTRUM_SERVICE,
          DROP_SINR,
          DROP_REGISTRATION_ID,
          DROP_DESTINATION_MAC
        };

        enum class OutboundAction
        {
          ACCEPT_GOOD,
          DROP_TOO_BIG,
          DROP_OVERFLOW,
          DROP_FLOW_CONTROL
         };

        virtual void inbound(NEMId src,
                             const MessageComponent & component,
                             InboundAction action) = 0;

        virtual void inbound(NEMId src,
                             const MessageComponents & components,
                             InboundAction action) = 0;

        virtual void inbound(NEMId src,
                             NEMId dst,
                             Priority priority,
                             size_t size,
                             InboundAction action) = 0;

        virtual void outbound(NEMId src,
                              NEMId dst,
                              Priority priority,
                              size_t size,
                              OutboundAction action) = 0;

        virtual void outbound(NEMId src,
                              const MessageComponents & components,
                              OutboundAction action) = 0;

      protected:
        PacketStatusPublisher(){}
      };
    }
  }
}

#endif // EMANEMODELSTDMAPACKETSTATUSPUBLISHER_HEADER_
