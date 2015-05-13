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

#ifndef EMANETDMARADIOMODELSCHEDULERUSER_HEADER_
#define EMANETDMARADIOMODELSCHEDULERUSER_HEADER_

#include "emane/models/tdma/types.h"
#include "emane/downstreampacket.h"
#include "emane/controlmessage.h"

namespace EMANE
{
  namespace Models
  {
    namespace TDMA
    {
      class SchedulerUser
      {
      public:
        virtual ~SchedulerUser(){};
          
        virtual void notifyScheduleChange(const Frequencies & frequencies,
                                          std::uint64_t u64BandwidthHz,
                                          const Microseconds & slotDuration,
                                          const Microseconds & slotOverhead) = 0;
        
        virtual void processSchedulerPacket(DownstreamPacket & pkt,
                                            const ControlMessages & msgs = empty) = 0;
        

        virtual void processSchedulerControl(const ControlMessages & msgs) = 0;
        
        virtual QueueInfos getPacketQueueInfo() const = 0;


        static const ControlMessages empty;
        
      protected:
        SchedulerUser(){}
      };
    }
  }
}

#endif // EMANETDMARADIOMODELSCHEDULERUSER_HEADER_
