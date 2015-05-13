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

#ifndef EMANEMODELSTDMABASICQUEUEMANAGER_HEADER_
#define EMANEMODELSTDMABASICQUEUEMANAGER_HEADER_

#include "emane/models/tdma/queuemanager.h"

namespace EMANE
{
  namespace Models
  {
    namespace TDMA
    {
      class BasicQueueManager : public QueueManager
      {
      public:
        BasicQueueManager(NEMId id,
                         PlatformServiceProvider * pPlatformServiceProvider);
          
        ~BasicQueueManager();
        
        void initialize(Registrar & registrar) override;
        
        void configure(const ConfigurationUpdate & update) override;
        
        void start() override;
        
        void postStart() override;
        
        void stop() override;
        
        void destroy() throw() override;
        
        void enqueue(std::uint8_t u8QueueIndex, DownstreamPacket && pkt) override;
        
        std::pair<DownstreamPacket,bool> dequeue(std::uint8_t u8QueueIndex,
                                                 NEMId destination) override;
      private:
        class Implementation;
        std::unique_ptr<Implementation> pImpl_;
      };
    }
  }
}


#endif // EMANEMODELSTDMABASICQUEUEMANAGER_HEADER_
