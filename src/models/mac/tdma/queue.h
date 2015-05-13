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

#ifndef EMANEMODELSTDMAQUEUE_HEADER_
#define EMANEMODELSTDMAQUEUE_HEADER_

#include <cstdint>
#include <map>

namespace EMANE
{
  namespace Models
  {
    namespace TDMA
    {
      class Queue
      {
      public:
        Queue():
          u16QueueDepth_{},
          u64Counter_{}{}
        
        void initialize(std::uint16_t u16QueueDepth)
        {
          u16QueueDepth_ = u16QueueDepth;
        }
        
        void enqueue(DownstreamPacket && pkt)
        {
          DownstreamPacket * pPkt{new DownstreamPacket{std::move(pkt)}};
          NEMId dest{pPkt->getPacketInfo().getDestination()};

          if(queue_.size() == u16QueueDepth_)
            {
              auto const & entry = queue_.begin();

              NEMId dest{entry->second->getPacketInfo().getDestination()};        

              delete entry->second;

              destQueue_[dest].erase(entry->first);

              queue_.erase(entry->first);
            }
            
          queue_.insert(std::make_pair(u64Counter_,pPkt));
            
          auto iter = destQueue_.find(dest);

          if(iter == destQueue_.end())
            {
              iter = destQueue_.insert({dest,PacketQueue{}}).first;
            }

          iter->second.insert(std::make_pair(u64Counter_,pPkt));

          ++u64Counter_;
        }

        std::pair<DownstreamPacket,bool> dequeue(NEMId destination)
        {
          if(destination)
            {
              auto iter = destQueue_.find(destination);
                
              if(iter != destQueue_.end())
                {
                  auto const & entry = iter->second.begin();
                    
                  auto retVal = std::make_pair(DownstreamPacket{std::move(*entry->second)},true);
                    
                  delete entry->second;
                    
                  queue_.erase(entry->first);
                    
                  destQueue_.erase(iter);
                    
                  return retVal;
                }
            }
          else if(!queue_.empty())
            {
              auto const & entry = queue_.begin();

              auto retVal = std::make_pair(DownstreamPacket{std::move(*entry->second)},true);

              delete entry->second;

              destQueue_.erase(entry->first);
                
              queue_.erase(entry);

              return retVal;
            }

          return {{{0,0,0,{}},nullptr,0},false};
        }

          
      private:
        using PacketQueue = std::map<std::uint64_t,DownstreamPacket *>;
        PacketQueue queue_;
        std::map<NEMId,PacketQueue> destQueue_;
        std::uint16_t u16QueueDepth_;
        std::uint64_t u64Counter_;
      };
    }
  }
}

#endif // EMANEMODELSTDMAQUEUE_HEADER_
