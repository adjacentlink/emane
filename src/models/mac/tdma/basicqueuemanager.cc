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

#include "emane/models/tdma/basicqueuemanager.h"
#include "emane/configureexception.h"
#include "queue.h"

class EMANE::Models::TDMA::BasicQueueManager::Implementation
{
public:
  bool bAggregationEnable_{};
  bool bFragmentationEnable_{};
  Queue queues_[5];
};

EMANE::Models::TDMA::BasicQueueManager::BasicQueueManager(NEMId id,
                                                          PlatformServiceProvider * pPlatformServiceProvider):
  QueueManager{id,pPlatformServiceProvider},
  pImpl_{new Implementation{}}{}
          
EMANE::Models::TDMA::BasicQueueManager::~BasicQueueManager(){}

void EMANE::Models::TDMA::BasicQueueManager::initialize(Registrar & registrar)
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL, 
                          "MACI %03hu TDMA::BasicQueueManager::%s", 
                          id_,
                          __func__);

  auto & configRegistrar = registrar.configurationRegistrar();
  
  configRegistrar.registerNumeric<std::uint16_t>("queuemanager.queuedepth",
                                                 ConfigurationProperties::DEFAULT,
                                                 {256},
                                                 "Defines the size of the per service class downstream packet"
                                                 " queues. Each of the 5 queues (control + 4 service classes) will be"
                                                 " 'queuedepth' size.");

  configRegistrar.registerNumeric<bool>("queuemanager.aggregationenable",
                                        ConfigurationProperties::DEFAULT,
                                        {true},
                                        "Defines whether packet aggregation is enabled for transmission. When"
                                        " enabled, multiple packets can be sent in the same transmission when"
                                        " there is additional room within the slot.");


  configRegistrar.registerNumeric<bool>("queuemanager.fragmentationenable",
                                        ConfigurationProperties::DEFAULT,
                                        {true},
                                        "Defines whether packet fragmentation is enabled. When enabled, a"
                                        " single packet will be fragmented into multiple packets to be sent"
                                        " over multiple transmissions when the slot is too small. Note:"
                                        " Fragmentation will not occur within an aggregated transmission");
}
        
void EMANE::Models::TDMA::BasicQueueManager::configure(const ConfigurationUpdate & update)
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL, 
                          "MACI %03hu TDMA::BasicQueueManager::%s", 
                          id_,
                          __func__);
  
  for(const auto & item : update)
    {
      if(item.first == "queuemanager.queuedepth")
        {
          std::uint16_t u16QueueDepth {item.second[0].asUINT16()};

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "MACI %03hu TDMA::BasicQueueManager::%s: %s = %hu",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  u16QueueDepth);

          for(auto & queue :  pImpl_->queues_)
            {
              queue.initialize(u16QueueDepth);
            }
        }
      else if(item.first == "queuemanager.aggregationenable")
        {
          pImpl_->bAggregationEnable_ = item.second[0].asBool();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "MACI %03hu TDMA::BaseModel::%s: %s = %s",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  pImpl_->bAggregationEnable_ ? "on" : "off");
        }
      else if(item.first == "queuemanager.fragmentationenable")
        {
          pImpl_->bFragmentationEnable_ = item.second[0].asBool();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "MACI %03hu TDMA::BaseModel::%s: %s = %s",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  pImpl_->bFragmentationEnable_ ? "on" : "off");
        }
      else
        {
          throw makeException<ConfigureException>("TDMA::BasicQueueManager: "
                                                   "Unexpected configuration item %s",
                                                   item.first.c_str());
        }
    }
}

void EMANE::Models::TDMA::BasicQueueManager::start()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL, 
                          "MACI %03hu TDMA::BasicQueueManager::%s", 
                          id_,
                          __func__);
}

void EMANE::Models::TDMA::BasicQueueManager::postStart()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL, 
                          "MACI %03hu TDMA::BasicQueueManager::%s", 
                          id_,
                          __func__);
}

void EMANE::Models::TDMA::BasicQueueManager::stop()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL, 
                          "MACI %03hu TDMA::BasicQueueManager::%s", 
                          id_,
                          __func__);
}

void EMANE::Models::TDMA::BasicQueueManager::destroy() throw()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL, 
                          "MACI %03hu TDMA::BasicQueueManager::%s", 
                          id_,
                          __func__);
}

void  EMANE::Models::TDMA::BasicQueueManager::enqueue(std::uint8_t u8QueueIndex,
                                                     DownstreamPacket && pkt)
{
  if(u8QueueIndex < 5)
    {
      pImpl_->queues_[u8QueueIndex].enqueue(std::move(pkt));
    }
}

std::pair<EMANE::DownstreamPacket,bool>
EMANE::Models::TDMA::BasicQueueManager::dequeue(std::uint8_t u8QueueIndex,
                                               NEMId destination)
{
  if(u8QueueIndex < 5)
    {
      return pImpl_->queues_[u8QueueIndex].dequeue(destination);
    }

  return {{{0,0,0,{}},nullptr,0},false};
}
