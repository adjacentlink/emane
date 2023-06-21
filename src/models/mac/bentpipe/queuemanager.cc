/*
 * Copyright (c) 2015,2023 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "queuemanager.h"

#include "emane/configureexception.h"

EMANE::Models::BentPipe::QueueManager::QueueManager(NEMId id,
                                                    PlatformServiceProvider * pPlatformService,
                                                    PacketStatusPublisher * pPacketStatusPublisher):
  id_{id},
  pPlatformService_{pPlatformService},
  pPacketStatusPublisher_{pPacketStatusPublisher},
  u16QueueDepth_{},
  bAggregationEnable_{},
  bFragmentationEnable_{}{}

EMANE::Models::BentPipe::QueueManager::~QueueManager(){}

void EMANE::Models::BentPipe::QueueManager::initialize(Registrar & registrar)
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "MACI %03hu BentPipe::QueueManager::%s",
                          id_,
                          __func__);

  auto & configRegistrar = registrar.configurationRegistrar();

  std::string sPrefix{CONFIG_PREFIX};

  configRegistrar.registerNumeric<std::uint16_t>(sPrefix + "depth",
                                                 ConfigurationProperties::DEFAULT,
                                                 {256},
                                                 "Defines the size of the per transponder downstream packet"
                                                 " queues in packets.");

  configRegistrar.registerNumeric<bool>(sPrefix + "aggregationenable",
                                        ConfigurationProperties::DEFAULT,
                                        {true},
                                        "Defines whether packet aggregation is enabled for transmission. When"
                                        " enabled, multiple packets up to a specified MTU can be sent in the same"
                                        " transmission.");


  configRegistrar.registerNumeric<bool>(sPrefix + "fragmentationenable",
                                        ConfigurationProperties::DEFAULT,
                                        {true},
                                        "Defines whether packet fragmentation is enabled. When enabled, a single"
                                        " packet larger than a specified MTU will be fragmented into multiple message"
                                        " and sent. When disabled, packets larger than the MTU will be discarded.");

  auto & statisticRegistrar = registrar.statisticRegistrar();

  queueStatusPublisher_.registerStatistics(statisticRegistrar);

}

void EMANE::Models::BentPipe::QueueManager::configure(const ConfigurationUpdate & update)
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "MACI %03hu BentPipe::QueueManager::%s",
                          id_,
                          __func__);

  std::string sPrefix{CONFIG_PREFIX};

  for(const auto & item : update)
    {
      if(item.first == sPrefix + "depth")
        {
          u16QueueDepth_ = item.second[0].asUINT16();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "MACI %03hu BentPipe::QueueManager::%s: %s = %hu",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  u16QueueDepth_);
        }
      else if(item.first == sPrefix + "aggregationenable")
        {
          bAggregationEnable_ = item.second[0].asBool();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "MACI %03hu BentPipe::QueueManager::%s: %s = %s",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  bAggregationEnable_ ? "on" : "off");
        }
      else if(item.first == sPrefix + "fragmentationenable")
        {
          bFragmentationEnable_ = item.second[0].asBool();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "MACI %03hu BentPipe::QueueManager::%s: %s = %s",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  bFragmentationEnable_ ? "on" : "off");
        }
      else
        {
          throw makeException<ConfigureException>("BentPipe::QueueManager: "
                                                  "Unexpected configuration item %s",
                                                  item.first.c_str());
        }
    }

}

void  EMANE::Models::BentPipe::QueueManager::addQueue(TransponderIndex transponderIndex)
{
  Queue * pQueue = new Queue{};

  pQueue->initialize(u16QueueDepth_,
                     bFragmentationEnable_,
                     bAggregationEnable_);

  queues_.emplace(transponderIndex,pQueue);
}

void EMANE::Models::BentPipe::QueueManager::removeQueue(TransponderIndex transponderIndex)
{
  queues_.erase(transponderIndex);
}

void EMANE::Models::BentPipe::QueueManager::start()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "MACI %03hu BentPipe::QueueManager::%s",
                          id_,
                          __func__);
}

void EMANE::Models::BentPipe::QueueManager::postStart()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "MACI %03hu BentPipe::QueueManager::%s",
                          id_,
                          __func__);
}

void EMANE::Models::BentPipe::QueueManager::stop()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "MACI %03hu BentPipe::QueueManager::%s",
                          id_,
                          __func__);
}

void EMANE::Models::BentPipe::QueueManager::destroy() throw()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "MACI %03hu BentPipe::QueueManager::%s",
                          id_,
                          __func__);
}

size_t EMANE::Models::BentPipe::QueueManager::enqueue(TransponderIndex transponserIndex,
                                                      DownstreamPacket && pkt)
{
  size_t packetsDropped{};

  if(auto iter = queues_.find(transponserIndex);
     iter != queues_.end())
    {
      auto ret = iter->second->enqueue(std::move(pkt));

      if(ret.second)
        {
          packetsDropped = 1;

          queueStatusPublisher_.drop(transponserIndex,
                                     QueueStatusPublisher::DropReason::DROP_OVERFLOW,
                                     1);

          const auto & pktInfo = ret.first->getPacketInfo();

          pPacketStatusPublisher_->outbound(pktInfo.getSource(),
                                            pktInfo.getDestination(),
                                            ret.first->length(),
                                            PacketStatusPublisher::OutboundAction::DROP_OVERFLOW);
        }

      queueStatusPublisher_.enqueue(transponserIndex);
    }

  return packetsDropped;
}

std::tuple<EMANE::Models::BentPipe::MessageComponents,size_t>
EMANE::Models::BentPipe::QueueManager::dequeue(TransponderIndex transponserIndex,
                                               size_t requestedBytes)
{
  MessageComponents components{};
  size_t totalLength{};

  if(auto iter = queues_.find(transponserIndex);
     iter != queues_.end())
    {
      auto ret = iter->second->dequeue(requestedBytes,
                                       true);

      auto length = std::get<1>(ret);

      if(length)
        {
          totalLength += length;

          auto & parts = std::get<0>(ret);

          queueStatusPublisher_.dequeue(transponserIndex,
                                        parts);

          components.splice(components.end(),parts);
        }

      // update drop info
      for(const auto & pPkt : std::get<2>(ret))
        {
          const auto & pktInfo = pPkt->getPacketInfo();

          pPacketStatusPublisher_->outbound(pktInfo.getSource(),
                                            pktInfo.getDestination(),
                                            pPkt->length(),
                                            PacketStatusPublisher::OutboundAction::DROP_TOO_BIG);

          queueStatusPublisher_.drop(transponserIndex,
                                     QueueStatusPublisher::DropReason::DROP_TOOBIG,
                                     1);
        }
    }

  pPacketStatusPublisher_->outbound(id_,
                                    components,
                                    PacketStatusPublisher::OutboundAction::ACCEPT_GOOD);

  return std::make_tuple(std::move(components),totalLength);
}

EMANE::Models::BentPipe::QueueInfos
EMANE::Models::BentPipe::QueueManager::getPacketQueueInfo() const
{
  QueueInfos queueInfos{};

  for(const auto & entry : queues_)
    {
      auto status = entry.second->getStatus();

      queueInfos.push_back({entry.first, // transponder index
          std::get<0>(status), // packets
          std::get<1>(status)}); //bytes
    }

  return queueInfos;
}
