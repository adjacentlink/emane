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

#include "eventscheduler.h"
#include "emane/events/tdmascheduleevent.h"

EMANE::Models::TDMA::EventScheduler::EventScheduler(NEMId id,
                                                    PlatformServiceProvider * pPlatformServiceProvider,
                                                    SchedulerUser * pSchedulerUser):
  Scheduler{id,pPlatformServiceProvider,pSchedulerUser},
  bWaitingFirstTxSlotInfoRequest_{}
{}

EMANE::Models::TDMA::EventScheduler::~EventScheduler()
{}

void EMANE::Models::TDMA::EventScheduler::initialize(Registrar & registrar)
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "MACI %03hu TDMA::EventScheduler::%s",
                          id_,
                          __func__);

  auto & statisticRegistrar = registrar.statisticRegistrar();

  eventTablePublisher_.registerStatistics(statisticRegistrar);

  auto & eventRegistrar = registrar.eventRegistrar();

  eventRegistrar.registerEvent(Events::TDMAScheduleEvent::IDENTIFIER);
}

void EMANE::Models::TDMA::EventScheduler::configure(const ConfigurationUpdate &)
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "MACI %03hu TDMA::EventScheduler::%s",
                          id_,
                          __func__);
}

void EMANE::Models::TDMA::EventScheduler::start()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "MACI %03hu TDMA::EventScheduler::%s",
                          id_,
                          __func__);
}

void EMANE::Models::TDMA::EventScheduler::postStart()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "MACI %03hu TDMA::EventScheduler::%s",
                          id_,
                          __func__);
}

void EMANE::Models::TDMA::EventScheduler::stop()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "MACI %03hu TDMA::EventScheduler::%s",
                          id_,
                          __func__);
}

void EMANE::Models::TDMA::EventScheduler::destroy() throw()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "MACI %03hu TDMA::EventScheduler::%s",
                          id_,
                          __func__);
}

void EMANE::Models::TDMA::EventScheduler::processEvent(const EventId & eventId,
                                                       const Serialization & serialization)
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "MACI %03hu TDMA::EventScheduler::%s",
                          id_,
                          __func__);

  if(eventId == Events::TDMAScheduleEvent::IDENTIFIER)
    {
      try
        {
          Events::TDMAScheduleEvent event{serialization};
          Events::SlotStructure structure{};
          bool bHasStructure{};
          bool bNotify{};

          std::tie(structure,bHasStructure) = event.getSlotStructure();

          if(bHasStructure)
            {
              LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                      DEBUG_LEVEL,
                                      "MACI %03hu TDMA::EventScheduler::%s full"
                                      " schdule received",
                                      id_,
                                      __func__);
              // clear out existing schedule
              slotInfos_.clear();

              // store new structure info
              structure_ = structure;

              // store new schedule
              slotInfos_ = event.getSlotInfos();

              // will notify ScheduleUser of schedule
              bNotify = true;
            }
          else if(slotInfos_.empty())
            {
              // no structure, this is an update but we never received a
              // schdule to update
              LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                      ERROR_LEVEL,
                                      "MACI %03hu TDMA::EventScheduler::%s schedule"
                                      " rejected update received before full schedule",
                                      id_,
                                      __func__);
            }
          else
            {
              // process the update
              auto framesPerMultiFrame = structure_.getFramesPerMultiFrame();
              auto slotsPerFrame = structure_.getSlotsPerFrame();
              const auto & slotInfos = event.getSlotInfos();

              std::vector<int> indexes{};
              indexes.reserve(slotInfos.size());

              for(const auto & slotInfo : slotInfos)
                {
                  if(slotInfo.getFrameIndex() < framesPerMultiFrame)
                    {
                      if(slotInfo.getSlotIndex() < slotsPerFrame)
                        {
                          indexes.push_back(slotInfo.getFrameIndex() *
                                            framesPerMultiFrame +
                                            slotInfo.getSlotIndex());
                        }
                      else
                        {
                          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                                  ERROR_LEVEL,
                                                  "MACI %03hu TDMA::EventScheduler::%s schedule"
                                                  " rejected update slot index %u out of range",
                                                  id_,
                                                  __func__,
                                                  slotInfo.getSlotIndex());

#warning "Add reject stats"
                          break;
                        }
                    }
                  else
                    {
                      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                              ERROR_LEVEL,
                                              "MACI %03hu TDMA::EventScheduler::%s schedule"
                                              " rejected update frame index %u out of range",
                                              id_,
                                              __func__,
                                              slotInfo.getFrameIndex());


#warning "add reject stats"

                      break;
                    }
                }

              // only update schedule if there are no errors
              if(indexes.size() == slotInfos.size())
                {
                  auto indexIter = indexes.begin();
                  auto slotInfoIter = slotInfos.begin();

                  for(;indexIter != indexes.end(); ++indexIter, ++slotInfoIter)
                    {
                      slotInfos_[*indexIter] = *slotInfoIter;
                    }

                  // will notify ScheduleUser of schedule update
                  bNotify = true;
                }
            }

          if(bNotify)
            {
              if(bHasStructure)
                {
                  eventTablePublisher_.replace(event.getSlotInfos(),structure_);
                }
              else
                {
                  eventTablePublisher_.update(event.getSlotInfos());
                }

              bWaitingFirstTxSlotInfoRequest_ = true;

              slotter_.reset(structure_.getSlotDuration(),
                             structure_.getSlotsPerFrame(),
                             structure_.getFramesPerMultiFrame());


              pSchedulerUser_->notifyScheduleChange(event.getFrequencies(),
                                                    structure_.getBandwidth(),
                                                    structure_.getSlotDuration(),
                                                    structure_.getSlotOverhead());
            }
        }
      catch(SerializationException & exp)
        {
           LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                   ERROR_LEVEL,
                                   "MACI %03hu TDMA::EventScheduler::%s schedule"
                                   " rejected %s",
                                   id_,
                                   __func__,
                                   exp.what());
        }

    }
}

EMANE::Models::TDMA::SlotInfo
EMANE::Models::TDMA::EventScheduler::getSlotInfo(std::uint64_t u64AbsoluteSlotIndex) const
{
  std::uint32_t u32RelativeSlotIndex{};
  std::uint32_t u32RelativeFrameIndex{};

  std::tie(u32RelativeSlotIndex,
           u32RelativeFrameIndex) =
    slotter_.getRelativeIndex(u64AbsoluteSlotIndex);

  auto index = u32RelativeFrameIndex * structure_.getSlotsPerFrame() + u32RelativeSlotIndex;

  const auto & info = slotInfos_[index];

  return {u64AbsoluteSlotIndex,
      index,
      u32RelativeSlotIndex,
      u32RelativeFrameIndex,
      slotter_.getSlotTime(u64AbsoluteSlotIndex),
      info.getType() == Events::SlotInfo::Type::RX ? SlotInfo::Type::RX :
      info.getType() == Events::SlotInfo::Type::TX ?
      SlotInfo::Type::TX : SlotInfo::Type::IDLE};
  }

std::pair<EMANE::Models::TDMA::RxSlotInfo,bool>
EMANE::Models::TDMA::EventScheduler::getRxSlotInfo(const TimePoint & timePoint) const
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "MACI %03hu TDMA::EventScheduler::%s",
                          id_,
                          __func__);

  std::uint64_t u64AbsoluteSlotIndex{};
  std::uint64_t u64AbsoluteFrameIndex{};
  std::uint64_t u64AbsoluteMultiFrameIndex{};

  std::tie(u64AbsoluteSlotIndex,
           u64AbsoluteFrameIndex,
           u64AbsoluteMultiFrameIndex) = slotter_.getAbsoluteIndex(timePoint);

  std::uint32_t u32RelativeSlotIndex{};
  std::uint32_t u32RelativeFrameIndex{};

  std::tie(u32RelativeSlotIndex,
           u32RelativeFrameIndex) =
    slotter_.getRelativeIndex(u64AbsoluteSlotIndex);

  auto index = u32RelativeFrameIndex * structure_.getSlotsPerFrame() + u32RelativeSlotIndex;


  const auto & info = slotInfos_[index];

  RxSlotInfo rxSlotInfo{u64AbsoluteSlotIndex,
      index,
      u32RelativeSlotIndex,
      u32RelativeFrameIndex,
      slotter_.getSlotTime(u64AbsoluteSlotIndex),
      info.getFrequency()};

  return {rxSlotInfo,info.getType() == Events::SlotInfo::Type::RX};
}

std::pair<EMANE::Models::TDMA::TxSlotInfos,EMANE::TimePoint>
EMANE::Models::TDMA::EventScheduler::getTxSlotInfo(const TimePoint & timePoint,
                                                   int multiframes) const
{
  TimePoint requestTime{timePoint};

  if(bWaitingFirstTxSlotInfoRequest_)
    {
      // first request since new schedule available
      auto indexes = slotter_.getAbsoluteIndex(timePoint);

      requestTime = slotter_.getMultiFrameTime(std::get<2>(indexes) + 1);

      bWaitingFirstTxSlotInfoRequest_ = false;
    }

  std::uint64_t u64AbsoluteSlotIndex{};
  std::uint64_t u64AbsoluteFrameIndex{};
  std::uint64_t u64AbsoluteMultiFrameIndex{};

  std::tie(u64AbsoluteSlotIndex,
           u64AbsoluteFrameIndex,
           u64AbsoluteMultiFrameIndex) = slotter_.getAbsoluteIndex(requestTime);

  std::uint32_t u32RelativeSlotIndex{};
  std::uint32_t u32RelativeFrameIndex{};

  std::tie(u32RelativeSlotIndex,
           u32RelativeFrameIndex) =
    slotter_.getRelativeIndex(u64AbsoluteSlotIndex);

  auto index = u32RelativeFrameIndex * structure_.getSlotsPerFrame() + u32RelativeSlotIndex;

  TxSlotInfos txSlotInfos{};

  for(int i = 0; i < multiframes; ++i)
    {
      while(index < slotInfos_.size())
        {
          const auto & info = slotInfos_[index];

          if(info.getType() == Events::SlotInfo::Type::TX)
            {
              txSlotInfos.push_back({u64AbsoluteSlotIndex,
                    index,
                    info.getSlotIndex(),
                    info.getFrameIndex(),
                    slotter_.getSlotTime(u64AbsoluteSlotIndex),
                    info.getFrequency(),
                    info.getDataRate(),
                    info.getServiceClass(),
                    info.getPower(),
                    info.getDestination()});
            }

          ++u64AbsoluteSlotIndex;
          ++index;
        }

      index = 0;
    }

  return {txSlotInfos,slotter_.getMultiFrameTime(u64AbsoluteMultiFrameIndex + multiframes)};
}

void EMANE::Models::TDMA::EventScheduler::processSchedulerPacket(UpstreamPacket &,
                                                                 const PacketMetaInfo &)
{
 LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "MACI %03hu TDMA::EventScheduler::%s",
                          id_,
                          __func__);
}

void EMANE::Models::TDMA::EventScheduler::processPacketMetaInfo(const PacketMetaInfo &)
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "MACI %03hu TDMA::EventScheduler::%s",
                          id_,
                          __func__);
}
