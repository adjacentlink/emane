/*
 * Copyright (c) 2023 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "transpondertdmaprotocol.h"
#include "transponderuser.h"

EMANE::Models::BentPipe::TransponderTDMAProtocol::TransponderTDMAProtocol(NEMId id,
                                                                          PlatformServiceProvider * pPlatformService,
                                                                          TransponderUser * pTransponderUser,
                                                                          const TransponderConfiguration & transponderConfiguration,
                                                                          SlotStatusPublisher * pSlotStatusPublisher):
  Transponder{id,pPlatformService,pTransponderUser,transponderConfiguration},
  pSlotStatusPublisher_{pSlotStatusPublisher},
  transmitTimedEventId_{},
  u64AbsoluteFrameIndexLastTxSchedule_{}
{
  slotter_.reset(configuration_.getTransmitSlotSize(),
                 configuration_.getTransmitSlotsPerFrame());
}

void EMANE::Models::BentPipe::TransponderTDMAProtocol::start()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          INFO_LEVEL,
                          "MACI %03hu BendPipe::TransponderTDMAProtocol::%s transponder:%hu",
                          id_,
                          __func__,
                          configuration_.getTransponderIndex());

  auto now = Clock::now();

  u64AbsoluteFrameIndexLastTxSchedule_ =
    std::get<1>(slotter_.getAbsoluteIndex(now)) + 100 - 1;

  getTxOpportunities(now,100);

  if(!pendingTxOpportunities_.empty())
    {
      transmitTimedEventId_ =
        pPlatformService_->timerService().
        schedule(std::bind(&TransponderTDMAProtocol::processTxOpportunity,
                           this),
                 slotter_.getSlotTime(*pendingTxOpportunities_.begin()));
    }
}

void EMANE::Models::BentPipe::TransponderTDMAProtocol::stop()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          INFO_LEVEL,
                          "MACI %03hu BendPipe::TransponderTDMAProtocol::%s transponder:%hu",
                          id_,
                          __func__,
                          configuration_.getTransponderIndex());

  if(transmitTimedEventId_)
    {
      pPlatformService_->timerService().cancelTimedEvent(transmitTimedEventId_);
      transmitTimedEventId_ = 0;
    }
}

size_t EMANE::Models::BentPipe::TransponderTDMAProtocol::getMTUBytes() const
{
  // bytes/second * SlotSize_seconds
  return (configuration_.getTransmitDataRatebps() / 8.0 ) * (configuration_.getTransmitSlotSize().count()/1000000.0);
}

EMANE::Models::BentPipe::Transponder::TransmissionInfo
EMANE::Models::BentPipe::TransponderTDMAProtocol::prepareTransmission(const TimePoint & now,
                                                                      size_t bytes,
                                                                      MessageComponents && components)
{
  std::uint64_t u64AbsoluteSlotIndex{};
  std::uint64_t u64AbsoluteFrameIndex{};

  std::tie(u64AbsoluteSlotIndex,
           u64AbsoluteFrameIndex) = slotter_.getAbsoluteIndex(now);

  Microseconds durationMicroseconds{std::chrono::duration_cast<Microseconds>(DoubleSeconds{(bytes * 8.0) / configuration_.getTransmitDataRatebps()})};

  // timestamp is start of current slot
  return {BentPipeMessage{slotter_.getSlotTime(u64AbsoluteSlotIndex),
                          configuration_.getPCRCurveIndex(),
                          std::move(components)},
          durationMicroseconds};
}


void EMANE::Models::BentPipe::TransponderTDMAProtocol::getTxOpportunities(const TimePoint & requestFrameTime,
                                                                          unsigned uNumFrames)
{
  std::uint64_t u64AbsoluteSlotIndex{};
  std::uint64_t u64AbsoluteFrameIndex{};

  std::tie(u64AbsoluteSlotIndex,
           u64AbsoluteFrameIndex) = slotter_.getAbsoluteIndex(requestFrameTime);

  std::uint64_t relativeSlot{std::get<0>(slotter_.getRelativeIndex(u64AbsoluteSlotIndex))};

  auto & txSlots = configuration_.getTransmitSlots();

  for(unsigned frame = 0; frame < uNumFrames; ++frame)
    {
      while(relativeSlot < configuration_.getTransmitSlotsPerFrame())
        {
          if(txSlots.count(relativeSlot))
            {
              pendingTxOpportunities_.insert(u64AbsoluteSlotIndex);
            }

          ++u64AbsoluteSlotIndex;
          ++relativeSlot;
        }
      relativeSlot = 0;
    }
}

void EMANE::Models::BentPipe::TransponderTDMAProtocol::processTxOpportunity()
{
  auto now = Clock::now();

  if(isTransmitOpportunity_i(now))
    {
      pTransponderUser_->notifyTxOpportunity(this);
    }

  if(!pendingTxOpportunities_.empty())
    {
      transmitTimedEventId_ =
        pPlatformService_->timerService().
        schedule(std::bind(&TransponderTDMAProtocol::processTxOpportunity,
                           this),
                 slotter_.getSlotTime(*pendingTxOpportunities_.begin()));
    }
}

bool EMANE::Models::BentPipe::TransponderTDMAProtocol::isTransmitOpportunity(const TimePoint & now)
{
  // isTransmitOpportunity: called by the radio model to test for tx op
  if(isTransmitOpportunity_i(now))
    {
      std::uint64_t u64AbsoluteSlotIndex{};
      std::uint64_t u64AbsoluteFrameIndex{};

      // set the current (now) abs slot and frame index
      std::tie(u64AbsoluteSlotIndex,
               u64AbsoluteFrameIndex) = slotter_.getAbsoluteIndex(now);

      pSlotStatusPublisher_->update(getIndex(),
                                    SlotStatusPublisher::Status::TX_GOOD,
                                    slotter_.slotPortionRatio(now,
                                                              *pendingTxOpportunities_.begin()));

      pendingTxOpportunities_.erase(pendingTxOpportunities_.begin());

      if(pendingTxOpportunities_.empty())
        {
          if(u64AbsoluteFrameIndexLastTxSchedule_ == u64AbsoluteFrameIndex)
            {
              getTxOpportunities(slotter_.getFrameTime(u64AbsoluteFrameIndex+1),
                                 100);
              u64AbsoluteFrameIndexLastTxSchedule_ = u64AbsoluteFrameIndex + 100;
            }
          else
            {
              getTxOpportunities(now,100);
              u64AbsoluteFrameIndexLastTxSchedule_ = u64AbsoluteFrameIndex + 100 - 1;
            }
        }

      return true;
    }

  return false;
}

bool EMANE::Models::BentPipe::TransponderTDMAProtocol::isTransmitOpportunity_i(const TimePoint & now)
{
  std::uint64_t u64AbsoluteSlotIndex{};
  std::uint64_t u64AbsoluteFrameIndex{};

  // set the current (now) abs slot and frame index
  std::tie(u64AbsoluteSlotIndex,
           u64AbsoluteFrameIndex) = slotter_.getAbsoluteIndex(now);

  // if the current abs slot index is greater than earliest pending tx
  // op, we need to do some cleanup -- there are expired tx opps in
  // the pending list, the pending list should NEVER be empty
  while(!pendingTxOpportunities_.empty() &&
        u64AbsoluteSlotIndex > *pendingTxOpportunities_.begin())
    {
      // find the next tx slot in the future
      auto nextTxOpportunityIter =
        pendingTxOpportunities_.upper_bound(u64AbsoluteSlotIndex);

      // there are none, left in the pending op list
      if(nextTxOpportunityIter == pendingTxOpportunities_.end())
        {
          // clear anything pending, it is all in the past
          pendingTxOpportunities_.clear();

          // if the current abs frame was the last one scheduled, get the next one
          if(u64AbsoluteFrameIndexLastTxSchedule_ == u64AbsoluteFrameIndex)
            {
              getTxOpportunities(slotter_.getFrameTime(u64AbsoluteFrameIndex+1),
                                 100);

              u64AbsoluteFrameIndexLastTxSchedule_ = u64AbsoluteFrameIndex + 100;
            }
          else
            {
              // just use now to get the next set of tx opp, no
              // telling how far into the future we have come
              getTxOpportunities(now,100);

              u64AbsoluteFrameIndexLastTxSchedule_ = u64AbsoluteFrameIndex + 100 - 1;
            }
        }
      else
        {
          for(auto iter = pendingTxOpportunities_.begin();
              iter != nextTxOpportunityIter; ++iter)
            {
              pSlotStatusPublisher_->update(getIndex(),
                                            SlotStatusPublisher::Status::TX_MISSED,
                                            slotter_.slotPortionRatio(now,*iter));
            }

          // erase all the expired tx opp
          pendingTxOpportunities_.erase(pendingTxOpportunities_.begin(),
                                        nextTxOpportunityIter);
        }
    }

  // is this a tx op
  return pendingTxOpportunities_.count(u64AbsoluteSlotIndex);
}
