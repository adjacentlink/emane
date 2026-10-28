/*
 * Copyright (c) 2015-2016,2025 - Adjacent Link LLC, Bridgewater,
 *  New Jersey
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

#include "emane/events/tdmascheduleevent.h"
#include "tdmascheduleevent.pb.h"

class EMANE::Events::TDMAScheduleEvent::Implementation
{
public:
  Implementation(const Serialization & serialization)
  {
    EMANEMessage::TDMAScheduleEvent msg{};

    if(!msg.ParseFromString(serialization))
      {
        throw SerializationException("unable to deserialize : TDMAScheduleEvent");
      }

    std::uint32_t u32FramesPerMultiFrame{};
    std::uint32_t u32SlotsPerFrame{};

    if(msg.has_structure())
      {
        const auto & structure = msg.structure();

        u32FramesPerMultiFrame = structure.framespermultiframe();
        u32SlotsPerFrame = structure.slotsperframe();

        slotStructure_ =
          TDMASchedule::SlotStructure{structure.bandwidthhz(),
                                      u32FramesPerMultiFrame,
                                      u32SlotsPerFrame,
                                      Microseconds{structure.slotdurationmicroseconds()},
                                      Microseconds{structure.slotoverheadmicroseconds()}};

        slotInfos_.reserve(u32FramesPerMultiFrame * u32SlotsPerFrame);
      }

    if(slotStructure_)
      {
        // pre-built a complete schedule defaulting all slots to idle
        for(unsigned i =  0; i < u32FramesPerMultiFrame; ++i)
          {
            for(unsigned j =  0; j < u32SlotsPerFrame; ++j)
              {
                slotInfos_.push_back({TDMASchedule::SlotInfo::Type::IDLE,i,j});
              }
          }
      }

    for(const auto & frame :msg.frames())
      {
        std::uint32_t u32FrameIndex = frame.index();

        if(slotStructure_ && u32FrameIndex >= u32FramesPerMultiFrame)
          {
            throw makeException<SerializationException>("TDMAScheduleEvent:"
                                                        " Frame %lu index out"
                                                        " of range",
                                                        u32FrameIndex);
          }

        std::set<std::uint32_t> presentSlots{};

        for(const auto & slot : frame.slots())
          {
            TDMASchedule::SlotInfo::Type type{TDMASchedule::SlotInfo::Type::IDLE};
            std::uint64_t u64FrequencyHz{};
            std::uint64_t u64DataRatebps{};
            std::uint8_t u8ServiceClass{};
            double dPowerdBm{};
            NEMId destination{};

            std::uint32_t u32SlotIndex = slot.index();

            if(slotStructure_ && u32SlotIndex >= u32SlotsPerFrame)
              {
                throw makeException<SerializationException>("TDMAScheduleEvent:"
                                                            " Frame %lu Slot %lu"
                                                            " slot index out of"
                                                            " range",
                                                            u32FrameIndex,
                                                            u32SlotIndex);
              }

            presentSlots.insert(u32SlotIndex);

            switch(slot.type())
              {
              case EMANEMessage::TDMAScheduleEvent::Frame::Slot::SLOT_TX:
                {
                  const auto & tx = slot.tx();

                  type = TDMASchedule::SlotInfo::Type::TX;

                  if(tx.has_frequencyhz())
                    {
                      u64FrequencyHz = tx.frequencyhz();
                    }
                  else if(frame.has_frequencyhz())
                    {
                      u64FrequencyHz = frame.frequencyhz();
                    }
                  else if(msg.has_frequencyhz())
                    {
                      u64FrequencyHz = msg.frequencyhz();
                    }
                  else
                    {
                      throw makeException<SerializationException>("TDMAScheduleEvent:"
                                                                  " Frame %lu Slot"
                                                                  " %lu has undeterminable"
                                                                  " frequency",
                                                                  u32FrameIndex,
                                                                  u32SlotIndex);
                    }

                  if(tx.has_dataratebps())
                    {
                      u64DataRatebps = tx.dataratebps();
                    }
                  else if(frame.has_dataratebps())
                    {
                      u64DataRatebps = frame.dataratebps();
                    }
                  else if(msg.has_dataratebps())
                    {
                      u64DataRatebps = msg.dataratebps();
                    }
                  else
                    {
                      throw makeException<SerializationException>("TDMAScheduleEvent:"
                                                                  " Frame %lu Slot"
                                                                  " %lu has undeterminable"
                                                                  " datarate",
                                                                  u32FrameIndex,
                                                                  u32SlotIndex);
                    }


                  if(tx.has_serviceclass())
                    {
                      u8ServiceClass = tx.serviceclass();
                    }
                  else if(frame.has_serviceclass())
                    {
                      u8ServiceClass = frame.serviceclass();
                    }
                  else if(msg.has_serviceclass())
                    {
                      u8ServiceClass = msg.serviceclass();
                    }
                  else
                    {
                      throw makeException<SerializationException>("TDMAScheduleEvent:"
                                                                  " Frame %lu Slot"
                                                                  " %lu has undeterminable"
                                                                  " class",
                                                                  u32FrameIndex,
                                                                  u32SlotIndex);
                    }


                  if(tx.has_powerdbm())
                    {
                      dPowerdBm = tx.powerdbm();
                    }
                  else if(frame.has_powerdbm())
                    {
                      dPowerdBm = frame.powerdbm();
                    }
                  else if(msg.has_powerdbm())
                    {
                      dPowerdBm = msg.powerdbm();
                    }
                  else
                    {
                      throw makeException<SerializationException>("TDMAScheduleEvent:"
                                                                  " Frame %lu Slot"
                                                                  " %lu has undeterminable"
                                                                  " power",
                                                                  u32FrameIndex,
                                                                  u32SlotIndex);
                    }


                  if(tx.has_destination())
                    {
                      destination = tx.destination();
                    }
                }
                break;

              case EMANEMessage::TDMAScheduleEvent::Frame::Slot::SLOT_RX:
                {
                  const auto & rx = slot.rx();

                  type = TDMASchedule::SlotInfo::Type::RX;

                  if(rx.has_frequencyhz())
                    {
                      u64FrequencyHz = rx.frequencyhz();
                    }
                  else if(frame.has_frequencyhz())
                    {
                      u64FrequencyHz = frame.frequencyhz();
                    }
                  else if(msg.has_frequencyhz())
                    {
                      u64FrequencyHz = msg.frequencyhz();
                    }
                  else
                    {
                      throw makeException<SerializationException>("TDMAScheduleEvent:"
                                                                  " Frame %lu Slot"
                                                                  " %lu has undeterminable"
                                                                  " frequency",
                                                                  u32FrameIndex,
                                                                  u32SlotIndex);
                    }
                }
                break;

              case EMANEMessage::TDMAScheduleEvent::Frame::Slot::SLOT_IDLE:
                type = TDMASchedule::SlotInfo::Type::IDLE;
                break;
              }


            frequencies_.insert(u64FrequencyHz);

            if(slotStructure_)
              {
                slotInfos_[u32FrameIndex * u32SlotsPerFrame + u32SlotIndex] =
                  {type,
                   u32FrameIndex,
                   u32SlotIndex,
                   u64FrequencyHz,
                   u64DataRatebps,
                   u8ServiceClass,
                   dPowerdBm,
                   destination};
              }
            else
              {
                slotInfos_.push_back({type,
                    u32FrameIndex,
                    u32SlotIndex,
                    u64FrequencyHz,
                    u64DataRatebps,
                    u8ServiceClass,
                    dPowerdBm,
                    destination});
              }
          }

        if(slotStructure_)
          {
            // add RX slot info for any missing slot
            for(unsigned i = 0; i < u32SlotsPerFrame; ++i)
              {
                if(!presentSlots.count(i))
                  {
                    std::uint64_t u64FrequencyHz{};

                    if(frame.has_frequencyhz())
                      {
                        u64FrequencyHz = frame.frequencyhz();
                      }
                    else if(msg.has_frequencyhz())
                      {
                        u64FrequencyHz = msg.frequencyhz();
                      }
                    else
                      {
                        throw makeException<SerializationException>("TDMAScheduleEvent:"
                                                                    " Frame %lu Slot"
                                                                    " %lu has undeterminable"
                                                                    " frequency",
                                                                    u32FrameIndex,
                                                                    i);
                      }

                    frequencies_.insert(u64FrequencyHz);

                    slotInfos_[u32FrameIndex * u32SlotsPerFrame + i] =
                      {TDMASchedule::SlotInfo::Type::RX,
                       u32FrameIndex,
                       i,
                       u64FrequencyHz};
                  }
              }
          }
      }
  }

  Implementation(const std::optional<TDMASchedule::SlotStructure> & slotStructure,
                 const TDMASchedule::MultiFrameParameters & multiFrameParameters,
                 const TDMASchedule::FrameParameterMap & frameParametertMap,
                 const TDMASchedule::FrameSlotMap & frameSlotMap):
    slotStructure_{slotStructure},
    multiFrameParameters_{multiFrameParameters},
    frameParametertMap_{frameParametertMap},
    frameSlotMap_{frameSlotMap}
  {}

  Serialization serialize() const
  {
    Serialization serialization{};

    EMANEMessage::TDMAScheduleEvent msg{};

    if(slotStructure_)
      {
        auto * pStructure = msg.mutable_structure();

        pStructure->set_slotsperframe(slotStructure_->getSlotsPerFrame());
        pStructure->set_framespermultiframe(slotStructure_->getFramesPerMultiFrame());
        pStructure->set_slotdurationmicroseconds(slotStructure_->getSlotDuration().count());
        pStructure->set_slotoverheadmicroseconds(slotStructure_->getSlotOverhead().count());
        pStructure->set_bandwidthhz(slotStructure_->getBandwidth());
      }

    if(multiFrameParameters_.u64FrequencyHz_)
      {
        msg.set_frequencyhz(*multiFrameParameters_.u64FrequencyHz_);
      }

    if(multiFrameParameters_.dPowerdBm_)
      {
        msg.set_powerdbm(*multiFrameParameters_.dPowerdBm_);
      }

    if(multiFrameParameters_.u8ServiceClass_)
      {
        msg.set_serviceclass(*multiFrameParameters_.u8ServiceClass_);
      }

    if(multiFrameParameters_.u64Dataratebps_)
      {
        msg.set_dataratebps(*multiFrameParameters_.u64Dataratebps_);
      }

    for(const auto & entry : frameSlotMap_)
      {
        auto frameIndex{entry.first};

        const auto & slotParamaterMap{entry.second};

        auto pFrame = msg.add_frames();

        pFrame->set_index(frameIndex);

        if(auto iter = frameParametertMap_.find(frameIndex);
           iter != frameParametertMap_.end())
          {
            const auto & frameParameters{iter->second};

            if(frameParameters.u64FrequencyHz_)
              {
                pFrame->set_frequencyhz(*frameParameters.u64FrequencyHz_);
              }

            if(frameParameters.dPowerdBm_)
              {
                pFrame->set_powerdbm(*frameParameters.dPowerdBm_);
              }

            if(frameParameters.u8ServiceClass_)
              {
                pFrame->set_serviceclass(*frameParameters.u8ServiceClass_);
              }

            if(frameParameters.u64Dataratebps_)
              {
                pFrame->set_dataratebps(*frameParameters.u64Dataratebps_);
              }
          }


        for(const auto & entry : slotParamaterMap)
          {
            auto slotIndex{entry.first};

            const auto & slotParameters{entry.second};

            auto pSlot = pFrame->add_slots();

            pSlot->set_index(slotIndex);

            if(std::holds_alternative<Events::TDMASchedule::TxSlotParameters>
               (slotParameters.parameters_))
              {
                auto pTx = pSlot->mutable_tx();

                pSlot->set_type(EMANEMessage::TDMAScheduleEvent::Frame::Slot::SLOT_TX);

                const auto &
                  txSlotParameters{std::get<Events::TDMASchedule::TxSlotParameters>
                                   (slotParameters.parameters_)};

                if(txSlotParameters.u64FrequencyHz_)
                  {
                    pTx->set_frequencyhz(*txSlotParameters.u64FrequencyHz_);
                  }

                if(txSlotParameters.dPowerdBm_)
                  {
                    pTx->set_powerdbm(*txSlotParameters.dPowerdBm_);
                  }

                if(txSlotParameters.u8ServiceClass_)
                  {
                    pTx->set_serviceclass(*txSlotParameters.u8ServiceClass_);
                  }

                if(txSlotParameters.u64Dataratebps_)
                  {
                    pTx->set_dataratebps(*txSlotParameters.u64Dataratebps_);
                  }

                if(txSlotParameters.destination_)
                  {
                    pTx->set_destination(*txSlotParameters.destination_);
                  }
              }
            else if(std::holds_alternative<Events::TDMASchedule::RxSlotParameters>
                    (slotParameters.parameters_))
              {
                auto pRx = pSlot->mutable_tx();

                pSlot->set_type(EMANEMessage::TDMAScheduleEvent::Frame::Slot::SLOT_RX);

                const auto &
                  rxSlotParameters{std::get<Events::TDMASchedule::RxSlotParameters>
                                   (slotParameters.parameters_)};

                if(rxSlotParameters.u64FrequencyHz_)
                  {
                    pRx->set_frequencyhz(*rxSlotParameters.u64FrequencyHz_);
                  }
              }
            else
              {
                pSlot->set_type(EMANEMessage::TDMAScheduleEvent::Frame::Slot::SLOT_IDLE);
              }
          }
      }

    if(!msg.SerializeToString(&serialization))
      {
        throw SerializationException("unable to serialize TDMAScheduleEvent");
      }

    return serialization;
  }

  const TDMASchedule::SlotInfos & getSlotInfos() const
  {
    return slotInfos_;
  }

  const Frequencies & getFrequencies() const
  {
    return frequencies_;
  }

  const std::optional<TDMASchedule::SlotStructure> & getSlotStructure() const
  {
    return slotStructure_;
  }

private:
  TDMASchedule::SlotInfos slotInfos_;
  Frequencies frequencies_;
  std::optional<TDMASchedule::SlotStructure> slotStructure_;
  TDMASchedule::MultiFrameParameters multiFrameParameters_;
  TDMASchedule::FrameParameterMap frameParametertMap_;
  TDMASchedule::FrameSlotMap frameSlotMap_;
};


EMANE::Events::TDMAScheduleEvent::TDMAScheduleEvent(const Serialization & serialization):
  Event{IDENTIFIER},
  pImpl_{new Implementation{serialization}}{}

EMANE::Events::
TDMAScheduleEvent::TDMAScheduleEvent(const std::optional<TDMASchedule::SlotStructure> & slotStructure,
                                     const TDMASchedule::MultiFrameParameters & multiFrameParameters,
                                     const TDMASchedule::FrameParameterMap & frameParametertMap,
                                     const TDMASchedule::FrameSlotMap & frameSlotMap):
  Event{IDENTIFIER},
  pImpl_{new Implementation{slotStructure,
                            multiFrameParameters,
                            frameParametertMap,
                            frameSlotMap}}{}

EMANE::Events::TDMAScheduleEvent::~TDMAScheduleEvent(){}

const EMANE::Events::TDMASchedule::SlotInfos &
EMANE::Events::TDMAScheduleEvent::getSlotInfos() const
{
  return pImpl_->getSlotInfos();
}


const EMANE::Events::TDMAScheduleEvent::Frequencies &
EMANE::Events::TDMAScheduleEvent::getFrequencies() const
{
  return pImpl_->getFrequencies();
}

const std::optional<EMANE::Events::TDMASchedule::SlotStructure> &
EMANE::Events::TDMAScheduleEvent::getSlotStructure() const
{
  return  pImpl_->getSlotStructure();
}

EMANE::Serialization EMANE::Events::TDMAScheduleEvent::serialize() const
{
  return pImpl_->serialize();
}
