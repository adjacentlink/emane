/*
 * Copyright (c) 2015-2016 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include <cstdint>
#include <tuple>

class EMANE::Events::TDMAScheduleEvent::Implementation
{
public:
  Implementation(const Serialization & serialization):
    bHasStructure_{}
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

        structure_ = SlotStructure{structure.bandwidthhz(),
                                   u32FramesPerMultiFrame,
                                   u32SlotsPerFrame,
                                   Microseconds{structure.slotdurationmicroseconds()},
                                   Microseconds{structure.slotoverheadmicroseconds()}};

        bHasStructure_ = true;

        slotInfos_.reserve(u32FramesPerMultiFrame * u32SlotsPerFrame);
      }

    if(bHasStructure_)
      {
        // pre-built a complete schedule defaulting all slots to idle
        for(unsigned i =  0; i < u32FramesPerMultiFrame; ++i)
          {
            for(unsigned j =  0; j < u32SlotsPerFrame; ++j)
              {
                slotInfos_.push_back({SlotInfo::Type::IDLE,i,j});
              }
          }
      }

    for(const auto & frame :msg.frames())
      {
        std::uint32_t u32FrameIndex = frame.index();

        if(bHasStructure_ && u32FrameIndex >= u32FramesPerMultiFrame)
          {
            throw makeException<SerializationException>("TDMAScheduleEvent : Frame %lu index out of range",
                                                        u32FrameIndex);
          }

        std::set<std::uint32_t> presentSlots{};

        for(const auto & slot : frame.slots())
          {
            SlotInfo::Type type{SlotInfo::Type::IDLE};
            std::uint64_t u64FrequencyHz{};
            std::uint64_t u64DataRatebps{};
            std::uint8_t u8ServiceClass{};
            double dPowerdBm{};
            NEMId destination{};

            std::uint32_t u32SlotIndex = slot.index();

            if(bHasStructure_ && u32SlotIndex >= u32SlotsPerFrame)
              {
                 throw makeException<SerializationException>("TDMAScheduleEvent : Frame %lu Slot %lu slot index out of range",
                                                             u32FrameIndex,
                                                             u32SlotIndex);
              }

            presentSlots.insert(u32SlotIndex);

            switch(slot.type())
              {
              case EMANEMessage::TDMAScheduleEvent::Frame::Slot::SLOT_TX:
                {
                  const auto & tx = slot.tx();

                  type = SlotInfo::Type::TX;

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
                      throw makeException<SerializationException>("TDMAScheduleEvent : Frame %lu Slot %lu has undeterminable frequency",
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
                      throw makeException<SerializationException>("TDMAScheduleEvent : Frame %lu Slot %lu has undeterminable datarate",
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
                      throw makeException<SerializationException>("TDMAScheduleEvent : Frame %lu Slot %lu has undeterminable class",
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
                      throw makeException<SerializationException>("TDMAScheduleEvent : Frame %lu Slot %lu has undeterminable power",
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

                  type = SlotInfo::Type::RX;

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
                      throw makeException<SerializationException>("TDMAScheduleEvent : Frame %lu Slot %lu has undeterminable frequency",
                                                                  u32FrameIndex,
                                                                  u32SlotIndex);
                    }
                }
                break;

              case EMANEMessage::TDMAScheduleEvent::Frame::Slot::SLOT_IDLE:
                type = SlotInfo::Type::IDLE;
                break;
              }


            frequencies_.insert(u64FrequencyHz);

            if(bHasStructure_)
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

        if(bHasStructure_)
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
                        throw makeException<SerializationException>("TDMAScheduleEvent : Frame %lu Slot %lu has undeterminable frequency",
                                                                    u32FrameIndex,
                                                                    i);
                      }

                    frequencies_.insert(u64FrequencyHz);

                    slotInfos_[u32FrameIndex * u32SlotsPerFrame + i] = {SlotInfo::Type::RX,
                                                                        u32FrameIndex,
                                                                        i,
                                                                        u64FrequencyHz};
                  }
              }
          }
      }
  }

  const SlotInfos & getSlotInfos() const
  {
    return slotInfos_;
  }

  const Frequencies & getFrequencies() const
  {
    return frequencies_;
  }

  std::pair<const SlotStructure &,bool> getSlotStructure() const
  {
    return {structure_,bHasStructure_};
  }

private:
  SlotInfos slotInfos_;
  Frequencies frequencies_;
  SlotStructure structure_;
  bool bHasStructure_;
};


EMANE::Events::TDMAScheduleEvent::TDMAScheduleEvent(const Serialization & serialization):
  Event{IDENTIFIER},
  pImpl_{new Implementation{serialization}}{}

EMANE::Events::TDMAScheduleEvent::~TDMAScheduleEvent(){}

const EMANE::Events::SlotInfos & EMANE::Events::TDMAScheduleEvent::getSlotInfos() const
{
  return pImpl_->getSlotInfos();
}


const EMANE::Events::TDMAScheduleEvent::Frequencies &
EMANE::Events::TDMAScheduleEvent::getFrequencies() const
{
  return pImpl_->getFrequencies();
}

std::pair<const EMANE::Events::SlotStructure &,bool>
EMANE::Events::TDMAScheduleEvent::getSlotStructure() const
{
  return  pImpl_->getSlotStructure();
}
