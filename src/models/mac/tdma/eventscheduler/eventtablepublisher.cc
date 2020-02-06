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

#include "emane/models/frameworkphy/eventtablepublisher.h"

EMANE::Models::TDMA::EventTablePublisher::EventTablePublisher():
  bHasStructure_{}{}

void EMANE::Models::TDMA::EventTablePublisher::registerStatistics(StatisticRegistrar & statisticRegistrar)
{
  pScheduleTable_ =
    statisticRegistrar.registerTable<std::uint32_t>("scheduler.ScheduleInfoTable",
      {"Index","Frame","Slot","Type","Frequency","Data Rate","Power","Class","Destination"},
      StatisticProperties::NONE,
      "Shows the current TDMA schedule.");

  pStrutureTable_ =
    statisticRegistrar.registerTable<std::string>("scheduler.StructureInfoTable",
      {"Name","Value"},
      StatisticProperties::NONE,
      "Shows the current TDMA structure: slot size, slot overhead, number of slots"
      " per frame, number of frames per multiframe and transceiver bandwidth.");
}

void EMANE::Models::TDMA::EventTablePublisher::clear()
{
  pStrutureTable_->clear();
  pScheduleTable_->clear();
  scheduleIndexSet_.clear();
  structure_ = {};
  bHasStructure_ = false;
}

void EMANE::Models::TDMA::EventTablePublisher::replace(const Events::SlotInfos & slotInfos,
                                                       const Events::SlotStructure & structure)
{
  pScheduleTable_->clear();
  scheduleIndexSet_.clear();
  structure_ = structure;

  auto loadStructureTable =
    bHasStructure_ ? &StatisticTable<std::string>::setRow : &StatisticTable<std::string>::addRow;

  (pStrutureTable_->*loadStructureTable)("bandwidth",{Any{"bandwidth"},Any{structure.getBandwidth()}});

  (pStrutureTable_->*loadStructureTable)("frames",{Any{"frames"},Any{structure.getFramesPerMultiFrame()}});

  (pStrutureTable_->*loadStructureTable)("slots",{Any{"slots"},Any{structure.getSlotsPerFrame()}});

  (pStrutureTable_->*loadStructureTable)("slotduration",{Any{"slotduration"},Any{structure.getSlotDuration().count()}});

  (pStrutureTable_->*loadStructureTable)("slotoverhead",{Any{"slotoverhead"},Any{structure.getSlotOverhead().count()}});

  bHasStructure_ = true;

  update(slotInfos);
}

void EMANE::Models::TDMA::EventTablePublisher::update(const Events::SlotInfos & slotInfos)
{
  for(const auto & slotInfo : slotInfos)
    {
      auto index =
        slotInfo.getFrameIndex() * structure_.getSlotsPerFrame() + slotInfo.getSlotIndex();

      std::vector<Any> anys{};

      switch(slotInfo.getType())
        {
        case Events::SlotInfo::Type::TX:
          anys = {Any{index},
                  Any{slotInfo.getFrameIndex()},
                  Any{slotInfo.getSlotIndex()},
                  Any{"TX"},
                  Any{slotInfo.getFrequency()},
                  Any{slotInfo.getDataRate()},
                  Any{slotInfo.getPower()},
                  Any{slotInfo.getServiceClass()},
                  Any{slotInfo.getDestination()}};
          break;

        case Events::SlotInfo::Type::RX:
          anys = {Any{index},
                  Any{slotInfo.getFrameIndex()},
                  Any{slotInfo.getSlotIndex()},
                  Any{"RX"},
                  Any{slotInfo.getFrequency()},
                  Any{""},
                  Any{""},
                  Any{""},
                  Any{""}};
          break;

        case Events::SlotInfo::Type::IDLE:
          anys = {Any{index},
                  Any{slotInfo.getFrameIndex()},
                  Any{slotInfo.getSlotIndex()},
                  Any{"IDLE"},
                  Any{""},
                  Any{""},
                  Any{""},
                  Any{""},
                  Any{""}};
          break;
        }

      if(scheduleIndexSet_.count(index))
        {
          pScheduleTable_->setRow(index,anys);
        }
      else
        {
          pScheduleTable_->addRow(index,anys);
          scheduleIndexSet_.insert(index);
        }
    }
}
