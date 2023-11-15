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

#include "transponderstatuspublisher.h"

namespace
{
  const EMANE::StatisticTableLabels TransponderStatusLabels =
    {
      "Idx",
      "Rx Hz",
      "Rx Bw",
      "Rx Ant",
      "Rx Enable",
      "Action",
      "Tx Hz",
      "Tx Bw",
      "Tx Bps",
      "Tx Ant",
      "Tx dBm",
      "Tx Enable",
    };

  const EMANE::StatisticTableLabels TransponderStatusExLabels =
    {
      "Idx",
      "Tx U_Delay",
      "Tx Slots/Frame",
      "Tx Slot Size",
      "MTU",
    };
}

EMANE::Models::BentPipe::TransponderStatusPublisher::TransponderStatusPublisher(){}

EMANE::Models::BentPipe::TransponderStatusPublisher::~TransponderStatusPublisher(){}

void EMANE::Models::BentPipe::TransponderStatusPublisher::registerStatistics(StatisticRegistrar & statisticRegistrar)
{
  pTransponderStatusTable_ =
    statisticRegistrar.registerTable<NEMId>("TransponderStatusTable",
                                            TransponderStatusLabels,
                                            StatisticProperties::NONE,
                                            "Transponder status table");

  pTransponderStatusExTable_ =
    statisticRegistrar.registerTable<NEMId>("TransponderStatusExTable",
                                            TransponderStatusExLabels,
                                            StatisticProperties::NONE,
                                            "Transponder status extented table");
}

void EMANE::Models::BentPipe::TransponderStatusPublisher::addTransponder(const Transponder & transponder)
{
  TransponderIndex index = transponder.getIndex();

  if(!knownTransponder_.count(index))
    {
      const auto & configuration = transponder.getConfiguration();

      pTransponderStatusTable_->addRow(index,
                                       {Any{index},
                                        Any{configuration.getReceiveFrequencyHz()},
                                        Any{configuration.getReceiveBandwidthHz()},
                                        Any{configuration.getReceiveAntennaIndex()},
                                        Any{configuration.getReceiveEnable() ? "on" : "off"},
                                        configuration.getReceiveAction() == ReceiveAction::UBEND ?
                                        Any{"ubend"} :
                                        configuration.getReceiveAction() == ReceiveAction::PROCESS ?
                                        Any{"process"} :
                                        Any{"unknown"},
                                        Any{configuration.getTransmitFrequencyHz()},
                                        Any{configuration.getTransmitBandwidthHz()},
                                        Any{configuration.getTransmitDataRatebps()},
                                        Any{configuration.getTransmitAntennaIndex()},
                                        Any{configuration.getTransmitPowerdBm()},
                                        Any{configuration.getTransmitEnable() ? "on" : "off"}});


      pTransponderStatusExTable_->addRow(index,
                                         {Any{index},
                                          Any{configuration.getTransmitUbendDelay().count()},
                                          Any{configuration.getTransmitSlotsPerFrame()},
                                          Any{configuration.getTransmitSlotSize().count()},
                                          Any{transponder.getMTUBytes()}});

      knownTransponder_.emplace(index);
    }
}

void EMANE::Models::BentPipe::TransponderStatusPublisher::removeTransponder(const Transponder & transponder)
{
  TransponderIndex index = transponder.getIndex();

  if(knownTransponder_.count(index))
    {
      pTransponderStatusTable_->deleteRow(index);

      pTransponderStatusExTable_->deleteRow(index);

      knownTransponder_.erase(index);
    }
}
