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

#include "slotstatustablepublisher.h"

void EMANE::Models::TDMA::SlotStatusTablePublisher::registerStatistics(StatisticRegistrar & statisticRegistrar)
{
  pTxSlotStatusTable_ =
    statisticRegistrar.registerTable<std::uint32_t>("TxSlotStatusTable",
      {"Index","Frame","Slot","Valid","Missed","Big",".25",".50",".75","1.0","1.25","1.50","1.75",">1.75"},
      StatisticProperties::NONE,
      "Shows the number of Tx slot opportunities that were valid or missed based on slot timing deadlines");

  pRxSlotStatusTable_ =
    statisticRegistrar.registerTable<std::uint32_t>("RxSlotStatusTable",
      {"Index","Frame","Slot","Valid","Missed","Idle","Tx","Long","Freq",".25",".50",".75","1.0","1.25","1.50","1.75",">1.75"},
      StatisticProperties::NONE,
      "Shows the number of Rx slot receptions that were valid or missed based on slot timing deadlines");
}

void EMANE::Models::TDMA::SlotStatusTablePublisher::clear()

{
  pTxSlotStatusTable_->clear();
  pRxSlotStatusTable_->clear();

  txSlotCounterMap_.clear();
  rxSlotCounterMap_.clear();
}

void EMANE::Models::TDMA::SlotStatusTablePublisher::update(std::uint32_t u32RelativeIndex,
                                                           std::uint32_t u32RelativeSlotIndex,
                                                           std::uint32_t u32RelativeFrameIndex,
                                                           Status status,
                                                           double dSlotRemainingRatio)
{
   switch(status)
    {
    case Status::TX_GOOD:
    case Status::TX_MISSED:
    case Status::TX_TOOBIG:
      updateTx(u32RelativeIndex,
               u32RelativeSlotIndex,
               u32RelativeFrameIndex,
               status,
               dSlotRemainingRatio);
      break;

    case Status::RX_GOOD:
    case Status::RX_MISSED:
    case Status::RX_IDLE:
    case Status::RX_TX:
    case Status::RX_TOOLONG:
    case Status::RX_WRONGFREQ:
      updateRx(u32RelativeIndex,
               u32RelativeSlotIndex,
               u32RelativeFrameIndex,
               status,
               dSlotRemainingRatio);
      break;
    }
}

void EMANE::Models::TDMA::SlotStatusTablePublisher::updateTx(std::uint32_t u32RelativeIndex,
                                                             std::uint32_t u32RelativeSlotIndex,
                                                             std::uint32_t u32RelativeFrameIndex,
                                                             Status status,
                                                             double dSlotRemainingRatio)
{
  auto iter = txSlotCounterMap_.find(u32RelativeIndex);

  if(iter == txSlotCounterMap_.end())
    {
      iter = txSlotCounterMap_.insert({u32RelativeIndex,std::make_tuple(0ULL,0ULL,0ULL,std::array<std::uint64_t,8>())}).first;

      pTxSlotStatusTable_->addRow(u32RelativeIndex,
                               {Any{u32RelativeIndex},
                                   Any{u32RelativeSlotIndex},
                                     Any{u32RelativeFrameIndex},
                                       Any{0L},
                                         Any{0L},
                                           Any{0L},
                                             Any{0L},
                                               Any{0L},
                                                 Any{0L},
                                                   Any{0L},
                                                       Any{0L},
                                                         Any{0L},
                                                           Any{0L},
                                                             Any{0L}});

    }

  auto & valid = std::get<0>(iter->second);
  auto & missed = std::get<1>(iter->second);
  auto & toobig = std::get<2>(iter->second);
  auto & quantile = std::get<3>(iter->second);

  switch(status)
    {
    case Status::TX_GOOD:
      pTxSlotStatusTable_->setCell(u32RelativeIndex,3,Any{++valid});
      break;
    case Status::TX_MISSED:
      pTxSlotStatusTable_->setCell(u32RelativeIndex,4,Any{++missed});
      break;
    case Status::TX_TOOBIG:
      pTxSlotStatusTable_->setCell(u32RelativeIndex,5,Any{++toobig});
      break;
    default:
      break;
    }

  double dSlotPassedRatio = 1 - dSlotRemainingRatio;
  int iQuantileIndex {};

  if(dSlotPassedRatio <= 0.25)
    {
      iQuantileIndex = 0;
    }
  else if(dSlotPassedRatio <= 0.50)
    {
      iQuantileIndex = 1;
    }
  else if(dSlotPassedRatio <= 0.75)
    {
      iQuantileIndex = 2;
    }
  else if(dSlotPassedRatio <= 1.00)
    {
      iQuantileIndex = 3;
    }
  else if(dSlotPassedRatio <= 1.25)
    {
      iQuantileIndex = 4;
    }
  else if(dSlotPassedRatio <= 1.50)
    {
      iQuantileIndex = 5;
    }
  else if(dSlotPassedRatio <= 1.75)
    {
      iQuantileIndex = 6;
    }
  else
    {
      iQuantileIndex = 7;
    }

  pTxSlotStatusTable_->setCell(u32RelativeIndex,iQuantileIndex+6,Any{++quantile[iQuantileIndex]});
}

void EMANE::Models::TDMA::SlotStatusTablePublisher::updateRx(std::uint32_t u32RelativeIndex,
                                                             std::uint32_t u32RelativeSlotIndex,
                                                             std::uint32_t u32RelativeFrameIndex,
                                                             Status status,
                                                             double dSlotRemainingRatio)
{
  auto iter = rxSlotCounterMap_.find(u32RelativeIndex);

  if(iter == rxSlotCounterMap_.end())
    {
      iter = rxSlotCounterMap_.insert({u32RelativeIndex,std::make_tuple(0ULL,0ULL,0ULL,0ULL,0ULL,0ULL,std::array<std::uint64_t,8>())}).first;

      pRxSlotStatusTable_->addRow(u32RelativeIndex,
                                  {Any{u32RelativeIndex},
                                      Any{u32RelativeSlotIndex},
                                        Any{u32RelativeFrameIndex},
                                          Any{0L},
                                            Any{0L},
                                              Any{0L},
                                                Any{0L},
                                                  Any{0L},
                                                    Any{0L},
                                                      Any{0L},
                                                        Any{0L},
                                                          Any{0L},
                                                            Any{0L},
                                                              Any{0L},
                                                                Any{0L},
                                                                  Any{0L},
                                                                    Any{0L}});

    }

  auto & valid = std::get<0>(iter->second);
  auto & missed = std::get<1>(iter->second);
  auto & rxidle = std::get<2>(iter->second);
  auto & rxtx = std::get<3>(iter->second);
  auto & rxtoolong = std::get<4>(iter->second);
  auto & rxwrongfreq = std::get<5>(iter->second);
  auto & quantile = std::get<6>(iter->second);

  switch(status)
    {
    case Status::RX_GOOD:
      pRxSlotStatusTable_->setCell(u32RelativeIndex,3,Any{++valid});
      break;
    case Status::RX_MISSED:
      pRxSlotStatusTable_->setCell(u32RelativeIndex,4,Any{++missed});
      break;
    case Status::RX_IDLE:
      pRxSlotStatusTable_->setCell(u32RelativeIndex,5,Any{++rxidle});
      break;
    case Status::RX_TX:
      pRxSlotStatusTable_->setCell(u32RelativeIndex,6,Any{++rxtx});
      break;
    case Status::RX_TOOLONG:
      pRxSlotStatusTable_->setCell(u32RelativeIndex,7,Any{++rxtoolong});
      break;
    case Status::RX_WRONGFREQ:
      pRxSlotStatusTable_->setCell(u32RelativeIndex,8,Any{++rxwrongfreq});
      break;
    default:
      break;
    }

  // ratio is either how much is left (< 1) or how much went over
  double dSlotPassedRatio{dSlotRemainingRatio};

  if(dSlotRemainingRatio < 1)
    {
      dSlotPassedRatio = 1 - dSlotRemainingRatio;
    }

  int iQuantileIndex {};

  if(dSlotPassedRatio <= 0.25)
    {
      iQuantileIndex = 0;
    }
  else if(dSlotPassedRatio <= 0.50)
    {
      iQuantileIndex = 1;
    }
  else if(dSlotPassedRatio <= 0.75)
    {
      iQuantileIndex = 2;
    }
  else if(dSlotPassedRatio <= 1.00)
    {
      iQuantileIndex = 3;
    }
  else if(dSlotPassedRatio <= 1.25)
    {
      iQuantileIndex = 4;
    }
  else if(dSlotPassedRatio <= 1.50)
    {
      iQuantileIndex = 5;
    }
  else if(dSlotPassedRatio <= 1.75)
    {
      iQuantileIndex = 6;
    }
  else
    {
      iQuantileIndex = 7;
    }

  pRxSlotStatusTable_->setCell(u32RelativeIndex,iQuantileIndex+9,Any{++quantile[iQuantileIndex]});
}
