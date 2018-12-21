/*
 * Copyright (c) 2015-2016,2018 - Adjacent Link LLC, Bridgewater,
 * New Jersey
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
                                                    {"Index",
                                                        "Frame",
                                                        "Slot",
                                                        "Valid",
                                                        "Missed",
                                                        "Big",
                                                        ".25",
                                                        ".50",
                                                        ".75",
                                                        "1.0",
                                                        "1.25",
                                                        "1.50",
                                                        "1.75",
                                                        ">1.75"},
                                                    StatisticProperties::NONE,
                                                    "Shows the number of Tx slot opportunities that were valid or missed based on slot timing deadlines");

  pRxSlotStatusTable_ =
    statisticRegistrar.registerTable<std::uint32_t>("RxSlotStatusTable",
                                                    {"Index",
                                                        "Frame",
                                                        "Slot",
                                                        "Valid",
                                                        "Missed",
                                                        "Idle",
                                                        "Tx",
                                                        "Long",
                                                        "Freq",
                                                        "Lock",
                                                        ".25",
                                                        ".50",
                                                        ".75",
                                                        "1.0",
                                                        "1.25",
                                                        "1.50",
                                                        "1.75",
                                                        ">1.75"},
                                                    StatisticProperties::NONE,
                                                    "Shows the number of Rx slot receptions that were valid or missed based on slot timing deadlines");


  pTxSlotValid_ =
    statisticRegistrar.registerNumeric<std::uint64_t>("numTxSlotValid",
                                                      StatisticProperties::NONE,
                                                      "Number of valid Tx slots");

  pTxSlotErrorMissed_ =
    statisticRegistrar.registerNumeric<std::uint64_t>("numTxSlotErrorMissed",
                                                      StatisticProperties::NONE,
                                                      "Number of Tx slot missed errors.");

  pTxSlotErrorTooBig_ =
    statisticRegistrar.registerNumeric<std::uint64_t>("numTxSlotErrorTooBig",
                                                      StatisticProperties::NONE,
                                                      "Number of Tx slot too big errors.");

  pRxSlotValid_ =
    statisticRegistrar.registerNumeric<std::uint64_t>("numRxSlotValid",
                                                      StatisticProperties::NONE,
                                                      "Number of valid Rx slots");

  pRxSlotErrorMissed_ =
    statisticRegistrar.registerNumeric<std::uint64_t>("numRxSlotErrorMissed",
                                                      StatisticProperties::NONE,
                                                      "Number of Rx slot missed errors.");

  pRxSlotErrorRxDuringIdle_ =
    statisticRegistrar.registerNumeric<std::uint64_t>("numRxSlotErrorRxDuringIdle",
                                                      StatisticProperties::NONE,
                                                      "Number of Rx slot rx during idle errors.");

  pRxSlotErrorRxDuringTx_ =
    statisticRegistrar.registerNumeric<std::uint64_t>("numRxSlotErrorRxDuringTx",
                                                      StatisticProperties::NONE,
                                                      "Number of Rx slot during tx errors.");

  pRxSlotErrorRxTooLong_ =
    statisticRegistrar.registerNumeric<std::uint64_t>("numRxSlotErrorRxTooLong",
                                                      StatisticProperties::NONE,
                                                      "Number of Rx slot rx too long errors.");

  pRxSlotErrorRxWrongFrequency_ =
    statisticRegistrar.registerNumeric<std::uint64_t>("numRxSlotErrorRxWrongFrequency",
                                                      StatisticProperties::NONE,
                                                      "Number of Rx slot rx wrong frequency errors.");

  pRxSlotErrorRxLock_ =
    statisticRegistrar.registerNumeric<std::uint64_t>("numRxSlotErrorRxLock",
                                                      StatisticProperties::NONE,
                                                      "Number of Rx slot rx lock errors.");
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
    case Status::RX_LOCK:
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
                                                             double dSlotPortionRatio)
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
      ++*pTxSlotValid_;
      pTxSlotStatusTable_->setCell(u32RelativeIndex,3,Any{++valid});
      break;
    case Status::TX_MISSED:
      ++*pTxSlotErrorMissed_;
      pTxSlotStatusTable_->setCell(u32RelativeIndex,4,Any{++missed});
      break;
    case Status::TX_TOOBIG:
      ++*pTxSlotErrorTooBig_;
      pTxSlotStatusTable_->setCell(u32RelativeIndex,5,Any{++toobig});
      break;
    default:
      break;
    }

  int iQuantileIndex {};

  if(dSlotPortionRatio <= 0.25)
    {
      iQuantileIndex = 0;
    }
  else if(dSlotPortionRatio <= 0.50)
    {
      iQuantileIndex = 1;
    }
  else if(dSlotPortionRatio <= 0.75)
    {
      iQuantileIndex = 2;
    }
  else if(dSlotPortionRatio <= 1.00)
    {
      iQuantileIndex = 3;
    }
  else if(dSlotPortionRatio <= 1.25)
    {
      iQuantileIndex = 4;
    }
  else if(dSlotPortionRatio <= 1.50)
    {
      iQuantileIndex = 5;
    }
  else if(dSlotPortionRatio <= 1.75)
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
                                                             double dSlotPortionRatio)
{
  auto iter = rxSlotCounterMap_.find(u32RelativeIndex);

  if(iter == rxSlotCounterMap_.end())
    {
      iter = rxSlotCounterMap_.insert({u32RelativeIndex,std::make_tuple(0ULL,0ULL,0ULL,0ULL,0ULL,0ULL,0ULL,std::array<std::uint64_t,8>())}).first;

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
                                                                    Any{0L},
                                                                      Any{0L}});

    }

  auto & valid = std::get<0>(iter->second);
  auto & missed = std::get<1>(iter->second);
  auto & rxidle = std::get<2>(iter->second);
  auto & rxtx = std::get<3>(iter->second);
  auto & rxtoolong = std::get<4>(iter->second);
  auto & rxwrongfreq = std::get<5>(iter->second);
  auto & rxlock = std::get<6>(iter->second);
  auto & quantile = std::get<7>(iter->second);

  switch(status)
    {
    case Status::RX_GOOD:
      ++*pRxSlotValid_;
      pRxSlotStatusTable_->setCell(u32RelativeIndex,3,Any{++valid});
      break;
    case Status::RX_MISSED:
      ++*pRxSlotErrorMissed_;
      pRxSlotStatusTable_->setCell(u32RelativeIndex,4,Any{++missed});
      break;
    case Status::RX_IDLE:
      ++*pRxSlotErrorRxDuringIdle_;
      pRxSlotStatusTable_->setCell(u32RelativeIndex,5,Any{++rxidle});
      break;
    case Status::RX_TX:
      ++*pRxSlotErrorRxDuringTx_;
      pRxSlotStatusTable_->setCell(u32RelativeIndex,6,Any{++rxtx});
      break;
    case Status::RX_TOOLONG:
      ++*pRxSlotErrorRxTooLong_;
      pRxSlotStatusTable_->setCell(u32RelativeIndex,7,Any{++rxtoolong});
      break;
    case Status::RX_WRONGFREQ:
      ++*pRxSlotErrorRxWrongFrequency_;
      pRxSlotStatusTable_->setCell(u32RelativeIndex,8,Any{++rxwrongfreq});
      break;
    case Status::RX_LOCK:
      ++*pRxSlotErrorRxLock_;
      pRxSlotStatusTable_->setCell(u32RelativeIndex,9,Any{++rxlock});
      break;
    default:
      break;
    }

  int iQuantileIndex {};

  if(dSlotPortionRatio <= 0.25)
    {
      iQuantileIndex = 0;
    }
  else if(dSlotPortionRatio <= 0.50)
    {
      iQuantileIndex = 1;
    }
  else if(dSlotPortionRatio <= 0.75)
    {
      iQuantileIndex = 2;
    }
  else if(dSlotPortionRatio <= 1.00)
    {
      iQuantileIndex = 3;
    }
  else if(dSlotPortionRatio <= 1.25)
    {
      iQuantileIndex = 4;
    }
  else if(dSlotPortionRatio <= 1.50)
    {
      iQuantileIndex = 5;
    }
  else if(dSlotPortionRatio <= 1.75)
    {
      iQuantileIndex = 6;
    }
  else
    {
      iQuantileIndex = 7;
    }

  pRxSlotStatusTable_->setCell(u32RelativeIndex,iQuantileIndex+10,Any{++quantile[iQuantileIndex]});
}
