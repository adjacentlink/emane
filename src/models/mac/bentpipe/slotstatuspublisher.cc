/*
 * Copyright (c) 2015-2018,2023 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "slotstatuspublisher.h"

void EMANE::Models::BentPipe::SlotStatusPublisher::registerStatistics(StatisticRegistrar & statisticRegistrar)
{
  pTxSlotStatusTable_ =
    statisticRegistrar.registerTable<TransponderIndex>("TxSlotStatusTable",
                                                       {"Transponder",
                                                        "Valid",
                                                        "Missed",
                                                        ".25",
                                                        ".50",
                                                        ".75",
                                                        "1.0",
                                                        "1.25",
                                                        "1.50",
                                                        "1.75",
                                                        ">1.75"},
                                                       StatisticProperties::NONE,
                                                       "Shows the number of Tx slot opportunities per transponder"
                                                       " that were valid or missed based on slot timing deadlines");
}

void EMANE::Models::BentPipe::SlotStatusPublisher::update(TransponderIndex transponderIndex,
                                                          Status status,
                                                          double dSlotPortionRatio)
{
  auto iter = txSlotCounterMap_.find(transponderIndex);

  if(iter == txSlotCounterMap_.end())
    {
      iter = txSlotCounterMap_.insert({transponderIndex,
          std::make_tuple(0ULL,0ULL,std::array<std::uint64_t,8>())}).first;

      pTxSlotStatusTable_->addRow(transponderIndex,
                                  {Any{transponderIndex},
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
  auto & quantile = std::get<2>(iter->second);

  switch(status)
    {
    case Status::TX_GOOD:
      pTxSlotStatusTable_->setCell(transponderIndex,1,Any{++valid});
      break;
    case Status::TX_MISSED:
      pTxSlotStatusTable_->setCell(transponderIndex,2,Any{++missed});
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

  pTxSlotStatusTable_->setCell(transponderIndex,
                               iQuantileIndex+3,
                               Any{++quantile[iQuantileIndex]});
}
