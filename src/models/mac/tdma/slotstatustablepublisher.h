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

#ifndef EMANEMODELSTDMASLOTSTATUSTABLEPUBLISHER_HEADER_
#define EMANEMODELSTDMASLOTSTATUSTABLEPUBLISHER_HEADER_

#include "emane/statisticregistrar.h"
#include <map>
#include <tuple>

namespace EMANE
{
  namespace Models
  {
    namespace TDMA
    {
      /**
       * @class SlotStatusTablePublisher
       *
       * @brief Slot statistic and statistic table status publisher.
       *
       * Slot statistic tables can be used to determine how well the
       * emulator is keeping up with the configured slotting
       * structure.
       */
      class SlotStatusTablePublisher
      {
      public:
        enum class Status
        {TX_GOOD,
            TX_MISSED,
            TX_TOOBIG,
            RX_GOOD,
            RX_MISSED,
            RX_IDLE,
            RX_TX,
            RX_TOOLONG};

        void registerStatistics(StatisticRegistrar & registrar);

        void update(std::uint32_t u32RelativeIndex,
                    std::uint32_t u32RelativeFrameIndex,
                    std::uint32_t u32RelativeSlotIndex,
                    Status status,
                    double dSlotRemainingRatio);

        void clear();

      private:
        StatisticTable<std::uint32_t> * pTxSlotStatusTable_;
        StatisticTable<std::uint32_t> * pRxSlotStatusTable_;

        using TxSlotCounterMap = std::map<std::uint32_t,
                                          std::tuple<std::uint64_t, // valid
                                                     std::uint64_t, // missed
                                                     std::uint64_t, // too big for slot
                                                     std::array<std::uint64_t,8> // quantile
                                                     >>;

        using RxSlotCounterMap = std::map<std::uint32_t,
                                          std::tuple<std::uint64_t, // valid
                                                     std::uint64_t, // missed
                                                     std::uint64_t, // rx during idle
                                                     std::uint64_t, // rx during tx
                                                     std::uint64_t, // rx too long for slot
                                                     std::array<std::uint64_t,8>>>;
        TxSlotCounterMap txSlotCounterMap_;
        RxSlotCounterMap rxSlotCounterMap_;

        void updateRx(std::uint32_t u32RelativeIndex,
                      std::uint32_t u32RelativeFrameIndex,
                      std::uint32_t u32RelativeSlotIndex,
                      Status status,
                      double dSlotRemainingRatio);

        void updateTx(std::uint32_t u32RelativeIndex,
                      std::uint32_t u32RelativeFrameIndex,
                      std::uint32_t u32RelativeSlotIndex,
                      Status status,
                      double dSlotRemainingRatio);

      };
    }
  }
}

#endif // EMANEMODELSTDMASLOTSTATUSTABLEPUBLISHER_HEADER_
