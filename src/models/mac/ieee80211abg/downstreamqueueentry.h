/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2008 - DRS CenGen, LLC, Columbia, Maryland
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
 * * Neither the name of DRS CenGen, LLC nor the names of its
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

#ifndef EMANEMODELSIEEE80211ABGDOWNSTREAMQUEUEENTRY_HEADER_
#define EMANEMODELSIEEE80211ABGDOWNSTREAMQUEUEENTRY_HEADER_

#include "emane/maclayerimpl.h"
#include "emane/types.h"


namespace EMANE
{
  namespace Models
  {
    namespace IEEE80211ABG
    {
      /**
       *
       * @brief structure defines the mac downstream packet queue entry
       *
       */
      struct DownstreamQueueEntry
      {
        EMANE::DownstreamPacket pkt_;                     // payload
        TimePoint               acquireTime_;             // acquire time from network layer abs
        Microseconds            txOpMicroseconds_;        // pkt tx op duration 
        Microseconds            durationMicroseconds_;    // pkt duration
        TimePoint               preTxDelayTime_;          // pre tx delay time abs
        Microseconds            postTxDelayMicroseconds_; // post tx delay duration
        TimePoint               txTime_;                  // tx time abs
        std::uint16_t           u16Seq_;                  // sequence number
        std::uint8_t            u8Category_;              // queue index
        std::uint8_t            numRetries_;              // number of retries count
        std::uint8_t            maxRetries_;              // max retry count
        bool                    bRtsCtsEnable_;           // rts cts enable
        bool                    bCollisionOccured_;       // flag that a tx collision occured
        std::uint64_t           u64DataRatebps_;          // data rate bps

        DownstreamQueueEntry():
          pkt_{EMANE::DownstreamPacket{EMANE::PacketInfo{0,0,0,{}},nullptr,0}},
          acquireTime_{},
          txOpMicroseconds_{},
          durationMicroseconds_{},
          preTxDelayTime_{},
          postTxDelayMicroseconds_{},
          txTime_{}, 
          u16Seq_{}, 
          u8Category_{},
          numRetries_{}, 
          maxRetries_{},
          bRtsCtsEnable_{false},
          bCollisionOccured_{false},
          u64DataRatebps_{}
        { } 

        DownstreamQueueEntry(EMANE::DownstreamPacket & pkt, 
                             const TimePoint & tv,
                             const Microseconds & txOpMicroseconds,
                             std::uint8_t u8Category, 
                             std::uint8_t maxRetries):
          pkt_{std::move(pkt)},
          acquireTime_{tv},
          txOpMicroseconds_{txOpMicroseconds},
          durationMicroseconds_{},
          preTxDelayTime_{},
          postTxDelayMicroseconds_{},
          txTime_{}, 
          u16Seq_{}, 
          u8Category_{u8Category},
          numRetries_{}, 
          maxRetries_{maxRetries},
          bRtsCtsEnable_{false},
          bCollisionOccured_{false},
          u64DataRatebps_{}
        { }

        DownstreamQueueEntry & operator=(DownstreamQueueEntry && entry)
        {
          pkt_ = std::move(entry.pkt_);
          acquireTime_ = entry.acquireTime_;
          txOpMicroseconds_ = entry.txOpMicroseconds_;
          durationMicroseconds_ = entry.durationMicroseconds_;
          preTxDelayTime_ = entry.preTxDelayTime_;
          postTxDelayMicroseconds_ = entry.postTxDelayMicroseconds_;
          txTime_ = entry.txTime_;
          u16Seq_ = entry.u16Seq_;
          u8Category_= entry.u8Category_;
          numRetries_ = entry.numRetries_;
          maxRetries_= entry.maxRetries_;
          bRtsCtsEnable_= entry.bRtsCtsEnable_;
          bCollisionOccured_ = entry. bCollisionOccured_;
          u64DataRatebps_ = entry.u64DataRatebps_;

          return *this;
        }

        DownstreamQueueEntry(DownstreamQueueEntry && entry):
          pkt_{std::move(entry.pkt_)},
          acquireTime_{entry.acquireTime_},
          txOpMicroseconds_{entry.txOpMicroseconds_},
          durationMicroseconds_{entry.durationMicroseconds_},
          preTxDelayTime_{entry.preTxDelayTime_},
          postTxDelayMicroseconds_{entry.postTxDelayMicroseconds_},
          txTime_{entry.txTime_},
          u16Seq_{entry.u16Seq_},
          u8Category_{entry.u8Category_},
          numRetries_{entry.numRetries_},
          maxRetries_{entry.maxRetries_},
          bRtsCtsEnable_{entry.bRtsCtsEnable_},
          bCollisionOccured_{entry. bCollisionOccured_},
          u64DataRatebps_{entry.u64DataRatebps_}
        {}
      };
    }
  }
}
#endif //EMANEMODELSIEEE80211ABGDOWNSTREAMQUEUEENTRY_HEADER_
