/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANEEVENTSCOMMEFFECT_HEADER_
#define EMANEEVENTSCOMMEFFECT_HEADER_

#include "emane/types.h"

#include <list>

namespace EMANE
{
  namespace Events
  {
    /**
     * @class CommEffect
     *
     * @brief A CommEffect entry holds the NEM Id of a transmitter and
     * the link effects to apply to received transmission.
     * 
     * @note Instances are immutable
     */
    class CommEffect
    {
    public:
      /**
       * Creates a CommEffect instance
       *
       * @param nemId NEM Id of the transmitter
       * @param lattency Latency in microseconds
       * @param jitter Jitter in microseconds
       * @param fProbabilityLoss Probability of loss
       * @param fProbabilityDuplicate Probability of duplication
       * @param u64UnicastBitRate Unicast bitrate in bps
       * @param u64BroadcastBitRate Broadcast bitrate in bps
       */
      CommEffect(NEMId nemId,
                 const Microseconds & lattency,
                 const Microseconds & jitter,
                 float fProbabilityLoss,
                 float fProbabilityDuplicate,
                 std::uint64_t u64UnicastBitRate,
                 std::uint64_t u64BroadcastBitRate);

      /**
       * Gets the NEM id
       *
       * @return NEM Id
       */
      NEMId getNEMId() const;
      
      /**
       * Gets the latency in microseconds
       *
       * @return latency
       */
      const Microseconds & getLatency() const;

      /**
       * Gets the jitter in microseconds
       */
      const Microseconds & getJitter() const;

      /**
       * Gets the loss probability
       *
       * @return probability
       */
      float getProbabilityLoss() const;

      /**
       * Gets the duplication probability
       *
       * @return probability
       */
      float getProbabilityDuplicate() const;

      /**
       * Getst the unicast bitrate in bps
       *
       * @return bitrate
       */
      std::uint64_t getUnicastBitRate() const;

      /**
       * Gets the broadcast bitrate in bps
       *
       * @return bitrate
       */
      std::uint64_t getBroadcastBitRate() const;

    private:
      NEMId nemId_;
      Microseconds latency_;
      Microseconds jitter_;
      float fProbabilityLoss_;
      float fProbabilityDuplicate_;
      std::uint64_t u64UnicastBitRate_;
      std::uint64_t u64BroadcastBitRate_;
    };

    using CommEffects = std::list<CommEffect>;
  }
}

#include "emane/events/commeffect.inl"

#endif // EMANEEVENTSCOMMEFFECT_HEADER_
