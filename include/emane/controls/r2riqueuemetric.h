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

#ifndef EMANECONTROLSR2RIQUEUEMETRIC_HEADER_
#define EMANECONTROLSR2RIQUEUEMETRIC_HEADER_

#include "emane/types.h"

#include <cstdint>
#include <list>

namespace EMANE
{
  namespace Controls
  {
    /**
     * @class R2RIQueueMetric
     *
     * @brief R2RI queue metrics are used in conjunction with the
     * R2RIQueueMetricControlMessage to inform an NEM's transport layer
     * of MAC  queue state.
     *
     * @note Instances are immutable
     */
    class R2RIQueueMetric
    {
    public:
      /**
       * Creates an R2RIQueueMetric instance
       *
       * @param u32QueueId Queue Id, 0 based
       * @param u32MaxSize Max queue size
       * @param u32CurrentDepth Current Queue depth
       * @param u32NumDiscards Number of discards
       * @param avgDelay Average queue delay in microseconds
       *
       * @note All data is over the report interval
       *
       * @see R2RISelfMetricControlMessage for report interval
       */
      R2RIQueueMetric(std::uint32_t u32QueueId,
                      std::uint32_t u32MaxSize,
                      std::uint32_t u32CurrentDepth,
                      std::uint32_t u32NumDiscards,
                      const Microseconds & avgDelay);
      
      /**
       * Destroys an instance
       */
      ~R2RIQueueMetric();
      
      /**
       * Gets the queue id
       *
       * @return Queue id
       */
      std::uint32_t getQueueId() const;
      
      /**
       * Gets the max queue size
       * over the report interval
       *
       * @return max size
       */
      std::uint32_t getMaxSize() const;
      
      /**
       * Gets the current queue depth
       * over the report interval
       *
       * @return current depth
       */
      std::uint32_t getCurrentDepth() const;
      
      /**
       * Gets the number of queue discards
       * over the report interval
       *
       * @return number of discards
       */
      std::uint32_t getNumDiscards() const;
      
      /**
       * Gets the average queue delay in microseconds
       * over the report interval
       *
       * @return average queue delay
       */
      const Microseconds & getAvgDelay() const;
      
    private:
      const std::uint32_t u32QueueId_;
      const std::uint32_t u32MaxSize_;
      const std::uint32_t u32CurrentDepth_;
      const std::uint32_t u32NumDiscards_;
      const Microseconds  avgDelay_;
    };
    
    typedef std::list<R2RIQueueMetric> R2RIQueueMetrics;
  }
}

#include "r2riqueuemetric.inl"

#endif // EMANECONTROLSR2RIQUEUEMETRIC_HEADER_
