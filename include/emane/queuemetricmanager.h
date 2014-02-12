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
 * * Neither the name of Adjacent Link, LLC nor the names of its
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


#ifndef EMANEQUEUEMETRICMANAGER_HEADER_
#define EMANEQUEUEMETRICMANAGER_HEADER_

#include "emane/types.h"
#include "emane/controls/r2riqueuemetriccontrolmessage.h"

namespace EMANE
{
  /**
   *
   * @class QueueMetricManager
   *
   * @brief Manager Queue metrics
   *
   */
  class QueueMetricManager
  { 
  public:
    /**
     * Creates a QueueMetricManager instance
     *
     * @param id NEM id
     */
    QueueMetricManager(NEMId id);
    
    /**
     * Destroys an instance
     */
    ~QueueMetricManager();
    

    /**
     * Updates the queue metric
     *
     * @param u16QueueId The queue id
     * @param u32QueueSize The queue size
     * @param u32QueueDepth The queue depth
     * @param u32NumDiscards The queue discards
     * @param queueDelay The queue delay time
     *
     */
    void updateQueueMetric(std::uint16_t u16QueueId,
                           std::uint32_t u32QueueSize,
                           std::uint32_t u32QueueDepth,
                           std::uint32_t u32NumDiscards,
                           const Microseconds & queueDelay);
  
    /**
     * Adds a queue metric entry
     *
     * @param u16QueueId The queue id
     * @param u32QueueSize The queue max size
     *
     * @return @a true if successful
     *
     */
    bool addQueueMetric(std::uint16_t u16QueueId, std::uint32_t u32QueueSize);

    /**
     * Removes a queue metric entry
     *
     * @param u16QueueId The queue id
     *
     * @return @a true if successful
     */
    bool removeQueueMetric(std::uint16_t u16QueueId);

    /**
     * Gets the queue metrics
     *
     * @return metrics
     */
    Controls::R2RIQueueMetrics getQueueMetrics();

  private:
    class Implementation;

    std::unique_ptr<Implementation> pImpl_;

    QueueMetricManager(const QueueMetricManager &) = delete;

    QueueMetricManager & operator=(const QueueMetricManager &) = delete;
  };
}

#endif // EMANEQUEUEMETRICMANAGER_HEADER_
