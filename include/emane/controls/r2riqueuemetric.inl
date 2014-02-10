/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
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

inline
EMANE::Controls::R2RIQueueMetric::R2RIQueueMetric(std::uint32_t u32QueueId,
                                                  std::uint32_t u32MaxSize,
                                                  std::uint32_t u32CurrentDepth,
                                                  std::uint32_t u32NumDiscards,
                                                  const Microseconds & avgDelay):
  u32QueueId_{u32QueueId},
  u32MaxSize_{u32MaxSize},
  u32CurrentDepth_{u32CurrentDepth},
  u32NumDiscards_{u32NumDiscards},
  avgDelay_{avgDelay}{}

inline
EMANE::Controls::R2RIQueueMetric::~R2RIQueueMetric(){}

inline
std::uint32_t EMANE::Controls::R2RIQueueMetric::getQueueId() const
{
  return u32QueueId_;
}


inline
std::uint32_t EMANE::Controls::R2RIQueueMetric::getMaxSize() const
{
  return u32MaxSize_;
}

    
inline
std::uint32_t EMANE::Controls::R2RIQueueMetric::getCurrentDepth() const
{
  return u32CurrentDepth_;
}

    
inline
std::uint32_t EMANE::Controls::R2RIQueueMetric::getNumDiscards() const
{
  return u32NumDiscards_;
}


inline
const EMANE::Microseconds & EMANE::Controls::R2RIQueueMetric::getAvgDelay() const
{
  return avgDelay_;
}
