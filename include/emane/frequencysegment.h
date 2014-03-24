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

#ifndef EMANEFREQUENCYSEGMENT_HEADER_
#define EMANEFREQUENCYSEGMENT_HEADER_

#include <cstdint>
#include <list>

namespace EMANE
{
  /**
   * @class FrequencySegment
   *
   * @brief Holds the frequency, offset and duration of
   * a frequency segment.
   * 
   * @note Instances are immutable
   */
  class FrequencySegment
  {
  public:
    /**
     * Create a FrequencySegment instance 
     *
     * @param u64FrequencyHz Frequency in Hz
     * @param durationMicroseconds Segment duration in microseconds
     * @param offsetMicroseconds Segment offset relative to the start not the previous segment
     * @param dRxPowerdBm Receive power of the segment in dBm (upstream only)
     */
    FrequencySegment(std::uint64_t u64FrequencyHz,
                     const Microseconds & durationMicroseconds,
                     const Microseconds & offsetMicroseconds = Microseconds::zero(),
                     double dRxPowerdBm = 0);

    /**
     * Create a FrequencySegment from another instance
     *
     * @param rhs Instance to copy
     * @param dRxPowerdBm Receive power of the segment in dBm (upstream only)
     */
    FrequencySegment(const FrequencySegment & rhs,
                     double dRxPowerdBm);

    /**
     * Create a FrequencySegment from another instance
     *
     */
    FrequencySegment(const FrequencySegment & rhs) = default;

    /**
     * Gets the segment frequency in Hz
     *
     * @return frequency
     */
    std::uint64_t getFrequencyHz() const;

    /**
     * Gets the segment offset in microseconds
     *
     * @return offset reference
     */
    const Microseconds & getOffset() const;
    
    /**
     * Gets the segment duration in microseconds
     *
     * @return duration reference
     */
    const Microseconds & getDuration() const;

    /**
     * Gets the receive power of the segment in dBm
     *
     * @return receive power
     */
    double getRxPowerdBm() const;
    
  private:
    std::uint64_t u64FrequencyHz_;
    Microseconds durationMicroseconds_;
    Microseconds offsetMicroseconds_;
    double dRxPowerdBm_;
  };
  
  using FrequencySegments = std::list<FrequencySegment>;
}

#include "emane/frequencysegment.inl"

#endif // EMANEFREQUENCYSEGMENT_HEADER_
