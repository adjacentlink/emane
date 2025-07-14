/*
 * Copyright (c) 2025 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANEEVENTSPATHLOSSEX_HEADER_
#define EMANEEVENTSPATHLOSSEX_HEADER_

#include "emane/types.h"

#include <list>
#include <map>

namespace EMANE
{
  namespace Events
  {
    /**
     * @class PathlossEx
     *
     * @brief A pathloss ex(tended) entry holds the source NEM Id and
     * the pathlosses to apply to received transmissions specified by
     * transmission center frequency.
     *
     * @see PathlossExEvent
     *
     * @note Instances are immutable
     */
    class PathlossEx
    {
    public:
      using FrequencyPathlossMap = std::map<std::uint64_t,float>;

      /**
       * Creates a PathlossEx instance
       *
       * @param id NEM id of the transmitter
       * @param fPathlossdB The pathloss from transmitter to receiver in dB
       * @param frequencyPathlossMap Mapping of center frequency in Hz to pathloss in dB
       */
      PathlossEx(NEMId id,
                 FrequencyPathlossMap && frequencyPathlossMap);

      /**
       * Creates a PathlossEx instance
       *
       * @param id NEM id of the transmitter
       * @param fPathlossdB The pathloss from transmitter to receiver in dB
       * @param frequencyPathlossMap Mapping of center frequency in Hz to pathloss in dB
       */
      PathlossEx(NEMId id,
                 const FrequencyPathlossMap & frequencyPathlossMap);

      /**
       * Gets the NEM Id of of the transmitter
       *
       * @return id
       */
      NEMId getNEMId() const;

      /**
       * Gets the mapping of center frequency in Hz to pathloss in dB
       *
       * @return frequency pathloss map
       */
      const FrequencyPathlossMap & getFrequencyPathlossMap() const;

    private:
      NEMId id_;
      FrequencyPathlossMap frequencyPathlossMap_;
    };

    using PathlossExs = std::list<PathlossEx>;
  }
}

#include "emane/events/pathlossex.inl"

#endif // EMANEEVENTSPATHLOSSEX_HEADER_
