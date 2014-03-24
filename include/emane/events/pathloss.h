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

#ifndef EMANEEVENTSPATHLOSS_HEADER_
#define EMANEEVENTSPATHLOSS_HEADER_

#include "emane/types.h"

#include <list>

namespace EMANE
{
  namespace Events
  {
    /**
     * @class Pathloss
     *
     * @brief A pathloss entry holds the source NEM Id and the forward and
     * reverse pathloss to apply to received transmissions.
     *
     * @see PathlossEvent
     *
     * @note Instances are immutable
     */
    class Pathloss
    {
    public:
      /**
       * Creates a Pathloss instance
       *
       * @param id NEM id of the transmitter
       * @param fForwardPathlossdB The pathloss from transmitter to receiver in dB
       * @param fReversePathlossdB The pathloss from the receiver to the transmitter in dB
       */
      Pathloss(NEMId id,
               float fForwardPathlossdB,
               float fReversePathlossdB);
      
      /**
       * Gets the NEM Id of of the transmitter
       *
       * @return id
       */
      NEMId getNEMId() const;
      
      /**
       * Gets the pathloss from the transmitter to receiver in dB
       *
       * @return pathloss
       */
      float getForwardPathlossdB() const;

      /**
       * Gets the pathloss from the receiver to the transmitter in dB
       *
       * @return pathloss
       */
      float getReversePathlossdB() const;
      
    private:
      EMANE::NEMId id_;
      float fForwardPathlossdB_;
      float fReversePathlossdB_;
    };
    
    using Pathlosses = std::list<Pathloss>;
  }
}

#include "emane/events/pathloss.inl"

#endif // EMANEEVENTSPATHLOSS_HEADER_
