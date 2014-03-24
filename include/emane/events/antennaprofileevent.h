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

#ifndef EMANEEVENTSANTENNAPROFILEEVENT_HEADER_
#define EMANEEVENTSANTENNAPROFILEEVENT_HEADER_

#include "emane/event.h"
#include "emane/antennaprofile.h"
#include "emane/events/eventids.h"

#include <memory>

namespace EMANE
{
  namespace Events
  {
    /**
     * @class AntennaProfileEvent
     *
     * @brief An antenna profile event is used to set the antenna profile selection
     * and pointing information for one or more NEMs.
     */
    class AntennaProfileEvent : public Event
    {
    public:
      /**
       * Creates an AntennaProfileEvent instance from a serialization
       *
       * @param serialization Message serialization
       *
       * @throw SerializationException when a valid message cannot be de-serialized
       */
      AntennaProfileEvent(const Serialization & serialization);
      
      /**
       * Creates an AntennaProfileEvent instance
       *
       * @param  antennaProfiles One or more AntennaProfile instances
       */
      AntennaProfileEvent(const AntennaProfiles & antennaProfiles);

      /**
       * Creates an AntennaProfileEvent by copy
       *
       * @param rhs Instance to copy
       */
      AntennaProfileEvent(const AntennaProfileEvent & rhs);

      /**
       * Sets an AntennaProfileEvent by copy
       *
       * @param rhs Instance to copy
       */
      AntennaProfileEvent & operator=(const AntennaProfileEvent & rhs);

      /**
       * Creates an AntennaProfileEvent by moving
       *
       * @param rval Instance to move
       */     
      AntennaProfileEvent(AntennaProfileEvent && rval);

      /**
       * Sets an AntennaProfileEvent by moving
       *
       * @param rval Instance to move
       */ 
      AntennaProfileEvent & operator=(AntennaProfileEvent && rval);

      /**
       * Destroys an instance
       */
      ~AntennaProfileEvent();

      /**
       * Serializes the instance
       *
       * @throw SerializationException if the instance cannot be serialized
       */
      Serialization serialize() const override;

      /**
       * Gets the antenna profile entries
       *
       * @return antenna profile entries
       */
      const AntennaProfiles & getAntennaProfiles() const;
      
      enum {IDENTIFIER = EMANE_EVENT_ANTENNA_PROFILE};
      
    private:
      class Implementation;
      std::unique_ptr<Implementation> pImpl_;
    };
  }
}


#endif // EMANEEVENTSANTENNAPROFILEEVENT_HEADER_
