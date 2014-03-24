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

#ifndef EMANEEVENTSLOCATIONEVENT_HEADER_
#define EMANEEVENTSLOCATIONEVENT_HEADER_

#include "emane/event.h"
#include "emane/events/location.h"
#include "emane/events/eventids.h"

#include <memory>

namespace EMANE
{
  namespace Events
  {
    /**
     * @class LocationEvent
     *
     * @brief A location event is usd to set the position, orientation
     * and velocity information for one or more NEMs.
     */
    class LocationEvent : public Event
    {
    public:
      /**
       * Creates a LocationEvent instance from a serialization
       *
       * @param serialization Message serialization
       *
       * @throw SerializationException when a valid message cannot be de-serialized
       */
      LocationEvent(const Serialization & serialization);
      
      /**
       * Creates a LocationEvent instance
       *
       * @param locations One or more Location instances
       */
      LocationEvent(const Locations & locations);
      
      /**
       * Creates a LocationEvent by copy
       *
       * @param rhs Instance to copy
       */
      LocationEvent(const LocationEvent & rhs);
      
      /**
       * Sets a LocationEvent by copy
       *
       * @param rhs Instance to copy
       */
      LocationEvent & operator=(const LocationEvent & rhs);

      /**
       * Creates a LocationEvent by moving
       *
       * @param rval Instance to move
       */     
      LocationEvent(LocationEvent && rval);

      /**
       * Sets a LocationEvent by moving
       *
       * @param rval Instance to move
       */ 
      LocationEvent & operator=(LocationEvent && rval);
      
      /**
       * Destroys an instance
       */
      ~LocationEvent();

      /**
       * Serializes the instance
       *
       * @throw SerializationException if the instance cannot be serialized
       */
      Serialization serialize() const override;

      /**
       * Gets the location entries
       *
       * @return location entries
       */
      const Locations & getLocations() const;
      
      enum {IDENTIFIER = EMANE_EVENT_LOCATION};
      
    private:
      class Implementation;
      std::unique_ptr<Implementation> pImpl_;
    };
  }
}


#endif // EMANEEVENTSLOCATIONEVENT_HEADER_
