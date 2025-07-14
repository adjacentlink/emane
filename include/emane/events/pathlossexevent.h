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

#ifndef EMANEEVENTSPATHLOSSEXEVENT_HEADER_
#define EMANEEVENTSPATHLOSSEXEVENT_HEADER_

#include "emane/event.h"
#include "emane/events/pathlossex.h"
#include "emane/events/eventids.h"

#include <memory>

namespace EMANE
{
  namespace Events
  {
    /**
     * @class PathlossExEvent
     *
     * @brief A pathloss ex(tended) event is used to set the pathloss
     * from one or more transmitting NEMs to a receiving NEM allowing
     * for different pathloss values per transmission center
     * frequency.
     */
    class PathlossExEvent : public Event
    {
    public:
      /**
       * Creates a PathlossExEvent instance from a serialization
       *
       * @param serialization Message serialization
       *
       * @throw SerializationException when a valid message cannot be de-serialized
       */
      PathlossExEvent(const Serialization & serialization);

      /**
       * Creates a PathlossExEvent instance
       *
       * @param pathlosses One or more PathlossEntry instances
       */
      PathlossExEvent(const PathlossExs & pathlosses);

      /**
       * Creates a PathlossExEvent by copy
       *
       * @param rhs Instance to copy
       */
      PathlossExEvent(const PathlossExEvent & rhs);

      /**
       * Sets a PathlossExEvent by copy
       *
       * @param rhs Instance to copy
       */
      PathlossExEvent & operator=(const PathlossExEvent & rhs);

      /**
       * Creates a PathlossExEvent by moving
       *
       * @param rval Instance to move
       */
      PathlossExEvent(PathlossExEvent && rval);

      /**
       * Sets a PathlossExEvent by moving
       *
       * @param rval Instance to move
       */
      PathlossExEvent & operator=(PathlossExEvent && rval);

      /**
       * Destroys an instance
       */
      ~PathlossExEvent();

      /**
       * Serializes the instance
       *
       * @throw SerializationException if the instance cannot be serialized
       */
      Serialization serialize() const override;

      /**
       * Gets the transmitter NEM pathloss extended entries
       *
       * @return pathloss extended entries
       */
      const PathlossExs & getPathlossExs() const;

      enum {IDENTIFIER = EMANE_EVENT_PATHLOSS_EX};

    private:
      class Implementation;
      std::unique_ptr<Implementation> pImpl_;
    };
  }
}

#endif // EMANEEVENTSPATHLOSSEXEVENT_HEADER_
