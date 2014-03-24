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

#ifndef EMANEEVENTSCOMMEFFECTEVENT_HEADER_
#define EMANEEVENTSCOMMEFFECTEVENT_HEADER_

#include "emane/event.h"
#include "emane/events/commeffect.h"
#include "emane/events/eventids.h"

#include <memory>

namespace EMANE
{
  namespace Events
  {
    /**
     * @class CommEffectEvent
     *
     * @brief Comm Effect events are used to set asynchronous link
     * characteristics from one or more transmitting NEMs to the
     * receiving NEM.
     */
    class CommEffectEvent : public Event
    {
    public:
      /**
       * Creates a CommEffectEvent instance from a serialization
       *
       * @param serialization Message serialization
       *
       * @throw SerializationException when a valid message cannot be de-serialized
       */
      CommEffectEvent(const Serialization & serialization);
      

      /**
       * Creates a CommEffectEvent instance
       *
       * @param commEffects Comm Effect link characteristics
       */
      CommEffectEvent(const CommEffects & commEffects);
      
      /**
       * Creates a CommEffectEvent by copy
       *
       * @param rhs Instance to copy
       */
      CommEffectEvent(const CommEffectEvent & rhs);
      
      /**
       * Sets a CommEffectEvent by copy
       *
       * @param rhs Instance to copy
       */
      CommEffectEvent & operator=(const CommEffectEvent & rhs);
      
      /**
       * Creates a CommEffectEvent by moving
       *
       * @param rval Instance to move
       */
      CommEffectEvent(CommEffectEvent && rval);
      
      /**
       * Sets a CommEffectEvent by moving
       *
       * @param rval Instance to moving
       */
      CommEffectEvent & operator=(CommEffectEvent && rval);
      
      /**
       * Destroys an instance
       */
      ~CommEffectEvent();
      
      /**
       * Serializes the instance
       *
       * @throw SerializationException if the instance cannot be serialized
       */
      Serialization serialize() const override;
     
      /**
       * Gets the Comm Effect link characteristics
       *
       * @return comm effects
       */
      const CommEffects & getCommEffects() const;
      
      enum {IDENTIFIER = EMANE_EVENT_COMMEFFECT};
      
    private:
      class Implementation;
      std::unique_ptr<Implementation> pImpl_;
    };
  }
}

#endif // EMANEEVENTSCOMMEFFECTEVENT_HEADER_
