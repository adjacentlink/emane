/*
 * Copyright (c) 2015 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANEEVENTSTDMASCHEDULEEVENT_HEADER_
#define EMANEEVENTSTDMASCHEDULEEVENT_HEADER_

#include "emane/event.h"
#include "emane/events/eventids.h"
#include "emane/events/slotinfo.h"
#include "emane/events/slotstructure.h"

#include <memory>

namespace EMANE
{
  namespace Events
  {
    class TDMAScheduleEvent : public Event
    {
    public:
      using Frequencies = std::set<uint64_t>;

      /**
       * Creates a TDMAScheduleEvent instance from a serialization
       *
       * @param serialization Message serialization
       *
       * @throw SerializationException when a valid message cannot be de-serialized
       */
      TDMAScheduleEvent(const Serialization & serialization);


      /**
       * Destroys an instance
       */
      ~TDMAScheduleEvent();

      const SlotInfos & getSlotInfos() const;


      const Frequencies & getFrequencies() const;

      std::pair<const SlotStructure &,bool> getSlotStructure() const;


      enum {IDENTIFIER = EMANE_EVENT_TDMA_SCHEDULE};

    private:
      class Implementation;
      std::unique_ptr<Implementation> pImpl_;
    };
  }
}

#endif // EMANEEVENTSTDMASCHEDULEEVENT_HEADER_
