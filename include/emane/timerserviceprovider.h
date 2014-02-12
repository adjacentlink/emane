/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2010 - DRS CenGen, LLC, Columbia, Maryland
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
 * * Neither the name of DRS CenGen, LLC nor the names of its
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

#ifndef EMNAETIMERSERVICEPROVIDER_HEADER_
#define EMNAETIMERSERVICEPROVIDER_HEADER_

#include "emane/types.h"

namespace EMANE
{

  /**
   * @class TimerServiceProvider
   *
   * @brief Timer service interface the allows for scheduling
   * timers that are placed on the NEMQueuedLayer functor queue
   * for processing.
   */
  class TimerServiceProvider
  {
   public:
     virtual ~TimerServiceProvider(){};


    /**
     * Cancels a timed event
     *
     * @param eventId  event id
     *
     * @note It is not an error to cancel an event that
     * has already occured.
     *
     */
     virtual bool cancelTimedEvent(TimerEventId eventId) = 0;


    /**
     * Schedules a timed event
     *
     * @param timePoint Schedule absolute time for timer to go off
     * @param arg Opaque data pointer
     * @param interval Timer reschedule interval. Default is one shot.
     *
     * @return timer id
     *
     */
    virtual TimerEventId scheduleTimedEvent(const TimePoint & timePoint,
                                            const void * arg = nullptr, 
                                            const Microseconds & interval = Microseconds::zero()) = 0;
    
    
  protected:
     TimerServiceProvider(){};
  };
}

#endif //EMNAETIMERSERVICEPROVIDER_HEADER_
