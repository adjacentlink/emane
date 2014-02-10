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

#ifndef EMANETIMERSERVICEUSER_HEADER_
#define EMANETIMERSERVICEUSER_HEADER_

#include "emane/types.h"

namespace EMANE
{
  /**
   * @class TimerServiceUser
   *
   * @brief TimerServiceUser interface
   */
  class TimerServiceUser
  {
  public:
    virtual ~TimerServiceUser(){}
    
    /**
     * Process a timed event
     *
     * @param eventId Identifier corresponding to the timer being processed
     * @param expireTime The time the timer was scheduled to expire
     * @param scheduleTime The Time the timer was scheduled
     * @param fire Time time The time the timer actually fired
     * @param arg Opaque timed event data
     *
     * @note All times are abosulte using CLOCK_REALTIME
     */
    virtual void processTimedEvent(TimerEventId eventId,
                                   const TimePoint & expireTime,
                                   const TimePoint & scheduleTime,
                                   const TimePoint & fireTime,
                                   const void * arg)
    {
      (void) eventId;
      (void) expireTime;
      (void) scheduleTime;
      (void) fireTime;
      (void) arg;
    };

  protected:
    TimerServiceUser(){}
  };
}


#endif // EMANETIMERSERVICEUSER_HEADER_
