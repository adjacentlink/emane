/*
 * Copyright (c) 2013-2014,2016 - Adjacent Link LLC, Bridgewater,
 * New Jersey
 * Copyright (c) 2010-2012 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANETIMERSERVICE_HEADER_
#define EMANETIMERSERVICE_HEADER_

#include "timerserviceexception.h"
#include "emane/timerserviceuser.h"
#include "emane/utils/singleton.h"
#include "emane/utils/timer.h"

namespace EMANE
{
  /**
   * @class TimerService
   *
   * @brief Platform timer service
   *
   * @details Realization of the TimerService interface.
   *
   */
  class TimerService : public Utils::Singleton<TimerService>
  {
  public:
    /**
     * @brief cancel a timed event
     *
     * @param eventId  event id to cancel
     */
    bool cancelTimedEvent(TimerEventId eventId);


    /**
     * @brief schedule a timed event.
     *
     * @param timePoint Schedule absolute time for timer to go off
     * @param arg Opaque data pointer
     * @param interval Timer reschedule interval. Default is one shot.
     * @param pTimerServiceUser Pointer to the TimerServiceUser
     *
     * @return identifier corresponding to the event id, or -1 on failure
     */
    TimerEventId scheduleTimedEvent(const TimePoint & timePoint,
                                    const void *arg,
                                    const Duration & interval,
                                    TimerServiceUser *pTimerServiceUser);


    /**
     * Schedules an generic interval timer callable
     *
     * @param fn A callable object
     * @param timePoint Absolute time of the timeout
     * @param interval Repeat interval
     */
    template <typename Function>
    TimerEventId schedule(Function fn,
                          const TimePoint & timePoint,
                          const Duration & interval = Duration::zero());


  protected:
    TimerService() = default;

  private:
    Utils::Timer timer_;
  };

  using TimerServiceSingleton = TimerService;
}

#include "timerservice.inl"

#endif //EMANETIMERSERVICE_HEADER_
