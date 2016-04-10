/*
 * Copyright (c) 2013-2014,2016 - Adjacent Link LLC, Bridgewater,
 * New Jersey
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

#include <functional>
#include <chrono>

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
                                            const Duration & interval = Duration::zero()) = 0;


    /**
     * Schedules an generic interval timer callable
     *
     * @param fn A callable object
     * @param absoluteTimePoint Absolute time of the timeout
     * @param interval Repeat interval
     */
    template <typename Function>
    TimerEventId schedule(Function fn,
                          const TimePoint & timePoint,
                          const Duration & interval = Duration::zero());

    using TimerCallback = std::function<void(const TimePoint &, // expireTime,
                                             const TimePoint &, // scheduleTime,
                                             const TimePoint &)>;// fireTime

  protected:
    virtual TimerEventId schedule_i(TimerCallback callback,
                                    const TimePoint & timePoint,
                                    const Duration & interval) = 0;

    TimerServiceProvider(){};
  };
}

#include "emane/timerserviceprovider.inl"

#endif //EMNAETIMERSERVICEPROVIDER_HEADER_


/**
 * @page TimerService Timer Service
 *
 * The @ref EMANE::TimerServiceProvider "TimerServiceProvider" is used by components to schedule
 * timed events.
 *
 * @section SchedulingATimedEvent Scheduling a Timed Event
 *
 * To schedule a timed event a component uses the @ref EMANE::TimerServiceProvider::scheduleTimedEvent
 * "TimerServiceProvider::scheduleTimedEvent" method. An optional opaque data pointer and reschedule
 * interval can be specified.
 *
 * The @ref EMANE::TimerServiceProvider "TimerServiceProvider" is accessed via the
 * @ref EMANE::PlatformServiceProvider "PlatformServiceProvider". All components are given a reference
 * to the @ref EMANE::PlatformServiceProvider "PlatformServiceProvider" when they are constructed.
 *
 * @snippet src/models/mac/rfpipe/maclayer.cc timerservice-scheduletimedevent-snippet
 *
 * An attempt to schedule a timed event will always succeed. If the requested expiration time is in the
 * past it will be scheduled to immediately fire.
 *
 * @section HandlingATimedEvent Handling a Timed Event
 *
 * When a timed event expires it is pushed onto the NEM's functor queue as a
 * @ref EMANE::TimerServiceUser::processTimedEvent "TimerServiceUser::processTimedEvent" method. The
 * @ref EMANE::TimerServiceUser::processTimedEvent "processTimedEvent" method arguments contain:
 * - The Timer Id of the expired timer
 * - The requested expiration time of the timer
 * - The actual time the timer was scheduled by the framework
 * - The actual time the timer fired and was handled by the framework
 * - The opaque data pointer used when the timer was scheduled
 *
 * The three time arguments are used by the framework to track timer service performance but may be of
 * interest to the component.
 *
 * The following example uses the opaque data pointer parameter to create a generic mechanism for
 * executing arbitrary callbacks. There are many ways to process timed events. An alternative is to
 * use an object passed via the opaque data pointer to interpret the required action and handle it
 * accordingly.
 *
 * @snippet src/models/mac/rfpipe/maclayer.cc timerservice-processtimedevent-snippet
 *
 * @section CancelingATimedEvent Canceling a Timed Event
 *
 * A scheduled timed event can be canceled using @ref EMANE::TimerServiceProvider::cancelTimedEvent
 * "TimerServiceProvider::cancelTimedEvent". It is not an error to attempt to cancel a timed event that
 * has already expired. The @ref EMANE::TimerServiceProvider::cancelTimedEvent "cancelTimedEvent" method
 * will return @a true if the event was canceled and @a false if the event has already expired prior to
 * the cancel attempt.
 *
 * @snippet src/models/mac/rfpipe/maclayer.cc timerservice-canceltimedevent-snippet
 */
