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

#ifndef EMANEUTILSTIMER_HEADER_
#define EMANEUTILSTIMER_HEADER_

#include "emane/types.h"

#include <thread>
#include <mutex>
#include <chrono>
#include <functional>
#include <map>
#include <tuple>

namespace EMANE
{
  namespace Utils
  {
    /**
     * @class Timer
     *
     * @brief A timer class that uses Linux interval timers
     */
    class Timer
    {
    public:
      using TimerId = std::size_t;

      class TimerException{};

      /**
       * Creates an timer 
       *
       * @throw TimerException when an internal exception occurs.
       * 
       */
      Timer();
      
      
      /**
       * Destroys an timer
       */
      ~Timer();
      
      /**
       * Cancels a timer 
       *
       * @param timerId Id of timer to cancel
       *
       * @return bool flag true - canceled, false - not canceled
       *                  
       * @note Canceling an expired timer has not effect.
       */
      bool cancel(TimerId timerId);
      
      /**
       * Sechules a one shot timer
       *
       * @param fn A callable object that takes a single TimerId parameter
       * @param absoluteTimePoint Absolute time of the timeout
       */
      template <typename Function>
      TimerId schedule(Function fn,
                       const TimePoint & absoluteTimePoint);
      
      /**
       * Sechules an interval timer
       *
       * @param fn A callable object that takes a single TimerId parameter
       * @param absoluteTimePoint Absolute time of the timeout
       * @param interval Repeat interval
       */
      template <typename Function>
      TimerId scheduleInterval(Function fn,
                               const TimePoint & absoluteTimePoint,
                               const Microseconds & interval);
      
    private:
      using Callback = std::function<void(TimerId,
                                          const TimePoint &,
                                          const TimePoint &,
                                          const TimePoint &)>;

      using TimerInfo = std::tuple<TimerId,TimePoint,Microseconds,Callback,TimePoint>;
      using TimePointMap = std::multimap<std::pair<TimePoint,TimerId>,TimerInfo>;
      using TimerIdMap = std::map<TimerId,TimePoint>;
      
      TimePointMap timePointMap_;
      TimerIdMap timerIdMap_;
      bool bRunning_;
      TimerId timerId_;
      int iFd_;
      std::thread thread_;
      std::mutex mutex_;
      
      void scheduler();
      
      TimerId schedule(Callback,const TimePoint &,const Microseconds &);

      void schedule_i();

      void cancel_i();

      Timer(const Timer &) = delete;

      Timer & operator=(const Timer &) = delete;
    };
  }
}

#include "emane/utils/timer.inl"

#endif // EMANEUTILSTIMER_HEADER_
