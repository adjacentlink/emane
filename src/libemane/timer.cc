/*
 * Copyright (c) 2013-2014,2025-2026 - Adjacent Link LLC, Bridgewater,
 *  New Jersey
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

#include "emane/utils/timer.h"
#include "emane/utils/timerexception.h"
#include "logservice.h"
#include "activityservice.h"

#include <sys/timerfd.h>
#include <vector>
#include <unistd.h>

EMANE::Utils::Timer::Timer():

  bRunning_{},
  timerId_{},
  pTimerBackend_{}
{}

EMANE::Utils::Timer::~Timer()
{
  // synchronization block
  {
    std::lock_guard<std::mutex> m(mutex_);

    if(bRunning_)
      {
        bRunning_ = false;

        if(pTimerBackend_)
          {
            pTimerBackend_->unblock();
          }
      }
  }

  if(thread_.joinable())
    {
      thread_.join();
    }
}

bool EMANE::Utils::Timer::cancel(TimerId timerId)
{
  std::lock_guard<std::mutex> m(mutex_);

  if(!pTimerBackend_)
    {
      throw TimerException{"backend timer not set cannot cancel"};
    }

  bool bCancel{};

  auto iter = timerIdMap_.find(timerId);

  if(iter != timerIdMap_.end())
    {
      bool bReschedule{};

      // if the timer to cancel is the earliest timer
      //  we need to cancel the timer and reschedule
      if(std::get<0>(timePointMap_.begin()->second) == timerId)
        {
          bReschedule = true;
          cancel_i();
        }

      // clean up the timer stores
      timePointMap_.erase({iter->second,timerId});

      timerIdMap_.erase(iter);

      // if necessary reschedule the new earliest timer
      if(bReschedule)
        {
          schedule_i();
        }

      bCancel = true;
    }

  return bCancel;
}

// precondition - mutex is acquired
void EMANE::Utils::Timer::cancel_i()
{
  // cancel any existing timer
  pTimerBackend_->cancel();

}

EMANE::Utils::Timer::TimerId
EMANE::Utils::Timer::schedule(Callback callback,
                              const TimePoint & timePoint,
                              const Duration & interval)
{
  std::unique_lock<std::mutex> lock(mutex_);

  if(!pTimerBackend_)
    {
      throw TimerException{"backend timer not set cannot schedule"};
    }

  bool bReschedule{};

  // if no timers are present we need to schedule this timer
  if(timePointMap_.empty())
    {
      // need to reschedule
      bReschedule = true;
    }
  else if(timePoint < timePointMap_.begin()->first.first)
    {
      // this time is earlier than the current earliest timer
      //  so we need to cancel the current timer and reschedule
      cancel_i();
      bReschedule = true;
    }

  timerId_+=1;

  timePointMap_.insert(std::make_pair(std::make_pair(timePoint,timerId_),
                                      std::make_tuple(timerId_,
                                                      timePoint,
                                                      interval,
                                                      callback,
                                                      Clock::now())));

  timerIdMap_.insert(std::make_pair(timerId_,timePoint));

  // if necessary rechedule the new earliest timer
  if(bReschedule)
    {
      schedule_i();
    }

  return timerId_;
}

// precondition - mutex is acquired
void EMANE::Utils::Timer::schedule_i()
{
  // only schedule the earliest time if one is present
  if(!timePointMap_.empty())
    {
      auto & timePoint = timePointMap_.begin()->first.first;

      pTimerBackend_->set(timePoint);
    }
}


void EMANE::Utils::Timer::scheduler()
{
  std::vector<TimerInfo> expired;

  while(1)
    {
      // if there are any expired timers execute their
      //  respective callbacks
      for(const auto & info : expired)
        {
          const TimerId & timerId{std::get<0>(info)};
          const TimePoint & expireTime{std::get<1>(info)};
          const Callback & callback{std::get<3>(info)};
          const TimePoint & scheduleTime{std::get<4>(info)};

          try
            {
              callback(timerId,
                       expireTime,
                       scheduleTime,
                       Clock::now());

              ACTIVITY_SERVICE_NOTIFY_WORK_DEQUEUE();
            }
          catch(std::exception & exp)
            {
              LogServiceSingleton::instance()->log(ERROR_LEVEL,
                                                   "Timer:%s callback"
                                                   " exception: %s",
                                                   __func__,
                                                   exp.what());
            }
        }

      // clear out expired timer list
      expired.clear();

      // wait for an interval timer to expire
      if(pTimerBackend_->wait() > 0)
        {
          std::unique_lock<std::mutex> lock(mutex_);

          auto now = Clock::now();

          if(!bRunning_)
            {
              break;
            }

          // an interval timer expired - iterate over all scheduled
          // timers starting with the earliest and expire any that have
          // expired. Stop iterating once you find a timer that is still
          // valid (in the future).
          for(const auto & entry : timePointMap_)
            {
              const auto & timePoint = std::get<1>(entry.second);

              if(now >= timePoint)
                {
                  ACTIVITY_SERVICE_NOTIFY_WORK_ENQUEUE();

                  expired.push_back(std::move(entry.second));
                }
              else
                {
                  break;
                }
            }

          // remove any expired timers from the timer stores and reschedule
          // any that have a repeat interval
          for(const auto & info : expired)
            {
              TimerId timerId{};
              TimePoint expireTime{};
              Duration interval{};
              Callback callback{};

              std::tie(timerId,expireTime,interval,callback,std::ignore) = info;

              timePointMap_.erase({expireTime,timerId});

              timerIdMap_.erase(timerId);

              if(interval != Duration::zero())
                {
                  expireTime += interval;

                  timePointMap_.insert(std::make_pair(std::make_pair(expireTime,timerId),
                                                      std::make_tuple(timerId,
                                                                      expireTime,
                                                                      interval,
                                                                      callback,
                                                                      now)));

                  timerIdMap_.insert(std::make_pair(timerId,expireTime));
                }
            }

          // reschedule the new earliest time, if one is present
          schedule_i();

          ACTIVITY_SERVICE_NOTIFY_WORK_DEQUEUE();
        }
    }
}

void
EMANE::Utils::Timer::setBackend(TimerBackend * pTimerBackend)
{
  std::unique_lock<std::mutex> lock(mutex_);

  if(pTimerBackend_)
    {
      throw TimerException{"backend timer is set cannot set again"};
    }

  pTimerBackend_ = pTimerBackend;

  bRunning_ = true;

  thread_ = std::move(std::thread{&Timer::scheduler,this});
}
