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

#include "emane/utils/timer.h"

#include <sys/timerfd.h>
#include <vector>
#include <unistd.h>

EMANE::Utils::Timer::Timer():

  bRunning_{true},
  timerId_{},
  iFd_{}
{
  // create an interval timer with CLOCK_REALTIME
  if((iFd_ = timerfd_create(CLOCK_REALTIME,0)) < 0)
    {
      throw TimerException{};
    }

  // spawn the scheduler thread
  thread_ = std::move(std::thread{&Timer::scheduler,this});
};

EMANE::Utils::Timer::~Timer()
{
  // synchronization block
  {
    std::lock_guard<std::mutex> m(mutex_);
    
    bRunning_ = false;
  }

  // insert a new timer to unblock 
  //  scheduler thread
  itimerspec spec{{0,0},{0,1}};
 
  timerfd_settime(iFd_,0,&spec,nullptr);
  
  thread_.join();

  close(iFd_);
}

bool EMANE::Utils::Timer::cancel(TimerId timerId)
{
  std::lock_guard<std::mutex> m(mutex_);

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
  itimerspec spec{{0,0},{0,0}};
  timerfd_settime(iFd_,0,&spec,nullptr);
}

EMANE::Utils::Timer::TimerId
EMANE::Utils::Timer::schedule(Callback callback,
                              const TimePoint & timePoint,
                              const Duration & interval)
{
  std::unique_lock<std::mutex> lock(mutex_);

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
      auto timeout = timePointMap_.begin()->first.first;
      
      timespec ts;

      Microseconds timeusec{std::chrono::duration_cast<Microseconds>(timeout.time_since_epoch())};
      
      ts.tv_sec = timeusec.count() / 1000000;
      
      ts.tv_nsec = (timeusec.count() % 1000000) * 1000;
      
      // schedule the interval timer
      itimerspec spec{{0,0},ts};
      
      timerfd_settime(iFd_,TFD_TIMER_ABSTIME,&spec,nullptr);
    }
}

  
void EMANE::Utils::Timer::scheduler()
{
  std::vector<TimerInfo> expired;

  std::uint64_t u64Expired{};

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
            }
          catch(...)
            {}
        }
      
      // clear out expired timer list
      expired.clear();
     
      // wait for an interval timer to expire
      if(read(iFd_,&u64Expired,sizeof(u64Expired)) > 0)
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
              TimePoint scheduleTime{};
              
              std::tie(timerId,expireTime,interval,callback,scheduleTime) = info;
              
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
        }
    }
}
