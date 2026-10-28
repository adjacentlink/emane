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

#ifndef EMANELOCALTIMERBACKEND_HEADER_
#define EMANELOCALTIMERBACKEND_HEADER_

#include "emane/utils/timerbackend.h"
#include "emane/utils/timerexception.h"

#include <sys/timerfd.h>
#include <unistd.h>

namespace EMANE
{
  class LocalTimerBackend : public Utils::TimerBackend
  {
  public:
    LocalTimerBackend():
      iFd_{}
    {
      // create an interval timer with CLOCK_REALTIME
      if((iFd_ = timerfd_create(CLOCK_REALTIME,0)) < 0)
        {
          throw Utils::TimerException{"unable to create timerfd"};
        }
    }

    ~LocalTimerBackend()
    {
      close(iFd_);
    };

    void set(const TimePoint & absoluteTimePoint) override
    {
      auto timeSinceEpoch = absoluteTimePoint.time_since_epoch();

      auto sec = std::chrono::duration_cast<Seconds>(timeSinceEpoch);

      auto nsec = std::chrono::duration_cast<Nanoseconds>(timeSinceEpoch % Seconds{1});

      // schedule the interval timer
      itimerspec spec{{0,0},{sec.count(),nsec.count()}};

      timerfd_settime(iFd_,TFD_TIMER_ABSTIME,&spec,nullptr);
    }

    int wait() override
    {
      std::uint64_t u64Expired{};
      return read(iFd_,&u64Expired,sizeof(u64Expired));
    }

    void cancel() override
    {
      itimerspec spec{{0,0},{0,0}};
      timerfd_settime(iFd_,0,&spec,nullptr);
    }

    void signal() override {}

    void unblock() override
    {
      //insert a new timer to unblock
      // scheduler thread
      itimerspec spec{{0,0},{0,1}};

      timerfd_settime(iFd_,0,&spec,nullptr);
    }

    std::optional<TimePoint> getEarliestExpireTime() override
    {
      return {};
    }

  private:
    int iFd_;
  };
}

#endif // EMANEACTIVITYSERVICE_HEADER_
