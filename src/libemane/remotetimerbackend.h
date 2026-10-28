/*
 * Copyright (c) 2025-2026 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANEREMOTETIMERBACKEND_HEADER_
#define EMANEREMOTETIMERBACKEND_HEADER_

#include "emane/utils/timerbackend.h"
#include "emane/utils/timerexception.h"
#include "logservice.h"
#include "activityservice.h"

#include <sys/eventfd.h>
#include <unistd.h>

namespace
{
  const uint64_t one{1};
}


namespace EMANE
{
  class RemoteTimerBackend : public Utils::TimerBackend
  {
  public:
    RemoteTimerBackend():
      iFd_{}
    {
      if((iFd_ = eventfd(0,0)) < 0)
        {
          throw Utils::TimerException{"unable to create eventfd"};
        }
    }

    ~RemoteTimerBackend()
    {
      close(iFd_);
    };

    void set(const TimePoint & scheduleTime) override
    {
      LogServiceSingleton::instance()->log(DEBUG_LEVEL,
                                           "RemoteTimerBackend::%s "
                                           " time: %zu",
                                           __func__,
                                           std::chrono::duration_cast<std::chrono::microseconds>
                                           (scheduleTime.time_since_epoch()).count());

      if(scheduleTime > Clock::now())
        {
          earliestExpireTime_ = scheduleTime;
        }
    }

    int wait() override
    {
      LogServiceSingleton::instance()->log(DEBUG_LEVEL,
                                           "RemoteTimerBackend::%s",
                                           __func__);

      std::uint64_t u64Expired{};
      return read(iFd_,&u64Expired,sizeof(u64Expired));
    }

    void cancel() override
    {}

    void signal() override
    {
      LogServiceSingleton::instance()->log(DEBUG_LEVEL,
                                           "RemoteTimerBackend::%s",
                                           __func__);

      // dequeue happens after the timer service has run for the time
      // associated with this signal
      ACTIVITY_SERVICE_NOTIFY_WORK_ENQUEUE();

      write(iFd_,&one,sizeof(one));
    }

    void unblock() override
    {
      LogServiceSingleton::instance()->log(DEBUG_LEVEL,
                                           "RemoteTimerBackend::%s",
                                           __func__);

      write(iFd_,&one,sizeof(one));
    }

    std::optional<TimePoint> getEarliestExpireTime() override
    {
      LogServiceSingleton::instance()->log(DEBUG_LEVEL,
                                           "RemoteTimerBackend::%s",
                                           __func__);

      if(earliestExpireTime_ >= Clock::now())
        {
          return earliestExpireTime_;
        }

      return {};
    }

  private:
    int iFd_;
    TimePoint earliestExpireTime_;
  };
}

#endif // EMANEREMOTETIMERBACKEND_HEADER_
