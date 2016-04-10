/*
 * Copyright (c) 2013-2014,2016 - Adjacent Link LLC, Bridgewater,
 * New Jersey
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

#include "timerserviceproxy.h"
#include "timerservice.h"
#include "logservice.h"

EMANE::TimerServiceProxy::TimerServiceProxy():
  pTimerServiceUser_{}{}

EMANE::TimerServiceProxy::~TimerServiceProxy()
{}

void EMANE::TimerServiceProxy::setTimerServiceUser(TimerServiceUser * pTimerServiceUser)
{
  pTimerServiceUser_ = pTimerServiceUser;
}

bool EMANE::TimerServiceProxy::cancelTimedEvent(TimerEventId eventId)
{
  return TimerServiceSingleton::instance()->cancelTimedEvent(eventId);
}

EMANE::TimerEventId EMANE::TimerServiceProxy::scheduleTimedEvent(const TimePoint & timeout,
                                                                 const void *arg,
                                                                 const Duration & interval)
{
  return TimerServiceSingleton::instance()->scheduleTimedEvent(timeout,
                                                               arg,
                                                               interval,
                                                               this);
}

void EMANE::TimerServiceProxy::processTimedEvent(TimerEventId eventId,
                                                 const TimePoint & expireTime,
                                                 const TimePoint & scheduleTime,
                                                 const TimePoint & fireTime,
                                                 const void * arg)
{
  pTimerServiceUser_->processTimedEvent(eventId,
                                        expireTime,
                                        scheduleTime,
                                        fireTime,
                                        arg);
}

EMANE::TimerEventId EMANE::TimerServiceProxy::schedule_i(TimerCallback,
                                                         const TimePoint &,
                                                         const Duration &)
{
  LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                          ERROR_LEVEL,
                          "TimerService schedule not available to component");

  return 0;
}
