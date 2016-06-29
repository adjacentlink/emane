/*
 * Copyright (c) 2016 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "nemtimerserviceproxy.h"
#include "timerservice.h"

EMANE::NEMTimerServiceProxy::NEMTimerServiceProxy():
  pNEMQueuedLayer_{}{}

EMANE::NEMTimerServiceProxy::~NEMTimerServiceProxy()
{}

void EMANE::NEMTimerServiceProxy::setNEMLayer(NEMQueuedLayer * pNEMQueuedLayer)
{
  pNEMQueuedLayer_ = pNEMQueuedLayer;
}

bool EMANE::NEMTimerServiceProxy::cancelTimedEvent(TimerEventId eventId)
{
  return TimerServiceSingleton::instance()->cancelTimedEvent(eventId);
}

EMANE::TimerEventId EMANE::NEMTimerServiceProxy::scheduleTimedEvent(const TimePoint & timeout,
                                                                    const void *arg,
                                                                    const Microseconds & interval)
{
  return TimerServiceSingleton::instance()->scheduleTimedEvent(timeout,
                                                               arg,
                                                               interval,
                                                               this);
}

void EMANE::NEMTimerServiceProxy::processTimedEvent(TimerEventId eventId,
                                                    const TimePoint & expireTime,
                                                    const TimePoint & scheduleTime,
                                                    const TimePoint & fireTime,
                                                    const void * arg)
{
  pNEMQueuedLayer_->processTimedEvent(eventId,
                                      expireTime,
                                      scheduleTime,
                                      fireTime,
                                      arg);
}

EMANE::TimerEventId EMANE::NEMTimerServiceProxy::schedule_i(TimerCallback callback,
                                                            const TimePoint & timePoint,
                                                            const Microseconds & interval)
{
  return TimerServiceSingleton::instance()->schedule([this,callback](const TimePoint & expireTime,
                                                                     const TimePoint & scheduleTime,
                                                                     const TimePoint & fireTime)
                                                     {
                                                       pNEMQueuedLayer_->processTimer(callback,
                                                                                      expireTime,
                                                                                      scheduleTime,
                                                                                      fireTime);
                                                     },
                                                     timePoint,
                                                     interval);
}
