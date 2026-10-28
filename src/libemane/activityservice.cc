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

#include "activityservice.h"
#include "timerservice.h"
#include "logservice.h"
#include "localactivityservicebackend.h"
#include "remoteactivityservicebackend.h"

EMANE::ActivityService::ActivityService():
  pBackend_{}{}

EMANE::ActivityService::~ActivityService()
{}

EMANE::Clock::time_point EMANE::ActivityService::now()
{
  // there are some start up errors that can happen as part of setting
  // up simulaiton mode. This check allows now() to return something
  // valid (epoch time) w/o a valid backend.
  if(pBackend_)
    {
      return pBackend_->now();
    }
  else
    {
      return {};
    }
}


void EMANE::ActivityService::openLocal()
{
  pBackend_.reset(new LocalActivityServiceBackend{});
}

void EMANE::ActivityService::openRemote(const uuid_t & uuid,
                                        const INETAddr & simulationCoonrdinatorEndpoint,
                                        const INETAddr & activityServiceEndpoint,
                                        const std::string & sActivityServiceTag)
{
  pBackend_.reset(new RemoteActivityServiceBackend{uuid,
                                                   simulationCoonrdinatorEndpoint,
                                                   activityServiceEndpoint,
                                                   sActivityServiceTag});
}

void
EMANE::ActivityService::waitForStart()
{
  LogServiceSingleton::instance()->log(DEBUG_LEVEL,
                                       "ActivityService::%s 0",
                                       __func__);

  pBackend_->waitForStart();

  LogServiceSingleton::instance()->log(DEBUG_LEVEL,
                                       "ActivityService::%s 1",
                                       __func__);
}


void
EMANE::ActivityService::cancelWaitForStart()
{
  LogServiceSingleton::instance()->log(DEBUG_LEVEL,
                                       "ActivityService::%s 0",
                                       __func__);

  pBackend_->cancelWaitForStart();

  LogServiceSingleton::instance()->log(DEBUG_LEVEL,
                                       "ActivityService::%s 1",
                                       __func__);
}

bool
EMANE::ActivityService::isStarted()
{
  return pBackend_->isStarted();
}

void EMANE::ActivityService::notifyWorkEnqueue()
{
  pBackend_->notifyWorkEnqueue();
}

void EMANE::ActivityService::notifyWorkDequeue()
{
  pBackend_->notifyWorkDequeue();
}

void
EMANE::ActivityService::notifyOTA(const uuid_t & uuid,
                                  std::uint64_t u64SequenceNumber,
                                  NEMId source,
                                  NEMId destination)
{
  pBackend_->notifyOTA(uuid,
                       u64SequenceNumber,
                       source,
                       destination);
}

void
EMANE::ActivityService::notifyEvent(const uuid_t & uuid,
                                    std::uint64_t u64SequenceNumber)
{
  pBackend_->notifyEvent(uuid,
                         u64SequenceNumber);
}

void
EMANE::ActivityService::notifyBoundary(NEMId source,
                                       NEMId destination,
                                       NEMId local,
                                       std::uint64_t u64Sequence)
{
  pBackend_->notifyBoundary(source,destination,local,u64Sequence);
}

void
EMANE::ActivityService::notifyNOP()
{
  pBackend_->notifyNOP();
}

void EMANE::ActivityService::notifyAction(const std::string & sName,
                                          const std::string & sIdentifier)
{
  pBackend_->notifyAction(sName,sIdentifier);
}
