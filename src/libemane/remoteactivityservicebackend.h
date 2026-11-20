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

#ifndef EMANEREMOTEACTIVITYSERVICEBACKEND_HEADER_
#define EMANEREMOTEACTIVITYSERVICEBACKEND_HEADER_

#include "activityservicebackend.h"

#include "emane/inetaddr.h"
#include "emane/clock.h"
#include "emane/utils/raiizmq.h"

#include <thread>
#include <mutex>
#include <uuid.h>

namespace SimActivityService
{
  class Activity;
}

namespace EMANE
{
  class RemoteActivityServiceBackend : public ActivityServiceBackend
  {
  public:
    RemoteActivityServiceBackend(const uuid_t & uuid,
                                 const INETAddr & simulationCoonrdinatorEndpoint,
                                 const INETAddr & activityServiceEndpoint,
                                 const std::string & sActivityServiceTag);

    ~RemoteActivityServiceBackend();

    void notifyWorkEnqueue() override;

    void notifyWorkDequeue() override;

    void notifyOTA(const uuid_t & uuid,
                   std::uint64_t u64SequenceNumber,
                   NEMId source,
                   NEMId destination) override;

    void notifyEvent(const uuid_t & uuid,
                     std::uint64_t u64SequenceNumber) override;

    void notifyBoundary(NEMId source,
                        NEMId destination,
                        NEMId local,
                        std::uint64_t u64Sequence) override;

    void notifyNOP() override;

    void notifyAction(const std::string & sName,
                      const std::string & sIdentifier) override;

    Clock::time_point now() override;

    void waitForStart() override;

    void cancelWaitForStart() override;

    bool isStarted() override;

  private:
    uuid_t uuid_;
    std::string sTag_;
    std::unique_ptr<Utils::TimerBackend> pTimerBackend_;
    Utils::RAIIZMQContext pContext_;
    Utils::RAIIZMQSocket pPubSocket_;
    Utils::RAIIZMQSocket pSubSocket_;
    std::thread thread_;
    std::recursive_mutex mutex_;
    std::mutex mutexSync_;
    int iEventFd_;
    Clock::time_point simulationTime_;
    bool bStarted_;
    std::uint16_t u16WorkQueueDepth_;
    std::uint64_t u64SequenceNumber_;
    std::uint64_t u64ActionsSinceLastStatus_;
    TimePoint lastStatusTime_;
    bool bNotFirstSignal_;

    void process();

    void sendHello();

    void send(const std::string & sTopic,
              SimActivityService::Activity & msg);
  };
}

#endif // EMANEACTIVITYSERVICEREMOTEBACKEND_HEADER_
