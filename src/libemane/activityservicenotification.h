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

#ifndef EMANEACTIVITYSERVICENOTIFICATION_HEADER_
#define EMANEACTIVITYSERVICENOTIFICATION_HEADER_

#include "emane/types.h"
#include "emane/activityserviceprovider.h"

#include <uuid.h>

namespace EMANE
{
  class ActivityServiceNotification : public ActivityServiceProvider
  {
  public:
    virtual ~ActivityServiceNotification(){}

    virtual
    void notifyWorkEnqueue() = 0;

    virtual
    void notifyWorkDequeue() = 0;

    virtual
    void notifyOTA(const uuid_t & uuid,
                   std::uint64_t u64SequenceNumber,
                   NEMId source,
                   NEMId destination) = 0;

    virtual
    void notifyEvent(const uuid_t & uuid,
                     std::uint64_t u64SequenceNumber) = 0;

    virtual
    void notifyBoundary(NEMId source,
                        NEMId destination,
                        NEMId local,
                        std::uint64_t u64Sequence) = 0;

    virtual
    void notifyNOP() = 0;

  protected:
    ActivityServiceNotification() = default;
  };
}

#endif // EMANEACTIVITYSERVICENOTIFICATION_HEADER_
