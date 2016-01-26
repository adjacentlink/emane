/*
 * Copyright (c) 2013-2016 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2008-2012 - DRS CenGen, LLC, Columbia, Maryland
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
 * * Neither the name of DRS CenGen, LLC nor the names of its
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

#ifndef EMANEEVENTSERVICE_HEADER_
#define EMANEEVENTSERVICE_HEADER_

#include "emane/types.h"
#include "emane/componenttypes.h"
#include "emane/eventserviceprovider.h"
#include "emane/eventserviceuser.h"
#include "emane/event.h"
#include "emane/inetaddr.h"
#include "emane/serializable.h"
#include "emane/utils/singleton.h"
#include "multicastsocket.h"

#include <map>
#include <tuple>
#include <atomic>
#include <thread>
#include <uuid.h>


namespace EMANE
{
  class EventService : public Utils::Singleton<EventService>
  {
  public:
    ~EventService();

    void open(const INETAddr & eventChannelAddress,
              const std::string & sDevice,
              int iTTL,
              bool loopbackEnable,
              const uuid_t & uuid);

    void close();

    void registerEventServiceUser(BuildId buildId,
                                  EventServiceUser * pEventServiceUser,
                                  NEMId = 0);

    void registerEvent(BuildId buildId,EventId eventId);


    void sendEvent(BuildId buildId,
                   NEMId nemId,
                   const Event & event) const;

    void sendEvent(BuildId buildId,
                   NEMId nemId,
                   EventId eventId,
                   const Serialization & serialization) const;


    void processEventMessage(NEMId nemId,
                             EventId eventId,
                             const Serialization & serialization,
                             NEMId ignoreNEM = {}) const;

  protected:
    EventService();

  private:
    using EventServiceUserMap = std::map<BuildId, std::pair<NEMId,EventServiceUser *>>;

    using EventRegistrationMap = std::multimap<EventId, std::tuple<BuildId,NEMId,EventServiceUser *>>;

    EventRegistrationMap eventRegistrationMap_;

    EventServiceUserMap eventServiceUserMap_;

    MulticastSocket mcast_;
    std::thread thread_;

    bool bOpen_;
    bool bCancel_;
    uuid_t uuid_;

    mutable std::atomic<std::uint64_t> u64SequenceNumber_;

    void process();

  };

  using EventServiceSingleton = EventService;
}

#endif //EMANEEVENTSERVICE_HEADER_
