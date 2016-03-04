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

#include "eventservice.h"
#include "emane/registrarexception.h"
#include "eventserviceexception.h"
#include "event.pb.h"
#include "logservice.h"
#include "eventservice.h"
#include "socketexception.h"

#include "emane/utils/vectorio.h"
#include "emane/utils/threadutils.h"
#include "emane/net.h"

#include <sstream>

EMANE::EventService::EventService():
  bOpen_{false},
  u64SequenceNumber_{}
{
  uuid_clear(uuid_);
}


EMANE::EventService::~EventService()
{
  if(thread_.joinable())
    {
      ThreadUtils::cancel(thread_);

      thread_.join();
    }
}

void EMANE::EventService::registerEvent(BuildId buildId, EventId eventId)

{
  auto iter = eventServiceUserMap_.find(buildId);

  if(iter != eventServiceUserMap_.end())
    {
      eventRegistrationMap_.insert(std::make_pair(eventId,
                                                  std::make_tuple(buildId,
                                                                  iter->second.first,
                                                                  iter->second.second)));
    }
  else
    {
      throw RegistrarException{"Component not eligible to register for events"};
    }
}

void EMANE::EventService::registerEventServiceUser(BuildId buildId,
                                                   EventServiceUser * pEventServiceUser,
                                                   NEMId nemId)
{
  eventServiceUserMap_.insert(std::make_pair(buildId,std::make_pair(nemId,pEventServiceUser)));
}

void EMANE::EventService::open(const INETAddr & eventChannelAddress,
                               const std::string & sDevice,
                               int iTTL,
                               bool loopbackEnable,
                               const uuid_t & uuid)
{
  if(bOpen_)
    {
      throw EventServiceException("EventService already open");
    }
  else
    {
      uuid_copy(uuid_,uuid);

      bOpen_ = true;

      const char * device{sDevice.empty() ? nullptr : sDevice.c_str()};

      try
        {
          mcast_.open(eventChannelAddress,true,device,iTTL,loopbackEnable);
        }
      catch(SocketException & exp)
        {
          std::stringstream sstream;
          sstream
            <<"Platform Event Service: Unable to open Event Service socket: '"
            <<eventChannelAddress.str()
            <<"'."
            <<std::endl
            <<std::endl
            <<"Possible reason(s):"
            <<std::endl
            <<" * No Multicast device specified and routing table non deterministic"
            <<std::endl
            <<"   (no multicast route and no default route)."
            <<std::endl
            <<" * Multicast device "
            <<sDevice
            <<" does not exist or is not up."
            <<std::endl
            <<exp.what()
            <<std::endl
            <<std::ends;

          throw EventServiceException(sstream.str());
        }

      thread_ = std::thread(&EventService::process,this);

      if(ThreadUtils::elevate(thread_))
        {
          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  ERROR_LEVEL,
                                  "EventService::open: Unable to set Real Time Priority");
        }
    }
}


void  EMANE::EventService::sendEvent(BuildId buildId,
                                     NEMId nemId,
                                     const Event & event) const
{
  sendEvent(buildId,nemId,event.getEventId(),event.serialize());
}


void EMANE::EventService::sendEvent(BuildId buildId,
                                    NEMId nemId,
                                    EventId eventId,
                                    const Serialization & serialization) const
{

  // determine if there are any locally registered users for this event
  const auto ret = eventRegistrationMap_.equal_range(eventId);

  // for each local event service user registered for this event
  // determine based on the nemId target whether they should
  // receive the event. The source (base on buildId) will never
  // receive an event it generated
  for(EventRegistrationMap::const_iterator iter = ret.first;
      iter != ret.second;
      ++iter)
    {
      BuildId registeredBuildId{};
      NEMId registeredNEMId{};
      EventServiceUser * pEventServiceUser{};

      std::tie(registeredBuildId,
               registeredNEMId,
               pEventServiceUser) = iter->second;

      if(!buildId || registeredBuildId != buildId)
        {
          if(!nemId || registeredNEMId == nemId)
            {
              pEventServiceUser->processEvent(eventId,serialization);
            }
        }
      else
        {
          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  DEBUG_LEVEL,
                                  "EventService sendEvent skipping originator"
                                  " event id:%hu for NEM:%hu buildId: %hu",
                                  eventId,
                                  registeredNEMId,
                                  buildId);
        }
    }

  // send the event out via the multicast channel
  if(bOpen_)
    {
      EMANEMessage::Event msg;

      auto pData = msg.mutable_data();

      auto pSerialization = pData->add_serializations();

      pSerialization->set_nemid(nemId);

      pSerialization->set_eventid(eventId);

      pSerialization->set_data(serialization);

      msg.set_uuid(reinterpret_cast<const char *>(uuid_),sizeof(uuid_));

      msg.set_sequencenumber(++u64SequenceNumber_);

      try
        {
          std::string sSerialization;

          if(!msg.SerializeToString(&sSerialization))
            {
              LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                      ERROR_LEVEL,
                                      "EventService sendEvent "
                                      "unable to send event id:%hu for NEM:%hu\n",
                                      eventId,
                                      nemId);
            }
          else
            {
              LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                      DEBUG_LEVEL,
                                      "Event %03hu EMANE::EventService::sendEvent",
                                      eventId);

              std::uint16_t u16Length = HTONS(sSerialization.size());

              Utils::VectorIO vectorIO{
                {reinterpret_cast<char *>(&u16Length),sizeof(u16Length)},
                  {const_cast<char *>(sSerialization.c_str()),sSerialization.size()}};

              if(mcast_.send(&vectorIO[0],static_cast<int>(vectorIO.size())) == -1)
                {
                  LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                          ERROR_LEVEL,
                                          "EventService sendEvent "
                                          "unable to send event id:%hu for NEM:%hu\n",
                                          eventId,
                                          nemId);
                }
            }
        }
      catch(google::protobuf::FatalException & exp)
        {
          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  ERROR_LEVEL,
                                  "EventService sendEvent unable to send "
                                  "event id:%hu for NEM:%hu\n",
                                  eventId,
                                  nemId);
        }
    }
  else
    {
      LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                              ERROR_LEVEL,
                              "Event %03hu EMANE::EventService::sendEvent, not open, drop",
                              eventId);
    }
}

void EMANE::EventService::processEventMessage(NEMId nemId,
                                              EventId eventId,
                                              const Serialization & serialization,
                                              NEMId ignoreNEM) const
{
  const auto ret = eventRegistrationMap_.equal_range(eventId);

  for(EventRegistrationMap::const_iterator iter = ret.first;
      iter != ret.second;
      ++iter)
    {
      BuildId registeredBuildId{};
      NEMId registeredNEMId{};
      EventServiceUser * pEventServiceUser{};

      std::tie(registeredBuildId,registeredNEMId,pEventServiceUser) = iter->second;

      if(!ignoreNEM || ignoreNEM != registeredNEMId)
        {
          if(!nemId || registeredNEMId == nemId)
            {
              pEventServiceUser->processEvent(eventId,serialization);
            }
        }
      else
        {
          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  DEBUG_LEVEL,
                                  "EventService sendEvent skipping all layers of"
                                  " originating nem event id:%hu NEM:%hu",
                                  eventId,
                                  ignoreNEM);

        }
    }
}



void  EMANE::EventService::process()
{
  std::uint8_t buf[65536];
  ssize_t len = 0;

  LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                          DEBUG_LEVEL,
                          "EventService::processEventMessage");

  while(1)
    {
      if((len = mcast_.recv(buf,sizeof(buf),0)) > 0)
        {
          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  DEBUG_LEVEL,
                                  "EventService packet received len: %zd",
                                  len);

          std::uint16_t * pu16Length{reinterpret_cast<std::uint16_t *>(buf)};

          *pu16Length = NTOHS(*pu16Length);

          len -= sizeof(std::uint16_t);

          EMANEMessage::Event msg;

          try
            {
              if(static_cast<size_t>(len) == *pu16Length &&
                 msg.ParseFromArray(&buf[2], *pu16Length))
                {
                  // only process multicast events that were not sourced locally
                  if(uuid_compare(uuid_,reinterpret_cast<const unsigned char *>(msg.uuid().data())))
                    {
                      using RepeatedPtrFieldSerilaization =
                        google::protobuf::RepeatedPtrField<EMANEMessage::Event::Data::Serialization>;

                      for(const auto & repeatedSerialization :
                            RepeatedPtrFieldSerilaization(msg.data().serializations()))
                        {
                          NEMId nemId{static_cast<NEMId>(repeatedSerialization.nemid())};

                          const auto ret = eventRegistrationMap_.equal_range(static_cast<EventId>(repeatedSerialization.eventid()));

                          for(EventRegistrationMap::const_iterator iter = ret.first;
                              iter != ret.second;
                              ++iter)
                            {
                              BuildId registeredBuildId{};
                              NEMId registeredNEMId{};
                              EventServiceUser * pEventServiceUser{};

                              std::tie(registeredBuildId,registeredNEMId,pEventServiceUser) = iter->second;

                              if(!nemId || !registeredNEMId || registeredNEMId == nemId)
                                {
                                  pEventServiceUser->processEvent(static_cast<EventId>(repeatedSerialization.eventid()),
                                                                  repeatedSerialization.data());
                                }
                            }
                        }
                    }
                }
              else
                {
                  LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                          ERROR_LEVEL,
                                          "EventService unable to deserialize event");
                }
            }
          catch(google::protobuf::FatalException & exp)
            {
              LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                      ERROR_LEVEL,
                                      "EventService unable to deserialize event");
            }
        }
      else
        {
          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  ERROR_LEVEL,
                                  "EventService Packet Receive error");
          break;
        }
    }

}
