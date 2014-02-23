/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "emane/utils/spawnmemberfunc.h"
#include "emane/utils/recvcancelable.h"

#include <sstream>

EMANE::EventService::EventService():
  mcast_(ACE_SOCK_Dgram_Mcast::OPT_BINDADDR_NO),
  thread_{0},
  bOpen_{false},
  u64SequenceNumber_{}
{
  uuid_clear(uuid_);
}


EMANE::EventService::~EventService()
{
  if(thread_)
    {
      ACE_OS::thr_cancel(thread_);
      
      ACE_OS::thr_join(thread_,0,0);
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

void EMANE::EventService::open(const ACE_INET_Addr & eventChannelAddress,
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

      const ACE_TCHAR * device{sDevice.empty() ? nullptr : sDevice.c_str()};
      
      if(mcast_.open(eventChannelAddress, device) == -1)
        {
          std::stringstream sstream;
          sstream
            <<"Platform Event Service: Unable to open Event Service socket: '"
            <<eventChannelAddress.get_host_addr()
            <<":"
            <<eventChannelAddress.get_port_number()
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
            <<std::ends;
      
          throw EventServiceException(sstream.str());
        }
  
      if(mcast_.join(eventChannelAddress,1,device) == -1)
        {
          std::stringstream sstream;
          sstream
            <<"Platform OTA Manager: Unable to join Event Service group: '"
            <<eventChannelAddress.get_host_addr()
            <<":"
            <<eventChannelAddress.get_port_number()
            <<"'."
            <<std::endl
            <<std::endl
            <<"Possible reason(s):"
            <<std::endl
            <<" * "
            <<eventChannelAddress.get_host_addr()
            <<" is not a multicast address."
            <<std::endl
            <<std::ends;
      
          throw EventServiceException(sstream.str());
        }

      if(loopbackEnable == false)
        {
          if(eventChannelAddress.get_type() == AF_INET)
            {
              if(mcast_.set_option(IP_MULTICAST_LOOP,0) == -1)
                {
                  std::stringstream ssDescription;
                  ssDescription
                    <<"unable to unset EventService group IP_MULTICAST_LOOP"
                    <<std::ends;
                  throw EventServiceException(ssDescription.str()); 
                }
            }
          else if(eventChannelAddress.get_type() == AF_INET6)
            {
              int loop = 0;
              if(mcast_.ACE_SOCK::set_option(IPPROTO_IPV6,IPV6_MULTICAST_LOOP,
                                             &loop,
                                             sizeof(loop)) == -1)
                {
                  std::stringstream ssDescription;
                  ssDescription
                    <<"unable to unset EventService group IPV6_MULTICAST_LOOP"
                    <<std::ends;
                  throw EventServiceException(ssDescription.str()); 
                }
            }
        }
      
      if(eventChannelAddress.get_type() == AF_INET)
        {
          if(mcast_.set_option(IP_MULTICAST_TTL,iTTL) == -1)
            {
              std::stringstream ssDescription;
              ssDescription
                <<"unable to set EventService TTL IP_MULTICAST_TTL"
                <<std::ends;
              throw EventServiceException(ssDescription.str()); 
            }
        }
      else if(eventChannelAddress.get_type() == AF_INET6)
        {
          if(mcast_.ACE_SOCK::set_option(IPPROTO_IPV6,IPV6_MULTICAST_HOPS,
                                         &iTTL,
                                         sizeof(iTTL)) == -1)
            {
              std::stringstream ssDescription;
              ssDescription
                <<"unable to set EventService TTL IPV6_MULTICAST_TTL"
                <<std::ends;
              throw EventServiceException(ssDescription.str()); 
            }
        }

      ACE_hthread_t threadHandle{};
      
      Utils::spawn(*this,&EventService::processEventMessage,&thread_,&threadHandle);

      int priority{};
      int policy{};

      ACE_OS::thr_getprio(threadHandle,priority,policy);
      
      if(policy == ACE_SCHED_RR)
        { 
          int retval = ACE_OS::thr_setprio(threadHandle,priority + 1,ACE_SCHED_RR);
          
          if(retval != 0)
            {
              LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                      ERROR_LEVEL,
                                      "EventService::open: Unable to set Real Time Priority");
            }
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

              if(mcast_.send(sSerialization.c_str(),sSerialization.size()) == -1)
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



ACE_THR_FUNC_RETURN  EMANE::EventService::processEventMessage()
{
  std::uint8_t buf[65536];
  ssize_t len = 0;

  LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                          DEBUG_LEVEL,
                          "EventService::processEventMessage");
  
  while(1)
    {
      if((len = Utils::recvCancelable(mcast_,buf,sizeof(buf),addr_)) > 0)
        {
          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  DEBUG_LEVEL,
                                  "EventService packet received len: %zd",
                                  len);
          


          EMANEMessage::Event msg;
  
          try
            {
              if(msg.ParseFromArray(buf,len))
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

  return 0;
}



