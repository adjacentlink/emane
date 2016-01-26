/*
 * Copyright (c) 2013-2015 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2011-2012 - DRS CenGen, LLC, Columbia, Maryland
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
#include "eventgeneratormanagerimpl.h"
#include "logservice.h"
#include "eventservice.h"
#include "timerservice.h"
#include "eventserviceexception.h"

#include "emane/configureexception.h"
#include "emane/startexception.h"

EMANE::Application::EventGeneratorManagerImpl::EventGeneratorManagerImpl(const uuid_t & uuid):
  EventGeneratorManager{uuid}{}

EMANE::Application::EventGeneratorManagerImpl::~EventGeneratorManagerImpl(){}

void EMANE::Application::EventGeneratorManagerImpl::add(std::unique_ptr<EventGenerator> & pGenerator)
{
  eventGenerators_.push_back(std::move(pGenerator));
}

void EMANE::Application::EventGeneratorManagerImpl::initialize(Registrar & registrar)
{
  auto & configRegistrar = registrar.configurationRegistrar();

  configRegistrar.registerNonNumeric<INETAddr>("eventservicegroup",
                                               ConfigurationProperties::REQUIRED,
                                               {},
                                               "IPv4 or IPv6 Event Service channel multicast endpoint.");

  configRegistrar.registerNonNumeric<std::string>("eventservicedevice",
                                                  ConfigurationProperties::NONE,
                                                  {},
                                                  "Device to associate with the Event Service channel multicast endpoint.");

  configRegistrar.registerNumeric<std::uint8_t>("eventservicettl",
                                                ConfigurationProperties::DEFAULT,
                                                {1},
                                                "Device to associate with the Event Service channel multicast endpoint.");
}


void EMANE::Application::EventGeneratorManagerImpl::configure(const ConfigurationUpdate & update)
{
  for(const auto & item : update)
    {
      if(item.first == "eventservicegroup")
        {
          eventServiceGroupAddr_ = item.second[0].asINETAddr();

          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  INFO_LEVEL,
                                  "EventGeneratorManagerImpl::configure %s: %s",
                                  item.first.c_str(),
                                  eventServiceGroupAddr_.str().c_str());
        }
      else if(item.first == "eventservicedevice")
        {
          sEventServiceDevice_ = item.second[0].asString();

          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  INFO_LEVEL,
                                  "EventGeneratorManagerImpl::configure %s: %s",
                                  item.first.c_str(),
                                  sEventServiceDevice_.c_str());

        }
      else if(item.first == "eventservicettl")
        {
          u8EventServiceTTL_ = item.second[0].asUINT8();

          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  INFO_LEVEL,
                                  "EventGeneratorManagerImpl::configure %s: %hhu",
                                  item.first.c_str(),
                                  u8EventServiceTTL_);
        }
      else
        {
          throw makeException<ConfigureException>("EventGeneratorManagerImpl: "
                                                  "Unexpected configuration item %s",
                                                  item.first.c_str());
        }
    }

}

void EMANE::Application::EventGeneratorManagerImpl::start()
{
  try
    {
      EventServiceSingleton::instance()->open(eventServiceGroupAddr_,
                                              sEventServiceDevice_,
                                              u8EventServiceTTL_,
                                              true,
                                              uuid_);
    }
  catch(EventServiceException & e)
    {
      throw StartException(e.what());
    }

  std::for_each(eventGenerators_.begin(),
                eventGenerators_.end(),
                std::bind(&Component::start,std::placeholders::_1));
}

void EMANE::Application::EventGeneratorManagerImpl::postStart()
{
  std::for_each(eventGenerators_.begin(),
                eventGenerators_.end(),
                std::bind(&Component::postStart,std::placeholders::_1));
}

void EMANE::Application::EventGeneratorManagerImpl::stop()
{
  std::for_each(eventGenerators_.begin(),
                eventGenerators_.end(),
                std::bind(&Component::stop,std::placeholders::_1));
}

void EMANE::Application::EventGeneratorManagerImpl::destroy()
  throw()
{
  std::for_each(eventGenerators_.begin(),
                eventGenerators_.end(),
                std::bind(&Component::destroy,std::placeholders::_1));
}
