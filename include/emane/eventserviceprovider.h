/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2008 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANEEVENTSERVICEPROVIDER_HEADER_
#define EMANEEVENTSERVICEPROVIDER_HEADER_

#include "emane/event.h"
#include "emane/componenttypes.h"

namespace EMANE
{
  
  /**
   * @class EventServiceProvider
   *
   * @brief Event service provider interface
   */
  class EventServiceProvider
  {
  public:
    /**
     * Destroys an instance
     */
    virtual ~EventServiceProvider(){}
    
    /**
     * Send an event
     *
     * @param nemId Id of destination NEM @c 0 for all NEMs in a platform
     * @param event The event object
     */
    virtual void sendEvent(NEMId         nemId, 
                           const Event & event) = 0;

    /**
     * Send an event
     *
     * @param nemId Id of destination NEM @c 0 for all NEMs in a platform
     * @param eventId The event id
     * @param serialization Serialized event object state
     */
    virtual void sendEvent(NEMId         nemId, 
                           EventId       eventId,
                           const Serialization & serialization) = 0;


  protected:
    EventServiceProvider(){}
  };
}

#endif //EMANEEVENTSERVICEPROVIDER_HEADER_

/**
 * @page EventService Event Service
 *
 * The @ref EMANE::EventRegistrar "EventRegistrar" is used by components to register to receive events.
 * Events target a specific NEM Id or the NEM Id 0 to indicate all NEMs. In order for a component to
 * receive an event it must register for the event and the inbound event NEM target must match the
 * component's NEM Id or be addressed to all NEMs.
 *
 * @section RegisteringForAnEvent Registering for an Event
 *
 * Events can only be registered during @ref EMANE::Component::initialize "Component::initialize".
 * The @ref EMANE::EventRegistrar "EventRegistrar" is accessible via the  @ref EMANE::Component::initialize
 * "initialize" method's @ref EMANE::Registrar "Registrar" argument.
 *
 * Components register to receive an event using @ref EMANE::EventRegistrar::registerEvent
 * "EventRegistrar::registerEvent".
 *
 * @snippet src/libemane/frameworkphy.cc eventservice-registerevent-snippet
 *
 * @section HandlingAnEvent  Handling an Event 
 *
 * When a registered event is received for a targeted NEM it is pushed onto the NEM's functor queue
 * as an @ref EMANE::EventServiceUser::processEvent "EventServiceUser::processEvent" method. The event data is
 *  serialized and an event object must be restored (de-serialized) in order to access the event data.
 *
 * @snippet src/libemane/frameworkphy.cc eventservice-processevent-snippet
 *
 * @section SendingAnEvent Sending an Event
 *
 * All components have the ability to send events using @ref EMANE::EventServiceProvider::sendEvent
 * "EventServiceProvider::sendEvent". The @ref EMANE::EventServiceProvider "EventServiceProvider" is accessed
 * via the @ref EMANE::PlatformServiceProvider "PlatformServiceProvider". All components are given a reference
 * to the @ref EMANE::PlatformServiceProvider "PlatformServiceProvider" when they are constructed.
 *
 * @snippet src/libemane/frameworkphy.cc eventservice-sendevent-snippet
 *
 * A component will never receive an event it sends even if it targets its own NEM Id or sends the event to all
 * NEMs.
 */
