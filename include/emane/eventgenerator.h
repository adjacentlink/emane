/*
 * Copyright (c) 2014 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2008-2011 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANEEVENTGENERATOR_HEADER_
#define EMANEEVENTGENERATOR_HEADER_

#include "emane/component.h"
#include "emane/componenttypes.h"
#include "emane/platformserviceuser.h"
#include "emane/buildable.h"

#include <string>
#include <list>

namespace EMANE
{
  /**
   * @class EventGenerator
   *
   * @brief Base class for all event generators
   */
  class EventGenerator : public Component,
                         public PlatformServiceUser,
                         public Buildable
  {
  public:
    /**
     * Destroys an instance
     */
    virtual ~EventGenerator(){}
    
  protected:
    EventGenerator(EMANE::PlatformServiceProvider *pPlatformService) : 
      PlatformServiceUser(pPlatformService)
    {}
  };
}

#define DECLARE_EVENT_GENERATOR(X)                                      \
  extern "C"  EMANE::EventGenerator * create(EMANE::PlatformServiceProvider *p) \
  {return new X(p);}                                                    \
  extern "C"  void destroy(EMANE::EventGenerator * p)                   \
  {delete p;}                            

#endif //EMANEEVENTGENERATOR_HEADER_

/**
 * @page EventGeneratorPlugin Event Generator Plugin
 *
 * Below are a list of steps necessary to create a loadable %Event Generator plugin. Order is not
 * important and each step is not as simple as it sounds. This is merely a plugin API checklist.
 * -# Derive your  %Event Generator from @ref EMANE::EventGenerator "EventGenerator"
 * -# Override all of the Component state transition methods:
 *   - @ref EMANE::Component::initialize "initialize"
 *     - Register plugin configuration items and an optional configuration validator
 *     - Register plugin statistics and statistic tables
 *     - Register plugin events of interest
 *   - @ref EMANE::Component::configure "configure"
 *     - Process all loaded configuration
 *   - @ref EMANE::Component::start "start"
 *     - Emulation starts at the conclusion of this method. Do any startup logic.
 *   - @ref EMANE::Component::postStart "postStart"
 *     - All components in the NEM layer stack are now in the @a start state. Do any cross layer startup handshaking.
 *   - @ref EMANE::Component::stop "stop"
 *     - Opposite of start. Do any tear down logic.
 *   - @ref EMANE::Component::destroy "destroy"
 *     - Opposite of initialize. Do any cleanup logic.
 *
 * -# If your plugin will be processing events you will need to override:
 *   - @ref EMANE::EventServiceUser::processEvent "processEvent"
 *
 * -# If your plugin will be scheduling timed events you will need to override:
 *   - @ref EMANE::TimerServiceUser::processTimedEvent "processTimedEvent"
 *
 * -# Add the appropriate functionality to generate events
 * -# Declare your plugin using #DECLARE_EVENT_GENERATOR
 */
