/*
 * Copyright (c) 2013,2015 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANEEVENTAGENTFACTORY_HEADER_
#define EMANEEVENTAGENTFACTORY_HEADER_

#include "emane/eventagent.h"

#include "emane/utils/factoryexception.h"

namespace EMANE
{
  /**
   * @class EventAgentFactory
   *
   * @brief Factory for creating EventAgents.  The factory
   * manages the DLL allowing for the creation of multiple agents
   *
   * @note DLLs must stay open in order to use their contents
   */
  class EventAgentFactory
  {
  public:
    /**
     * Constructor
     *
     * @param sLibraryName Filename of DLL
     *
     * @throw Utils::FactoryException
     */
    EventAgentFactory(const std::string & sLibraryName);

    
    ~EventAgentFactory();
    
    /**
     * Create an EventAgent
     *
     * @param nemId Id of the NEM this agent is part of
     * @param pPlatformService pointer tothe PlatformServiceProvider
     *
     * @returns EventAgent reference
     */
    EventAgent * createEventAgent(NEMId nemId,
                                  PlatformServiceProvider * pPlatformService) const;

    /**
     * Destory an EventAgent
     *
     * @param pAgent Reference to EventAgent 
     */
    void destoryEventAgent(EventAgent * pAgent) const;
    
  private:
    using CreateEventAgentFunc =  EventAgent * (*)(NEMId nemId,
                                                   PlatformServiceProvider * pPlatformService); 
    using DestroyEventAgentFunc = void (*)(EventAgent*); 

    void * pLibHandle_;
    CreateEventAgentFunc createEventAgentFunc_;
    DestroyEventAgentFunc destroyEventAgentFunc_;
  };
}

#endif // EMANEEVENTAGENTFACTORY_HEADER_

