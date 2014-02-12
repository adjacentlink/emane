/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANEAPPLICATIONEVENTAGENTBUILDER_HEADER_
#define EMANEAPPLICATIONEVENTAGENTBUILDER_HEADER_

#include "emane/application/eventagentmanager.h"
#include "emane/configurationupdate.h"
 
#include <string>
#include <list> 
#include <memory>

namespace EMANE
{
  namespace Application
  {
    using EventAgents = std::list<std::unique_ptr<EMANE::EventAgent>>;

    /**
     * @class EventAgentBuilder
     *
     * @brief Provides methods for constructing event agents and a
     * manager to contain and control them as a group.
     */
    class EventAgentBuilder
    {
    public:
      /**
       * Creates an instance
       */
      EventAgentBuilder();
    
      /**
       * Destroys an instance
       */
      ~EventAgentBuilder();
    
      /**
       * Builds an EventAgentManager
       *
       * @param agents the list of agents to manage
       * @param request configuration update
       *
       * @return Unique pointer to an initialized and configured EventAgentManager
       *
       * @throw InitializeException when an error occurs during
       * initialization.
       * @throw BuildException when an error occurs building an NEM
       * from the specified layers.
       * @throw ConfigureException when an error occurs during
       * configure.
       */
      std::unique_ptr<EventAgentManager>
      buildEventAgentManager(EventAgents & agents,
                             const ConfigurationUpdateRequest& request);

      /**
       * Builds an EventAgent
       *
       * @param nemId NEM id of the nem for which this event agent is being 
       * created
       * @param sLibraryFile Name of the dll containing the generator
       * @param request configuration update
       * @param bSkipConfigure Flag indicating whether to skip
       * calling Component::configure
       *
       * @return Unique pointer to an initialized and configured EventAgent
       *
       * @throw Utils::FactoryException when a DLL load error occurs.
       * @throw InitializeException when an error occurs during
       * initialization.
       * @throw ConfigureException when an error occurs during
       * configure.
       */
      std::unique_ptr<EventAgent>
      buildEventAgent(EMANE::NEMId nemId,
                      const std::string & sLibraryFile,
                      const ConfigurationUpdateRequest & request,
                      bool bSkipConfigure = false);
    };
  }
}

#endif //EMANEAPPLICATIONEVENTAGENTBUILDER_HEADER_
