/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2009 - DRS CenGen, LLC, Columbia, Maryland
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

#include "eventagentdirector.h"

#include <sstream>
#include <iostream>
/*
 * Constructor
 *
 * @param filename reference to the base XML filename
 * @param builder reference to the EventAgentBuilder
 */
EMANE::Application::EventAgentDirector::EventAgentDirector(const std::string & filename, 
                                                           EventAgentBuilder & builder):
  eventDaemonConfig_(filename),
  builder_(builder)
{}
  
/*
 * Destructor
 */
EMANE::Application::EventAgentDirector::~EventAgentDirector()
{}

/*
 * Constructs the event agent and generators
 *
 */
std::unique_ptr<EMANE::Application::EventAgentManager>
EMANE::Application::EventAgentDirector::construct()
{
  /* Get NEM Id */
  NEMId nemId = eventDaemonConfig_.getNEMId();

  /* Now go through each event generator configuration and build appropriately */
  EventAgents agents;
  
  for(const auto & pLayerConfig : eventDaemonConfig_.getAgents())
    {
      agents.push_back(builder_.buildEventAgent(nemId,
                                                pLayerConfig->getLibrary(),
                                                pLayerConfig->getConfigurationUpdateRequest()));
    }
  
  /* Build Event Agent Manager */
  return builder_.buildEventAgentManager(agents,
                                         eventDaemonConfig_.getConfigurationUpdateRequest());
}

