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

#include "emane/application/eventagentbuilder.h"
#include "emane/buildexception.h"
#include "eventagentfactorymanager.h"
#include "eventagentmanagerimpl.h"
#include "logservice.h"
#include "timerserviceproxy.h"
#include "eventservice.h"
#include "platformservice.h"
#include "buildidservice.h"
#include "registrarproxy.h"

EMANE::Application::EventAgentBuilder::EventAgentBuilder(){}

EMANE::Application::EventAgentBuilder::~EventAgentBuilder(){}

std::unique_ptr<EMANE::Application::EventAgentManager>
EMANE::Application::EventAgentBuilder::buildEventAgentManager(EventAgents & agents,
                                                              const ConfigurationUpdateRequest& request)
{
  if(agents.empty())
    {
      throw BuildException("Trying to build an EventAgentManager without any EventAgents");
    }

  std::unique_ptr<EventAgentManager> pManager{new EventAgentManagerImpl};

  BuildId buildId{BuildIdServiceSingleton::instance()->registerBuildable(pManager.get())};
  
  RegistrarProxy registrarProxy{buildId};

  pManager->initialize(registrarProxy);

  // configure 
  pManager->configure(ConfigurationServiceSingleton::instance()->buildUpdates(buildId,request));

  std::for_each(agents.begin(),
                agents.end(),
                [&pManager](std::unique_ptr<EventAgent> & pAgent)
                {
                  pManager->add(pAgent);
                });

  return pManager;
}


std::unique_ptr<EMANE::EventAgent>
EMANE::Application::EventAgentBuilder::buildEventAgent(EMANE::NEMId nemId,
                                                       const std::string & sLibraryFile,
                                                       const ConfigurationUpdateRequest& request,
                                                       bool bSkipConfigure)
{
  std::string sNativeLibraryFile = ACE_DLL_PREFIX + 
                                   sLibraryFile + 
                                   ACE_DLL_SUFFIX;

  const EMANE::EventAgentFactory & eventAgentFactory = 
    EMANE::EventAgentFactoryManagerSingleton::instance()->getEventAgentFactory(sNativeLibraryFile);
      
  // new platform service 
  EMANE::PlatformService *
    pPlatformService{new EMANE::PlatformService{}};

  // create agent
  std::unique_ptr<EventAgent> pAgent{eventAgentFactory.createEventAgent(nemId, pPlatformService)};


  BuildId buildId{BuildIdServiceSingleton::instance()->registerBuildable(pAgent.get())};

  // pass agent to platform service
  pPlatformService->setPlatformServiceUser(buildId,pAgent.get());

  // register event service user with event service
  EventServiceSingleton::instance()->registerEventServiceUser(buildId,
                                                              pAgent.get(),
                                                              nemId);



  RegistrarProxy registrarProxy{buildId};

  // initialize 
  pAgent->initialize(registrarProxy);

  if(!bSkipConfigure)
    {
      // configure
      pAgent->configure(ConfigurationServiceSingleton::instance()->buildUpdates(buildId,request));
    }

  return pAgent;
}
