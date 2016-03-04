/*
 * Copyright (c) 2013-2014,2016 - Adjacent Link LLC, Bridgewater, New
 * Jersey
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

#include "emane/application/eventgeneratorbuilder.h"
#include "emane/buildexception.h"
#include "eventgeneratorfactorymanager.h"
#include "eventgeneratormanagerimpl.h"
#include "logservice.h"
#include "eventservice.h"
#include "platformservice.h"
#include "timerserviceproxy.h"
#include "buildidservice.h"
#include "registrarproxy.h"



EMANE::Application::EventGeneratorBuilder::EventGeneratorBuilder(){}

EMANE::Application::EventGeneratorBuilder::~EventGeneratorBuilder(){}

std::unique_ptr<EMANE::Application::EventGeneratorManager>
EMANE::Application::EventGeneratorBuilder::buildEventGeneratorManager(const uuid_t & uuid,
                                                                      EventGenerators & generators,
                                                                      const ConfigurationUpdateRequest & request)
{
  if(generators.empty())
    {
      throw BuildException("Trying to build an EventGeneratorManager without any EventGenerators");
    }

  std::unique_ptr<EventGeneratorManager> pManager{new EventGeneratorManagerImpl{uuid}};

  BuildId buildId{BuildIdServiceSingleton::instance()->registerBuildable(pManager.get())};

  RegistrarProxy registrarProxy{buildId};

  pManager->initialize(registrarProxy);

  pManager->configure(ConfigurationServiceSingleton::instance()->buildUpdates(buildId,request));

  std::for_each(generators.begin(),
                generators.end(),
                [&pManager](std::unique_ptr<EventGenerator> & pGenerator)
                {
                  pManager->add(pGenerator);
                });

  return pManager;
}


std::unique_ptr<EMANE::EventGenerator>
EMANE::Application::EventGeneratorBuilder::buildEventGenerator(const std::string & sLibraryFile,
                                                               const ConfigurationUpdateRequest& request,
                                                               bool bSkipConfigure)
{
   std::string sNativeLibraryFile = "lib" +
                                    sLibraryFile +
                                    ".so";

  const EMANE::EventGeneratorFactory & eventGeneratorFactory =
    EMANE::EventGeneratorFactoryManagerSingleton::instance()->getEventGeneratorFactory(sNativeLibraryFile);

  // new platform service
  EMANE::PlatformService *
    pPlatformService{new EMANE::PlatformService{}};

  // create generator
  std::unique_ptr<EMANE::EventGenerator>
    pGenerator{eventGeneratorFactory.createEventGenerator(pPlatformService)};

  BuildId buildId{BuildIdServiceSingleton::instance()->registerBuildable(pGenerator.get())};

  // pass generator to platform service
  pPlatformService->setPlatformServiceUser(buildId,pGenerator.get());

  // register event service user with event service
  // event generators will get any event they register for
  // regardless of target nem
  EventServiceSingleton::instance()->registerEventServiceUser(buildId,
                                                              pGenerator.get(),
                                                              0);

  RegistrarProxy registrarProxy{buildId};

  // initialize
  pGenerator->initialize(registrarProxy);

  if(!bSkipConfigure)
    {
      // configure
      pGenerator->configure(ConfigurationServiceSingleton::instance()->buildUpdates(buildId,request));
    }

  return pGenerator;
}
