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

#include "emane/application/transportbuilder.h"
#include "emane/buildexception.h"
#include "transportmanagerimpl.h"
#include "transportadapterimpl.h"
#include "transportfactorymanager.h"
#include "logservice.h"
#include "timerserviceproxy.h"
#include "eventservice.h"
#include "platformservice.h"
#include "buildidservice.h"
#include "registrarproxy.h"

EMANE::Application::TransportBuilder::TransportBuilder(){}

EMANE::Application::TransportBuilder::~TransportBuilder(){}

std::unique_ptr<EMANE::Application::TransportManager>
EMANE::Application::TransportBuilder::buildTransportManager(const uuid_t & uuid,
                                                            TransportAdapters & adapters,
                                                            const ConfigurationUpdateRequest& request)
{
  if(adapters.empty())
    {
      throw BuildException("Trying to build a TransportManager without any TransportAdapters");
    }

  std::unique_ptr<TransportManager> pManager{new TransportManagerImpl{uuid}};

  BuildId buildId{BuildIdServiceSingleton::instance()->registerBuildable(pManager.get())};

  RegistrarProxy registrarProxy{buildId};

  pManager->initialize(registrarProxy);
  
  // configure 
  pManager->configure(ConfigurationServiceSingleton::instance()->buildUpdates(buildId,request));

  std::for_each(adapters.begin(),
                adapters.end(),
                [&pManager](std::unique_ptr<TransportAdapter> & pAdapter)
                {
                  pManager->add(pAdapter);
                });

  return pManager;
}


std::unique_ptr<EMANE::Application::TransportAdapter>
EMANE::Application::TransportBuilder::buildTransportAdapter(std::unique_ptr<Transport> & pTransport,
                                                            const ConfigurationUpdateRequest& request)
{
  if(pTransport == NULL)
    {
      throw BuildException("Trying to build a TransportAdapter without a Transport");
    }

  std::unique_ptr<TransportAdapter> pAdapter{new TransportAdapterImpl{pTransport->getNEMId()}};

  BuildId buildId{BuildIdServiceSingleton::instance()->registerBuildable(pAdapter.get())};
  
  RegistrarProxy registrarProxy{buildId};
  
  pAdapter->initialize(registrarProxy);

  // configure 
  pAdapter->configure(ConfigurationServiceSingleton::instance()->buildUpdates(buildId,request));
  
  pAdapter->setTransport(pTransport);
  
  return pAdapter;
}


std::unique_ptr<EMANE::Transport>
EMANE::Application::TransportBuilder::buildTransport(NEMId id,
                                                     const std::string & sLibraryFile,
                                                     const ConfigurationUpdateRequest & request,
                                                     bool bSkipConfigure)
{
  std::string sNativeLibraryFile = ACE_DLL_PREFIX + 
                                   sLibraryFile + 
                                   ACE_DLL_SUFFIX;

  const EMANE::TransportFactory & transportFactory = 
    EMANE::TransportFactoryManagerSingleton::instance()->getTransportFactory(sNativeLibraryFile);
      
  // new platform service
  EMANE::PlatformService *
    pPlatformService{new EMANE::PlatformService{}};

  // create transport
  std::unique_ptr<Transport> pTransport{transportFactory.createTransport(id, pPlatformService)};

  initializeTransport(pTransport.get(), pPlatformService, request,bSkipConfigure);

  return pTransport;
}


EMANE::PlatformServiceProvider * 
EMANE::Application::TransportBuilder::newPlatformService() const
{
  return new EMANE::PlatformService{};
}


void 
EMANE::Application::TransportBuilder::initializeTransport(Transport * pTransport, 
                                                          PlatformServiceProvider * pProvider,
                                                          const ConfigurationUpdateRequest & request,
                                                          bool bSkipConfigure) const
{
  BuildId buildId{BuildIdServiceSingleton::instance()->registerBuildable(pTransport)};

  // pass transport to platform service
  dynamic_cast<EMANE::PlatformService*>(pProvider)->setPlatformServiceUser(buildId,pTransport);

  RegistrarProxy registrarProxy{buildId};
      
  // initialize
  pTransport->initialize(registrarProxy);

  if(!bSkipConfigure)
    {
      // configure 
      pTransport->configure(ConfigurationServiceSingleton::instance()->buildUpdates(buildId,request));
    }
}
