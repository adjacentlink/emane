/*
 * Copyright (c) 2013-2014,2016 - Adjacent Link LLC, Bridgewater,
 * New Jersey
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
#include "transportlayer.h"
#include "nemstatefullayer.h"

EMANE::Application::TransportBuilder::TransportBuilder(){}

EMANE::Application::TransportBuilder::~TransportBuilder(){}

std::unique_ptr<EMANE::Application::TransportManager>
EMANE::Application::TransportBuilder::buildTransportManager(const uuid_t & uuid,
                                                            TransportAdapters & adapters,
                                                            const ConfigurationUpdateRequest& request) const
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
EMANE::Application::TransportBuilder::buildTransportAdapter(std::unique_ptr<NEMLayer> & pTransport,
                                                            const ConfigurationUpdateRequest& request) const
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


std::unique_ptr<EMANE::NEMLayer>
EMANE::Application::TransportBuilder::buildTransport(NEMId id,
                                                     const std::string & sLibraryFile,
                                                     const ConfigurationUpdateRequest & request,
                                                     bool bSkipConfigure) const
{
  std::string sNativeLibraryFile = "lib" +
    sLibraryFile +
    ".so";

  const TransportFactory & transportLayerFactory =
    TransportFactoryManagerSingleton::instance()->getTransportFactory(sNativeLibraryFile);

  // new platform service
  PlatformService *
    pPlatformService{new PlatformService{}};

  // create plugin
  Transport * impl =
    transportLayerFactory.createTransport(id, pPlatformService);

  TransportLayer * pTransportLayer = new TransportLayer{id, impl, pPlatformService};

  // new concreate layer
  std::unique_ptr<NEMLayer> pNEMLayer{new NEMStatefulLayer{id,
        pTransportLayer,
        pPlatformService}};


  // register to the component map
  BuildId buildId{BuildIdServiceSingleton::instance()->registerBuildable(pNEMLayer.get(),
                                                                         COMPONENT_TRANSPORTILAYER,
                                                                         sLibraryFile)};

  ConfigurationServiceSingleton::instance()->registerRunningStateMutable(buildId,
                                                                         pNEMLayer.get());

  // pass nem to platform service
  pPlatformService->setPlatformServiceUser(buildId,pNEMLayer.get());

  // set the FileDescriptor Service
  pPlatformService->setFileDescriptorServiceProvider(pTransportLayer);

  // register event service handler with event service
  EventServiceSingleton::instance()->registerEventServiceUser(buildId,
                                                              pNEMLayer.get(),
                                                              id);

  RegistrarProxy registrarProxy{buildId};

  // initialize
  pNEMLayer->initialize(registrarProxy);

  if(!bSkipConfigure)
    {
      pNEMLayer->configure(ConfigurationServiceSingleton::instance()->buildUpdates(buildId,
                                                                                   request));
    }

  return pNEMLayer;
}


EMANE::PlatformServiceProvider *
EMANE::Application::TransportBuilder::newPlatformService() const
{
  return new EMANE::PlatformService{};
}


std::unique_ptr<EMANE::Application::TransportAdapter>
EMANE::Application::TransportBuilder::buildTransportWithAdapter_i(Transport * pTransport,
                                                                  PlatformServiceProvider * pProvider,
                                                                  const ConfigurationUpdateRequest & request,
                                                                  const std::string & sPlatformEndpoint,
                                                                  const std::string & sTransportEndpoint) const
{
  // pass transport to platform service
  EMANE::PlatformService * pPlatformService{dynamic_cast<EMANE::PlatformService*>(pProvider)};

  NEMId id{pTransport->getNEMId()};

  TransportLayer * pTransportLayer = new TransportLayer{id,pTransport, pPlatformService};

  // new concreate layer
  std::unique_ptr<NEMLayer> pNEMLayer{new NEMStatefulLayer{id,
        pTransportLayer,
        pPlatformService}};

  // register to the component map

  BuildId buildId{BuildIdServiceSingleton::instance()->registerBuildable(pTransport)};

  ConfigurationServiceSingleton::instance()->registerRunningStateMutable(buildId,
                                                                         pNEMLayer.get());

  // pass nem to platform service
  pPlatformService->setPlatformServiceUser(buildId,pNEMLayer.get());

  // set the FileDescriptor Service
  pPlatformService->setFileDescriptorServiceProvider(pTransportLayer);

  // register event service handler with event service
  EventServiceSingleton::instance()->registerEventServiceUser(buildId,
                                                              pNEMLayer.get(),
                                                              id);

  RegistrarProxy registrarProxy{buildId};

  // initialize
  pNEMLayer->initialize(registrarProxy);

  pNEMLayer->configure(ConfigurationServiceSingleton::instance()->buildUpdates(buildId,
                                                                               request));

  auto pTransportAdapter = buildTransportAdapter(pNEMLayer,
                                                 {
                                                   {"platformendpoint",{sPlatformEndpoint}},
                                                     {"transportendpoint",{sTransportEndpoint}}
                                                 });


  return pTransportAdapter;
}
