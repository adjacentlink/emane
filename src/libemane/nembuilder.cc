/*
 * Copyright (c) 2013-2014,2016 - Adjacent Link LLC, Bridgewater, New
 * Jersey
 * Copyright (c) 2008-2009-2010 - DRS CenGen, LLC, Columbia, Maryland
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
#include "nemmanagerimpl.h"
#include "emane/application/nembuilder.h"
#include "emane/buildexception.h"
#include "timerserviceproxy.h"
#include "maclayer.h"
#include "phylayer.h"
#include "shimlayer.h"
#include "transportlayer.h"
#include "nemstatefullayer.h"
#include "nemimpl.h"
#include "buildidservice.h"
#include "layerfactorymanager.h"
#include "transportfactorymanager.h"
#include "logservice.h"
#include "eventservice.h"
#include "platformservice.h"
#include "registrarproxy.h"
#include "frameworkphy.h"
#include "radioservice.h"
#include "spectrummonitor.h"

class EMANE::Application::NEMBuilder::NEMBuilderImpl
{
public:
  SpectrumMonitor * getSpectrumMonitor(NEMId id)
  {
    auto iter = spectrumMonitorCache_.find(id);

    if(iter != spectrumMonitorCache_.end())
      {
        return iter->second;
      }
    else
      {
        SpectrumMonitor * pSpectrumMonitor{new SpectrumMonitor};

        spectrumMonitorCache_.insert(std::make_pair(id,pSpectrumMonitor));

        return pSpectrumMonitor;
      }
  }

private:
  using SpectrumMonitorCache = std::map<NEMId,SpectrumMonitor *>;
  SpectrumMonitorCache spectrumMonitorCache_;
};

EMANE::Application::NEMBuilder::NEMBuilder():
  pImpl_{new NEMBuilderImpl}{}

EMANE::Application::NEMBuilder::~NEMBuilder(){}

std::unique_ptr<EMANE::NEMLayer>
EMANE::Application::NEMBuilder::buildPHYLayer(NEMId id,
                                              const std::string & sLibraryFile,
                                              const ConfigurationUpdateRequest & request,
                                              bool bSkipConfigure)
{
  std::string sNativeLibraryFile = "lib" +
                                   sLibraryFile +
                                   ".so";

  // new platform service
  PlatformService *
    pPlatformService{new PlatformService{}};

  PHYLayerImplementor * pImpl{};

  std::string sRegistrationName{};

  // no library specification implies the framework phy implementation
  // should be used
  if(sLibraryFile.empty())
    {
      pImpl = new FrameworkPHY{id, pPlatformService, pImpl_->getSpectrumMonitor(id)};

      sRegistrationName = "emanephy";
    }
  else
    {
      const PHYLayerFactory & phyLayerFactory =
        LayerFactoryManagerSingleton::instance()->getPHYLayerFactory(sNativeLibraryFile);

      // create plugin
      pImpl =  phyLayerFactory.createLayer(id, pPlatformService);

      sRegistrationName = sLibraryFile;
    }

  // new concrete layer
  std::unique_ptr<NEMLayer> pNEMLayer{new NEMStatefulLayer{id,
        new PHYLayer(id, pImpl, pPlatformService),
        pPlatformService}};

  BuildId buildId{BuildIdServiceSingleton::instance()->registerBuildable(pNEMLayer.get(),
                                                                         COMPONENT_PHYILAYER,
                                                                         sRegistrationName)};

  ConfigurationServiceSingleton::instance()->registerRunningStateMutable(buildId,
                                                                         pNEMLayer.get());


  // pass nem to platform service
  pPlatformService->setPlatformServiceUser(buildId,pNEMLayer.get());

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

std::unique_ptr<EMANE::NEMLayer>
EMANE::Application::NEMBuilder::buildMACLayer(NEMId id,
                                              const std::string & sLibraryFile,
                                              const ConfigurationUpdateRequest & request,
                                              bool bSkipConfigure)
{
  std::string sNativeLibraryFile = "lib" +
                                   sLibraryFile +
                                   ".so";

  const MACLayerFactory & macLayerFactory =
    LayerFactoryManagerSingleton::instance()->getMACLayerFactory(sNativeLibraryFile);

  // new platform service
  PlatformService * pPlatformService{new PlatformService{}};

  // new radio service
  RadioService * pRadioService{new RadioService{pImpl_->getSpectrumMonitor(id)}};

  // create plugin
  MACLayerImplementor * impl =
    macLayerFactory.createLayer(id, pPlatformService,pRadioService);

  // new concreate layer
  std::unique_ptr<NEMLayer> pNEMLayer{new NEMStatefulLayer{id,
        new MACLayer{id, impl, pPlatformService},
        pPlatformService}};

  // register to the component map
  BuildId buildId{BuildIdServiceSingleton::instance()->registerBuildable(pNEMLayer.get(),
                                                                         COMPONENT_MACILAYER,
                                                                         sLibraryFile)};

  ConfigurationServiceSingleton::instance()->registerRunningStateMutable(buildId,
                                                                         pNEMLayer.get());


  // pass nem to platform service
  pPlatformService->setPlatformServiceUser(buildId,pNEMLayer.get());

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


std::unique_ptr<EMANE::NEMLayer>
EMANE::Application::NEMBuilder::buildShimLayer(NEMId id,
                                               const std::string & sLibraryFile,
                                               const ConfigurationUpdateRequest & request,
                                               bool bSkipConfigure)
{
  std::string sNativeLibraryFile = "lib" +
                                   sLibraryFile +
                                   ".so";



  const ShimLayerFactory & shimLayerFactory =
    LayerFactoryManagerSingleton::instance()->getShimLayerFactory(sNativeLibraryFile);

  // new platform service
  PlatformService *
    pPlatformService{new PlatformService{}};

  // new radio service
  RadioService * pRadioService{new RadioService{pImpl_->getSpectrumMonitor(id)}};

  // create plugin
  ShimLayerImplementor * impl =
    shimLayerFactory.createLayer(id, pPlatformService,pRadioService);

  // new concreate layer
  std::unique_ptr<NEMLayer> pNEMLayer{new NEMStatefulLayer{id,
        new ShimLayer{id, impl, pPlatformService},
        pPlatformService}};


  // register to the component map
  BuildId buildId{BuildIdServiceSingleton::instance()->registerBuildable(pNEMLayer.get(),
                                                                         COMPONENT_SHIMILAYER,
                                                                         sLibraryFile)};

  ConfigurationServiceSingleton::instance()->registerRunningStateMutable(buildId,
                                                                         pNEMLayer.get());

  // pass nem to platform service
  pPlatformService->setPlatformServiceUser(buildId,pNEMLayer.get());

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


std::unique_ptr<EMANE::Application::NEM>
EMANE::Application::NEMBuilder::buildNEM(NEMId id,
                                         NEMLayers & layers,
                                         const ConfigurationUpdateRequest & request,
                                         bool bExternalTransport)
{
  std::unique_ptr<NEMLayerStack> pLayerStack{new NEMLayerStack};

  if(layers.empty())
    {
      throw BuildException("Trying to build a NEM without NEMLayers.");
    }

  std::vector<BuildId> nemBuildIds;
  for(auto & pLayer : layers)
    {
      if(pLayer->getNEMId() != id)
        {

          throw makeException<BuildException>("NEMId mismatch: NEMLayer (%hu) NEM (%hu)",
                                              pLayer->getNEMId(),
                                              id);
        }

      pLayerStack->addLayer(pLayer);
    }

  std::unique_ptr<NEM> pNEM{new NEMImpl{id,pLayerStack,bExternalTransport}};

  // register to the component map
  BuildId buildId{BuildIdServiceSingleton::instance()->registerBuildable(pNEM.get())};

  RegistrarProxy registrarProxy{buildId};

  pNEM->initialize(registrarProxy);

  pNEM->configure(ConfigurationServiceSingleton::instance()->buildUpdates(buildId,
                                                                          request));

  return pNEM;
}


std::unique_ptr<EMANE::Application::NEMManager>
EMANE::Application::NEMBuilder::buildNEMManager(const uuid_t & uuid,
                                                NEMs & nems,
                                                const ConfigurationUpdateRequest & request)
{
  if(nems.empty())
    {
      throw BuildException("Trying to build an NEM Manager without any NEMs.");
    }

  std::unique_ptr<Application::NEMManager> pPlatform{new NEMManagerImpl{uuid}};

  BuildId buildId{0};

  RegistrarProxy registrarProxy{buildId};

  pPlatform->initialize(registrarProxy);

  pPlatform->configure(ConfigurationServiceSingleton::instance()->buildUpdates(buildId,
                                                                               request));

  std::for_each(nems.begin(),
                nems.end(),
                [&pPlatform](std::unique_ptr<NEM> & pNEM)
                {
                  pPlatform->add(pNEM);
                });

  return pPlatform;
}

std::unique_ptr<EMANE::NEMLayer>
EMANE::Application::NEMBuilder::buildTransportLayer(NEMId id,
                                                    const std::string & sLibraryFile,
                                                    const ConfigurationUpdateRequest & request,
                                                    bool bSkipConfigure)
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

  // new concreate layer
  std::unique_ptr<NEMLayer> pNEMLayer{new NEMStatefulLayer{id,
        new TransportLayer{id, impl, pPlatformService},
        pPlatformService}};


  // register to the component map
  BuildId buildId{BuildIdServiceSingleton::instance()->registerBuildable(pNEMLayer.get(),
                                                                         COMPONENT_TRANSPORTILAYER,
                                                                         sLibraryFile)};

  ConfigurationServiceSingleton::instance()->registerRunningStateMutable(buildId,
                                                                         pNEMLayer.get());

  // pass nem to platform service
  pPlatformService->setPlatformServiceUser(buildId,pNEMLayer.get());

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
