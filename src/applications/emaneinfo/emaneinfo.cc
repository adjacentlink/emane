/*
 * Copyright (c) 2013-2014,2016 - Adjacent Link LLC, Bridgewater, New
 * Jersey
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
 * * Neither the name of Adjacent Link LLC nor the names of its
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

#include "emane/application/libemane.h"
#include "emane/application/nembuilder.h"
#include "emane/application/eventgeneratorbuilder.h"
#include "emane/application/eventagentbuilder.h"
#include "emane/application/transportbuilder.h"
#include "emane/application/logger.h"
#include "emane/application/configurationcontroller.h"

#include "emane/exception.h"
#include "manifest.h"
#include "configurationloader.h"
#include "nemmanagerimpl.h"
#include "transportmanagerimpl.h"
#include "eventgeneratormanagerimpl.h"
#include "eventagentmanagerimpl.h"
#include "registrarproxy.h"
#include "buildidservice.h"
#include <sstream>
#include <iostream>
#include <tuple>
#include <uuid.h>
#include <getopt.h>

void usage();

template<typename T>
T * createManager()
{
  uuid_t uuid{};
  uuid_generate(uuid);
  T *  pManager{new T{uuid}};
  auto buildId = EMANE::BuildIdServiceSingleton::instance()->registerBuildable(pManager);
  EMANE::RegistrarProxy registrarProxy{buildId};
  pManager->initialize(registrarProxy);
  return pManager;
}

int main(int argc, char * argv[])
{
  EMANE::Application::initialize();

  try
    {
      std::vector<option> options =
        {
          {"help",0,nullptr,'h'},
          {"version",0,nullptr,'v'},
          {"configuration",0,nullptr,'c'},
          {"manifest",0,nullptr,'m'},
          {0, 0,nullptr,0},
        };

      int iOption{};
      int iOptionIndex{};
      bool bConfiguration{false};
      bool bShowManifest{false};

      while((iOption = getopt_long(argc,argv,"hcmv", &options[0],&iOptionIndex)) != -1)
        {
          switch(iOption)
            {
            case 'h':
              usage();
              return EXIT_SUCCESS;
              break;

            case 'v':
              // --version
              std::cout<<VERSION<<std::endl;
              return EXIT_SUCCESS;

            case 'c':
              // --configuration
              bConfiguration = true;
              break;

            case 'm':
              bShowManifest = true;
              break;

            default:
              std::cerr<<"Unknown option: -"<<static_cast<char>(iOption)<<std::endl;
              return EXIT_FAILURE;
            }
        }

      std::string sFileName;

      if(optind >= argc)
        {
          std::cerr<<"Missing plugin"<<std::endl;
          return EXIT_FAILURE;
        }
      else
        {
          sFileName = argv[optind];
        }

      EMANE::Application::NEMBuilder nemBuilder;
      EMANE::Application::EventGeneratorBuilder eventGeneratorBuilder;
      EMANE::Application::EventAgentBuilder eventAgentBuilder;
      EMANE::Application::TransportBuilder transportBuilder;

      EMANE::ConfigurationUpdateRequest request;
      std::string sPluginName;
      EMANE::Application::PluginType pluginType =  EMANE::Application::PluginType::SHIM;

      if(bConfiguration)
        {
          EMANE::Application::ConfigurationLoader loader(sFileName);
          sPluginName = loader.getPluginName();
          request = loader.getConfigurationUpdateRequest();
          pluginType = loader.getPluginType();
        }
      else
        {
          sPluginName = sFileName;
        }

      std::unique_ptr<EMANE::Component> pComponent;
      EMANE::BuildId buildId{};

      if(sPluginName == "nemmanager")
        {
          auto pManager = createManager<EMANE::Application::NEMManagerImpl>();
          buildId = pManager->getBuildId();
          pComponent.reset(pManager);
        }
      else if(sPluginName == "transportmanager")
        {
          auto pManager = createManager<EMANE::Application::TransportManagerImpl>();
          buildId = pManager->getBuildId();
          pComponent.reset(pManager);
        }
      else if(sPluginName == "eventgeneratormanager")
        {
          auto pManager = createManager<EMANE::Application::EventGeneratorManagerImpl>();
          buildId = pManager->getBuildId();
          pComponent.reset(pManager);
        }
      else if(sPluginName == "eventagentmanager")
        {
          auto pManager = createManager<EMANE::Application::EventAgentManagerImpl>();
          buildId = pManager->getBuildId();
          pComponent.reset(pManager);
        }
      else
        {
          if(sPluginName == "emanephy" || sPluginName.empty())
            {
              sPluginName.clear();
              pluginType = EMANE::Application::PluginType::PHY;
            }


          // the nem layer API is all the same, so you can get
          // away with building a shim if you are accessing
          // the Component and Buildable API
          switch(pluginType)
            {
            case EMANE::Application::PluginType::PHY:
              {
                auto pPlugin = nemBuilder.buildPHYLayer(1,sPluginName,request,!bConfiguration);
                buildId = pPlugin->getBuildId();
                pComponent.reset(pPlugin.release());
              }
              break;

            case EMANE::Application::PluginType::MAC:
              {
                auto pPlugin = nemBuilder.buildMACLayer(1,sPluginName,request,!bConfiguration);
                buildId = pPlugin->getBuildId();
                pComponent.reset(pPlugin.release());
              }
              break;

            case EMANE::Application::PluginType::SHIM:
              {
                auto pPlugin = nemBuilder.buildShimLayer(1,sPluginName,request,!bConfiguration);
                buildId = pPlugin->getBuildId();
                pComponent.reset(pPlugin.release());
              }
              break;

            case EMANE::Application::PluginType::GENERATOR:
              {
                auto pPlugin = eventGeneratorBuilder.buildEventGenerator(sPluginName,request,!bConfiguration);
                buildId = pPlugin->getBuildId();
                pComponent.reset(pPlugin.release());
              }
              break;

            case EMANE::Application::PluginType::AGENT:
              {
                auto pPlugin = eventAgentBuilder.buildEventAgent(1,sPluginName,request,!bConfiguration);
                buildId = pPlugin->getBuildId();
                pComponent.reset(pPlugin.release());
              }
              break;

            case EMANE::Application::PluginType::TRANSPORT:
              {
                auto pPlugin = transportBuilder.buildTransport(1,sPluginName,request,!bConfiguration);
                buildId = pPlugin->getBuildId();
                pComponent.reset(pPlugin.release());
              }

              break;
            }
        }

      if(bShowManifest)
        {
          std::cout<<EMANE::Application::manifest(buildId,sFileName);
        }
    }
  catch(const EMANE::Exception & exp)
    {
      std::cerr<<exp.what()<<std::endl;

      EMANE::Application::shutdown();

      return EXIT_FAILURE;
    }
  catch(const std::exception & exp)
    {
      std::cerr<<exp.what()<<std::endl;

      EMANE::Application::shutdown();

      return EXIT_FAILURE;
    }

  EMANE::Application::shutdown();

  return EXIT_SUCCESS;
}

void usage()
{
  std::cout<<"usage: emaneinfo [OPTIONS]... <plugin>|'nemmanager'|'transportmanager'|"<<std::endl;
  std::cout<<"                              'eventagentmanager'|'eventgeneratormanager'"<<std::endl;
  std::cout<<std::endl;
  std::cout<<" CONFIG_URI                    URI of XML configuration file."<<std::endl;
  std::cout<<std::endl;
  std::cout<<"options:"<<std::endl;
  std::cout<<"  -h, --help                     Print this message and exit."<<std::endl;
  std::cout<<"  -v, --version                  Print version and exit."<<std::endl;
  std::cout<<std::endl;
}
