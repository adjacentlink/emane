/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2011 - DRS CenGen, LLC, Columbia, Maryland
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

#include "nemimpl.h"
#include "logservice.h"
#include "emane/configureexception.h"
#include "emane/startexception.h"

#include <sstream>
#include <iomanip>

EMANE::Application::NEMImpl::NEMImpl(NEMId id, 
                                     std::unique_ptr<NEMLayerStack> & pNEMLayerStack,
                                     bool bExternalTransport):
  pNEMLayerStack_(std::move(pNEMLayerStack)),
  id_{id},
  bExternalTransport_{bExternalTransport},
  NEMOTAAdapter_{id},
  NEMNetworkAdapter_{id}
{
  pNEMLayerStack_->connectLayers(&NEMNetworkAdapter_,&NEMOTAAdapter_);
}

EMANE::Application::NEMImpl::~NEMImpl(){}
    
void EMANE::Application::NEMImpl::initialize(Registrar & registrar)
{
  if(bExternalTransport_)
    {
      auto & configRegistrar = registrar.configurationRegistrar();
      
      configRegistrar.registerNonNumeric<ACE_INET_Addr>("platformendpoint",
                                                        ConfigurationProperties::REQUIRED,
        {},
                                                        "IPv4 or IPv6 NEM Platform Service endpoint.");

      configRegistrar.registerNonNumeric<ACE_INET_Addr>("transportendpoint",
                                                        ConfigurationProperties::REQUIRED,
        {},
                                                        "IPv4 or IPv6 Transport endpoint.");
    }

  pNEMLayerStack_->initialize(registrar); 
}
    
void EMANE::Application::NEMImpl::configure(const ConfigurationUpdate & update)
{
  for(const auto & item : update)
    {
      if(item.first == "platformendpoint")
        {
          platformEndpointAddr_ = item.second[0].asINETAddr();
          
          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  INFO_LEVEL,
                                  "NEM  %03hu NEMImpl::configure platformendpoint: %s/%hu",
                                  id_,
                                  platformEndpointAddr_.get_host_addr(),
                                  platformEndpointAddr_.get_port_number());
          
        }
      else if(item.first == "transportendpoint")
        {
          transportEndpointAddr_ = item.second[0].asINETAddr();
          
          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  INFO_LEVEL,
                                  "NEM  %03hu NEMImpl::configure transportendpoint: %s/%hu",
                                  id_,
                                  transportEndpointAddr_.get_host_addr(),
                                  transportEndpointAddr_.get_port_number());
        }
      else
        {
          throw makeException<ConfigureException>("NEMImpl::configure: "
                                                  "Unexpected configuration item %s",
                                                  item.first.c_str());
        }
    }
}
    
void EMANE::Application::NEMImpl::start()
{
  NEMOTAAdapter_.open();

  if(bExternalTransport_)
    {
      try
        {
          NEMNetworkAdapter_.open(platformEndpointAddr_,transportEndpointAddr_);
        }
      catch(NetworkAdapterException & exp)
        {
          throw StartException(exp.what());
        }
    }

  pNEMLayerStack_->start();
}

void EMANE::Application::NEMImpl::postStart()
{
  pNEMLayerStack_->postStart();
}
    
void EMANE::Application::NEMImpl::stop()
{
  if(bExternalTransport_)
    {
      NEMNetworkAdapter_.close();
    }

  NEMOTAAdapter_.close();

  pNEMLayerStack_->stop(); 
}
    
void EMANE::Application::NEMImpl::destroy()
  throw()
{
  pNEMLayerStack_->destroy();
}

EMANE::NEMId EMANE::Application::NEMImpl::getNEMId() const
{
  return id_;
}
