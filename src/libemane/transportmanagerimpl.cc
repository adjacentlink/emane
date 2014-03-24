/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2011-2012 - DRS CenGen, LLC, Columbia, Maryland
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

#include "transportmanagerimpl.h"
#include "timerservice.h"

#include "emane/configureexception.h"

EMANE::Application::TransportManagerImpl::TransportManagerImpl(const uuid_t & uuid):
  TransportManager{uuid}{}

EMANE::Application::TransportManagerImpl::~TransportManagerImpl(){}

void EMANE::Application::TransportManagerImpl::initialize(Registrar &)
{}
    
void EMANE::Application::TransportManagerImpl::configure(const ConfigurationUpdate & update)
{
  if(!update.empty())
    {
      throw ConfigureException("TransportManagerImpl: Unexpected configuration items.");
    }
}
    
void EMANE::Application::TransportManagerImpl::start()
{
  std::for_each(transportAdapters_.begin(),
                transportAdapters_.end(),
                std::bind(&Component::start,std::placeholders::_1));
}

void EMANE::Application::TransportManagerImpl::postStart()
{
  std::for_each(transportAdapters_.begin(),
                transportAdapters_.end(),
                std::bind(&Component::postStart,std::placeholders::_1));
}

void EMANE::Application::TransportManagerImpl::stop()
{
   std::for_each(transportAdapters_.begin(),
                transportAdapters_.end(),
                 std::bind(&Component::stop,std::placeholders::_1));
}

void EMANE::Application::TransportManagerImpl::destroy()
  throw()
{
  std::for_each(transportAdapters_.begin(),
                transportAdapters_.end(),
                std::bind(&Component::destroy,std::placeholders::_1));
}

void
EMANE::Application::TransportManagerImpl::add(std::unique_ptr<Application::TransportAdapter> & pTransportAdapter)
{
  transportAdapters_.push_back(std::move(pTransportAdapter)); 
}

