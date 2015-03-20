/*
 * Copyright (c) 2013,2015 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2008-2009 - DRS CenGen, LLC, Columbia, Maryland
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

#include "eventgeneratorfactory.h"
#include "emane/utils/factoryexception.h"
#include <dlfcn.h>

EMANE::EventGeneratorFactory::EventGeneratorFactory(const std::string & sLibraryName):
  pLibHandle_{},
  createEventGeneratorFunc_{},
  destroyEventGeneratorFunc_{}
{
   if((pLibHandle_ = dlopen(sLibraryName.c_str(), RTLD_NOW)) == 0)
    {
      throw Utils::FactoryException(dlerror());
    }
  
  if((createEventGeneratorFunc_ = reinterpret_cast<CreateEventGeneratorFunc>((void (*)())(dlsym(pLibHandle_,"create")))) == 0)
    {
      dlclose(pLibHandle_);
      throw makeException<Utils::FactoryException>("%s  missing create symbol. (Missing DECLARE_EVENT_GENERATOR()?)",
                                                   sLibraryName.c_str());
    }

  if((destroyEventGeneratorFunc_ = reinterpret_cast<DestroyEventGeneratorFunc>((void (*)())dlsym(pLibHandle_,"destroy"))) == 0)
    {
      dlclose(pLibHandle_);
      throw makeException<Utils::FactoryException>("%s missing destory symbol. (Missing DECLARE_EVENT_GENERATOR()?)",
                                                   sLibraryName.c_str());

    }
}

EMANE::EventGeneratorFactory::~EventGeneratorFactory()
{
  dlclose(pLibHandle_);
}

EMANE::EventGenerator * EMANE::EventGeneratorFactory::createEventGenerator(EMANE::PlatformServiceProvider *pPlatformService) const
{
  return createEventGeneratorFunc_(pPlatformService);
}

void EMANE::EventGeneratorFactory::destoryEventGenerator(EventGenerator * pGenerator) const
{
  destroyEventGeneratorFunc_(pGenerator);
}
