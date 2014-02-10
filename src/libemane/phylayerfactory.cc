/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "phylayerfactory.h"
#include "emane/utils/factoryexception.h"

#include <ace/OS_NS_dlfcn.h>

EMANE::PHYLayerFactory::PHYLayerFactory(const std::string & sLibraryName):
  shlibHandle_(0),
  createLayerFunc_(0),
  destroyLayerFunc_(0)
{
  if((shlibHandle_ = ACE_OS::dlopen(sLibraryName.c_str(), RTLD_NOW)) == 0)
    {
      throw Utils::FactoryException(ACE_OS::dlerror());
    }
  
  if((createLayerFunc_ = reinterpret_cast<createLayerFunc>((void (*)())(ACE_OS::dlsym(shlibHandle_,"create")))) == 0)
    {
      ACE_OS::dlclose(shlibHandle_);
      throw makeException<Utils::FactoryException>("%s  missing create symbol. (Missing DECLARE_PHY_LAYER()?)",
                                                   sLibraryName.c_str());
    }

  if((destroyLayerFunc_ = reinterpret_cast<destroyLayerFunc>((void (*)())ACE_OS::dlsym(shlibHandle_,"destroy"))) == 0)
    {
      ACE_OS::dlclose(shlibHandle_);
      throw makeException<Utils::FactoryException>("%s missing destroy symbol. (Missing DECLARE_PHY_LAYER()?)",
                                                   sLibraryName.c_str());
    }
}

EMANE::PHYLayerFactory::~PHYLayerFactory()
{
  ACE_OS::dlclose(shlibHandle_);
}

EMANE::PHYLayerImplementor * EMANE::PHYLayerFactory::createLayer(NEMId id, PlatformServiceProvider * pPlatformServiceProvider) const
{
  return createLayerFunc_(id, pPlatformServiceProvider);
}

void EMANE::PHYLayerFactory::destoryLayer(PHYLayerImplementor * pLayer) const
{
  destroyLayerFunc_(pLayer);
}
