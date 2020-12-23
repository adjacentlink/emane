/*
 * Copyright (c) 2017,2020 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANEFADINGALGORITHMMANAGER_HEADER_
#define EMANEFADINGALGORITHMMANAGER_HEADER_

#include "fadingalgorithm.h"
#include "emane/platformserviceprovider.h"
#include "emane/registrar.h"
#include "emane/events/fadingmodel.h"

namespace EMANE
{
  class FadingAlgorithmManager
  {
  public:
    FadingAlgorithmManager(const std::string & sName,
                           Events::FadingModel type,
                           NEMId id,
                           PlatformServiceProvider * pPlatformService,
                           const std::string & sPrefix):
      sName_{sName},
      type_{type},
      id_{id},
      pPlatformService_(pPlatformService),
      sPrefix_{sPrefix}{}

    virtual ~FadingAlgorithmManager(){};

    virtual void initialize(Registrar & registrar) = 0;

    virtual void configure(const ConfigurationUpdate & update) = 0;

    virtual void modify(const ConfigurationUpdate & update) = 0;

    virtual const void * getParameters() const = 0;

    const std::string & name()
    {
      return sName_;
    }

    const Events::FadingModel & type()
    {
      return type_;
    }

    virtual std::unique_ptr<FadingAlgorithm> createFadingAlgorithm() = 0;

  protected:
    const std::string sName_;
    const Events::FadingModel type_;
    const NEMId id_;
    PlatformServiceProvider * const pPlatformService_;
    const std::string sPrefix_;
  };
}

#endif // EMANEFADINGALGORITHMMANAGER_HEADER_
