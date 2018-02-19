/*
 * Copyright (c) 2017 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANEFADINGALGORITHM_HEADER_
#define EMANEFADINGALGORITHM_HEADER_

#include "emane/platformserviceprovider.h"
#include "emane/registrar.h"

namespace EMANE
{
  class FadingAlgorithm
  {
  public:
    FadingAlgorithm(const std::string & sName,
                    NEMId id,
                    PlatformServiceProvider * pPlatformService,
                    const std::string & sPrefix):
      sName_{sName},
      id_{id},
      pPlatformService_(pPlatformService),
      sPrefix_{sPrefix}{}

    virtual ~FadingAlgorithm(){};

    virtual void initialize(Registrar & registrar) = 0;

    virtual void configure(const ConfigurationUpdate & update) = 0;

    virtual void modify(const ConfigurationUpdate & update) = 0;

    virtual double operator()(double dPowerdBm,
                              double dDistanceMeters) = 0;

    std::string name() const {return sName_;}

  protected:
    const std::string sName_;
    const NEMId id_;
    PlatformServiceProvider * const pPlatformService_;
    const std::string sPrefix_;
  };
}

#endif // EMANEFADINGALGORITHM_HEADER_
