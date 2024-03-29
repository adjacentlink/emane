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

#ifndef EMANEFADINGMANAGER_HEADER_
#define EMANEFADINGMANAGER_HEADER_

#include "emane/platformserviceprovider.h"
#include "emane/registrar.h"
#include "emane/events/fadingselection.h"
#include "locationinfo.h"
#include "fadingalgorithm.h"
#include "fadingalgorithmmanager.h"

#include <map>
#include <memory>

namespace EMANE
{
  using FadingAlgorithmStore = std::map<Events::FadingModel,
                                        std::unique_ptr<FadingAlgorithm>>;

  using FadingInfo = std::pair<Events::FadingModel,const void *>;

  class FadingManager
  {
  public:
    FadingManager(NEMId id,
                  PlatformServiceProvider * pPlatformService,
                  const std::string & sPrefix);

    void initialize(Registrar & registrar);

    void configure(const ConfigurationUpdate & update);

    void modify(const ConfigurationUpdate & update);

    enum class FadingStatus
      {
       SUCCESS = 0,
       ERROR_LOCATIONINFO,
       ERROR_ALGORITHM,
       ERROR_SELECTION,
      };

    FadingAlgorithmStore createFadingAlgorithmStore() const;

    std::pair<FadingInfo,bool> getFadingSelection(NEMId nemId) const;

    std::pair<double,FadingStatus> calculate(NEMId txNEMId,
                                             double dPowerdBm,
                                             const std::pair<LocationInfo,bool> & location) const;

    void update(const Events::FadingSelections & fadingSelections);

  private:
    NEMId id_;
    PlatformServiceProvider * pPlatformService_;
    std::string sPrefix_;
    using FadingAlgorithmManagers = std::map<std::string,
                                             std::unique_ptr<FadingAlgorithmManager>>;

    using TxNEMFadingSelections = std::map<NEMId,
                                           std::pair<Events::FadingModel,
                                                     FadingAlgorithmManager *>>;

    TxNEMFadingSelections TxNEMFadingSelections_;
    bool bFading_;
    FadingAlgorithmManagers fadingAlgorithmManagers_;
    FadingAlgorithmManager * pFadingAlgorithmManagerForAll_;

    void configure_i(const ConfigurationUpdate & update,
                     void (FadingAlgorithmManager::*)(const ConfigurationUpdate&));
  };
};

#endif // EMANEFADINGMANAGER_HEADER_
