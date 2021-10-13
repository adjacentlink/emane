/*
 * Copyright (c) 2013-2014,2016,2020 - Adjacent Link LLC, Bridgewater,
 * New Jersey
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

#ifndef EMANETESTUTILS_HEADER_
#define EMANETESTUTILS_HEADER_

#include "platformservice.h"
#include "buildidservice.h"
#include "configurationservice.h"
#include "registrarproxy.h"
#include "eventservice.h"
#include "frameworkphy.h"
#include "statisticservice.h"

#include <iostream>
#include <iomanip>

namespace EMANE
{
  namespace Test
  {
    FrameworkPHY * createPHY(NEMId id, SpectrumService * pSpectrumService)
    {
      PlatformService * pPlatformService{new PlatformService{}};

      FrameworkPHY * pPHYLayer{new FrameworkPHY{id, pPlatformService,pSpectrumService}};

      BuildId buildId{BuildIdServiceSingleton::instance()->registerBuildable(pPHYLayer,
                                                                             COMPONENT_PHYILAYER,
                                                                             "")};

      ConfigurationServiceSingleton::instance()->registerRunningStateMutable(buildId,
                                                                             pPHYLayer);

      pPlatformService->setPlatformServiceUser(buildId,pPHYLayer);

      // register event service handler with event service
      EventServiceSingleton::instance()->registerEventServiceUser(buildId,
                                                                  pPHYLayer,
                                                                  id);

      RegistrarProxy registrarProxy{buildId};

      // initialize
      pPHYLayer->initialize(registrarProxy);

      return pPHYLayer;
    }

    void dumpTables(BuildId buildId,
                    const std::vector<std::string> & names)
    {
      auto results =
        StatisticService::instance()->queryTable(buildId,names);
      for(const auto entry : results)
        {
          std::vector<int> labelLengths;
          for(const auto & label : entry.second.first)
            {
              labelLengths.push_back(label.size());
              std::cout<<std::setiosflags(std::ios::left) <<std::setw(label.size())<<label<<"|";
            }

          std::cout<<std::endl;

          for(const auto & row : entry.second.second)
            {
              int i{};
              for(const auto & any : row)
                {
                  std::cout<<std::setiosflags(std::ios::left) <<std::setw(labelLengths[i++])<<any.toString()<<"|";
                }

              std::cout<<std::endl;
            }
        }
      std::cout<<std::endl;
    }

    void
    dumpDropTables(BuildId buildId)
    {
      dumpTables(buildId,{"UnicastPacketDropTable0",
                          "BroadcastPacketDropTable0"});
    }

    FrequencySet generateFrequencySet(std::size_t size,
                                      std::uint64_t u64BeginFrequencyHz,
                                      std::uint64_t u64BandwidthHz)
    {
      FrequencySet frequencySet{};

      for(std::size_t i = 0; i < size; ++i)
        {
          frequencySet.insert(u64BeginFrequencyHz + u64BandwidthHz * 2 * i);
        }

      return frequencySet;
    }


    void dumpGainCacheStats(BuildId buildId)
    {
      auto results =
        StatisticService::instance()->queryStatistic(buildId,
                                                     {"numGainCacheHit",
                                                      "numGainCacheMiss"});

      for(const auto & result : results)
        {
          std::cout<<result.first<<"="<<result.second.toString()<<std::endl;
        }
    }
  }
}

#endif // EMANETESTUTILS_HEADER_
