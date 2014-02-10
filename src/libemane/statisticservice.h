/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANESTATISTICSERVICE_HEADER_
#define EMANESTATISTICSERVICE_HEADER_

#include "emane/types.h"
#include "emane/statistic.h"
#include "emane/statistictable.h"
#include "emane/statisticinfo.h"
#include "emane/statistictableinfo.h"
#include "emane/statisticproperties.h"
#include "emane/utils/singleton.h"

#include <string>
#include <map>
#include <vector>
#include <memory>


namespace EMANE
{
  using StatisticManifest = std::vector<StatisticInfo>;

  using StatisticTableManifest = std::vector<StatisticTableInfo>;

  class StatisticService : public Utils::Singleton<StatisticService>
  {
  public:
    /**
     * Register a statistic and take ownership
     *
     * @param buildId Build Id of the registering component
     * @param sName Name of the statistic
     * @param type Any type of the statistic
     * @param properties Statistic properties
     * @param sDescription Statistic description
     * @param pStatistic Pointer to the statistic
     *
     * @throw RegistrarException when a error occurs during
     * registration.
     */
    void registerStatistic(BuildId buildId,
                           const std::string & sName,
                           Any::Type type,
                           const StatisticProperties & properties,
                           const std::string & sDescription,
                           Statistic * pStatistic);

   
    /**
     * Register a statistic table and take ownership
     *
     * @param buildId Build Id of the registering component
     * @param sName Name of the statistic table
     * @param properties Statistic properties
     * @param sDescription Statistic table description
     * @param pStatisticTablePublisher Pointer to the statistic table publisher
     * @param clearFunc Function object to excute as part of clear
     *
     * @throw RegistrarException when a error occurs during
     * registration.
     */
    void registerTable(BuildId buildId,
                       const std::string & sName,
                       const StatisticProperties & properties,
                       const std::string & sDescription,
                       StatisticTablePublisher * pStatisticTablePublisher,
                       std::function<void(StatisticTablePublisher *)> clearFunc);


    std::map<std::string,EMANE::Any>
    queryStatistic(BuildId, const std::vector<std::string> & names) const;

    void clearStatistic(BuildId, const std::vector<std::string> & names) const;

    std::map<std::string,std::pair<StatisticTableLabels,StatisticTableValues>>
      queryTable(BuildId, const std::vector<std::string> & names) const;

    void clearTable(BuildId, const std::vector<std::string> & names) const;

    StatisticManifest getStatisticManifest(BuildId id ) const;

    StatisticTableManifest getTableManifest(BuildId id ) const;


  protected:
    StatisticService() = default;

  private:
    using StatisticStore = 
      std::map<std::string,std::pair<std::unique_ptr<Statistic>,StatisticInfo>>;

    using BuildIdStatisticStore = std::map<BuildId,StatisticStore>;
    BuildIdStatisticStore buildIdStatisticStore_;

    using TableStore = 
      std::map<std::string,std::tuple<std::unique_ptr<StatisticTablePublisher>,
                                      StatisticTableInfo,
                                      std::function<void(StatisticTablePublisher *)>>>;

    using BuildIdTableStore = std::map<BuildId,TableStore>;
    BuildIdTableStore buildIdTableStore_;
  };

  using StatisticServiceSingleton = StatisticService;
}

#endif // EMANESTATISTICSERVICE_HEADER_
