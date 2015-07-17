/*
 * Copyright (c) 2013,2015 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "emane/registrarexception.h"
#include "statisticservice.h"

#include <iterator>

void EMANE::StatisticService::registerStatistic(BuildId buildId,
                                                const std::string & sName,
                                                Any::Type type,
                                                const StatisticProperties & properties,
                                                const std::string & sDescription,
                                                Statistic * pStatistic)
{
  if(std::find_if_not(sName.begin(),sName.end(),[](int ch){return isalnum(ch) || ch == '.';}) != sName.end())
    {
      throw makeException<RegistrarException>("Invalid charater in the statistic name: %s",
                                              sName.c_str());
    }

  StatisticInfo info{sName,
      type,
      properties,
      sDescription};

  std::unique_ptr<Statistic> pStat{pStatistic};
    
  auto iter = buildIdStatisticStore_.find(buildId);
    
  if(iter != buildIdStatisticStore_.end())
    {
      auto & store = iter->second;
      
      if(!store.insert(std::make_pair(sName,std::make_pair(std::move(pStat),std::move(info)))).second)
        {
          throw makeException<RegistrarException>("Statistic already registered: %s",
                                                 sName.c_str());
        }
    }
  else
    {
      StatisticStore store;
      store.insert(std::make_pair(sName,std::make_pair(std::move(pStat),std::move(info))));
      buildIdStatisticStore_.insert(std::make_pair(buildId,std::move(store)));
    }
}

void EMANE::StatisticService::registerTable(BuildId buildId,
                                            const std::string & sName,
                                            const StatisticProperties & properties,
                                            const std::string & sDescription,
                                            StatisticTablePublisher * pStatisticTablePublisher,
                                            std::function<void(StatisticTablePublisher *)> clearFunc)
{
  if(std::find_if_not(sName.begin(),sName.end(),[](int ch){return isalnum(ch) || ch == '.';}) != sName.end())
    {
      throw makeException<RegistrarException>("Invalid charater in the statistic table name: %s",
                                              sName.c_str());
    }

  StatisticTableInfo info{sName,properties,sDescription};

  std::unique_ptr<StatisticTablePublisher> pTable{pStatisticTablePublisher};
    
  auto iter = buildIdTableStore_.find(buildId);
    
  if(iter != buildIdTableStore_.end())
    {
      auto & store = iter->second;
      
      if(!store.insert(std::make_pair(sName,std::make_tuple(std::move(pTable),std::move(info),std::move(clearFunc)))).second)
        {
          throw makeException<RegistrarException>("Statistic table already registered: %s",
                                                  sName.c_str());
        }
    }
  else
    {
      TableStore store;
      store.insert(std::make_pair(sName,std::make_tuple(std::move(pTable),std::move(info),std::move(clearFunc))));
      buildIdTableStore_.insert(std::make_pair(buildId,std::move(store)));
    }
}



std::map<std::string,EMANE::Any>
EMANE::StatisticService::queryStatistic(BuildId buildId,
                                        const std::vector<std::string> & names) const
{
  std::map<std::string,Any> values;

  auto iter = buildIdStatisticStore_.find(buildId);

  if(iter != buildIdStatisticStore_.end())
    {
      auto & store = iter->second;

      if(names.empty())
        {
          std::transform(store.begin(),
                         store.end(),
                         std::inserter(values,values.end()),
                         [](const StatisticStore::value_type & p)
                         {
                           return std::make_pair(p.first,p.second.first->asAny());
                         });

        }
      else
        {
          for_each(names.begin(),
                   names.end(),
                   [&store,&values](const std::string & s)
                   {
                     auto iter = store.find(s);

                     if(iter != store.end())
                       {
                         values.insert(std::make_pair(s,iter->second.first->asAny()));
                       }
                     else
                       {
                         throw makeException<RegistrarException>("Unknown statistic name: %s",
                                                                 s.c_str());
                       }
                   });
        }
    }

  return values;
  
}

void
EMANE::StatisticService::clearStatistic(BuildId buildId,
                                        const std::vector<std::string> & names) const
{
  auto iter = buildIdStatisticStore_.find(buildId);

  if(iter != buildIdStatisticStore_.end())
    {
      auto & store = iter->second;

      if(names.empty())
        {
          std::for_each(store.begin(),
                        store.end(),
                        [](const StatisticStore::value_type & p)
                        {
                          if(p.second.second.isClearable())
                            {
                              p.second.first->clear();
                            }
                        });

        }
      else
        {
          std::vector<Statistic *> statsToClear;
          
          // store references to all the requested statistics to clear
          // this a transactional API call so either all requested items
          // are valid names and clearable or none will be cleared
          for_each(names.begin(),
                   names.end(),
                   [&store,&statsToClear](const std::string & s)
                   {
                     auto iter = store.find(s);

                     if(iter != store.end())
                       {
                         if(iter->second.second.isClearable())
                           {
                             statsToClear.push_back(iter->second.first.get());
                           }
                         else
                           {
                             throw makeException<RegistrarException>("Statistic not clearable: %s",
                                                                     s.c_str());
                           }
                       }
                     else
                       {
                         throw makeException<RegistrarException>("Unknown statistic name: %s",
                                                                 s.c_str());
                       }
                   });

          for_each(statsToClear.begin(),
                   statsToClear.end(),
                   bind(&Statistic::clear,
                        std::placeholders::_1));
        }
    }
}

std::map<std::string,std::pair<EMANE::StatisticTableLabels,EMANE::StatisticTableValues>>
EMANE::StatisticService::queryTable(BuildId buildId,
                                    const std::vector<std::string> & names) const
{
  std::map<std::string,std::pair<StatisticTableLabels,StatisticTableValues>> values;

  auto iter = buildIdTableStore_.find(buildId);

  if(iter != buildIdTableStore_.end())
    {
      auto & store = iter->second;

      if(names.empty())
        {
          std::transform(store.begin(),
                         store.end(),
                         std::inserter(values,values.end()),
                         [](const TableStore::value_type & p)
                         {
                           auto pTable = std::get<0>(p.second).get();
                           return std::make_pair(p.first,
                                                 std::make_pair(pTable->getLabels(),
                                                                pTable->getValues()));
                         });

        }
      else
        {
          for_each(names.begin(),
                   names.end(),
                   [&store,&values](const std::string & s)
                   {
                     auto iter = store.find(s);
                     
                     if(iter != store.end())
                       {
                         auto pTable = std::get<0>(iter->second).get();
                         
                         values.insert(std::make_pair(s,
                                                      std::make_pair(pTable->getLabels(),
                                                                     pTable->getValues())));
                       }
                     else
                       {
                         throw makeException<RegistrarException>("Unknown statistic table name: %s",
                                                                 s.c_str());
                       }
                   });
        }
    }

  return values;
  
}


EMANE::StatisticManifest EMANE::StatisticService::getStatisticManifest(BuildId buildId) const
{
  StatisticManifest manifest;
  
  auto iter = buildIdStatisticStore_.find(buildId);

  if(iter != buildIdStatisticStore_.end())
    {
      std::transform(iter->second.begin(), 
                     iter->second.end(), 
                     std::back_inserter(manifest),
                     std::bind(&StatisticStore::value_type::second_type::second,
                               std::bind(&StatisticStore::value_type::second,
                                         std::placeholders::_1)));
    }

  return manifest;
}


EMANE::StatisticTableManifest EMANE::StatisticService::getTableManifest(BuildId buildId) const
{
  StatisticTableManifest manifest;
  
  auto iter = buildIdTableStore_.find(buildId);

  if(iter != buildIdTableStore_.end())
    {
      std::transform(iter->second.begin(),
                     iter->second.end(),
                     std::back_inserter(manifest),
                     std::bind([&manifest](const TableStore::value_type::second_type & t)
                               {return std::get<1>(t);},
                               std::bind(&TableStore::value_type::second,
                                         std::placeholders::_1)));
    }

  return manifest;
}

void
EMANE::StatisticService::clearTable(BuildId buildId,
                                    const std::vector<std::string> & names) const
{
  auto iter = buildIdTableStore_.find(buildId);

  if(iter != buildIdTableStore_.end())
    {
      // store references to all the requested statistics to clear
      // this a transactional API call so either all requested items
      // are valid names and clearable or none will be cleared
      std::vector<std::function<void()>> tablesToClear;
      
      auto & store = iter->second;

      if(names.empty())
        {
          std::for_each(store.begin(),
                        store.end(),
                        [&tablesToClear](const TableStore::value_type & p)
                        {
                          if(std::get<1>(p.second).isClearable())
                           {
                             tablesToClear.push_back(std::bind(std::get<2>(p.second),
                                                               std::get<0>(p.second).get()));
                           }
                        });

        }
      else
        {
          for_each(names.begin(),
                   names.end(),
                   [&store,&tablesToClear](const std::string & s)
                   {
                     auto iter = store.find(s);

                     if(iter != store.end())
                       {
                         if(std::get<1>(iter->second).isClearable())
                           {
                             tablesToClear.push_back(std::bind(std::get<2>(iter->second),
                                                               std::get<0>(iter->second).get()));
                           }
                         else
                           {
                             throw makeException<RegistrarException>("Table not clearable: %s",
                                                                     s.c_str());
                           }
                       }
                     else
                       {
                         throw makeException<RegistrarException>("Unknown table name: %s",
                                                                 s.c_str());
                       }
                   });
        }

      for_each(tablesToClear.begin(),
               tablesToClear.end(),
               [](const std::function<void()> & f)
               {f();});


    }
}
