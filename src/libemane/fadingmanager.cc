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

#include "fadingmanager.h"
#include "nakagamifadingalgorithmmanager.h"

EMANE::FadingManager::FadingManager(NEMId id,
                                    PlatformServiceProvider * pPlatformService,
                                    const std::string & sPrefix):
  id_{id},
  pPlatformService_{pPlatformService},
  sPrefix_{sPrefix},
  bFading_{},
  pFadingAlgorithmManagerForAll_{}
{
  fadingAlgorithmManagers_.insert(std::make_pair("nakagami",
                                                 std::unique_ptr<FadingAlgorithmManager>(new NakagamiFadingAlgorithmManager{id,
                                                                                                                              pPlatformService,
                                                                                                                              sPrefix})));
}

void EMANE::FadingManager::initialize(Registrar & registrar)
{
  auto & configRegistrar = registrar.configurationRegistrar();

  std::string sModelDescription{"Defines the fading model:"
                                " none, event"};

  std::string sModelsRegex{"^(none|event"};

  for(const auto & entry : fadingAlgorithmManagers_)
    {
      sModelsRegex +=  "|" + entry.second->name();
      sModelDescription += ", " +  entry.second->name();
    }

  sModelsRegex += ")$";
  sModelDescription += ".";

  configRegistrar.registerNonNumeric<std::string>(sPrefix_ + "model",
                                                  EMANE::ConfigurationProperties::DEFAULT |
                                                  EMANE::ConfigurationProperties::MODIFIABLE,
                                                  {"none"},
                                                  sModelDescription,
                                                  1,
                                                  1,
                                                  sModelsRegex);
  for(const auto & entry : fadingAlgorithmManagers_)
    {
      entry.second->initialize(registrar);
    }
}

void EMANE::FadingManager::configure(const ConfigurationUpdate & update)
{
  configure_i(update,
              &FadingAlgorithmManager::configure);
}

void EMANE::FadingManager::modify(const ConfigurationUpdate & update)
{
  configure_i(update,
              &FadingAlgorithmManager::modify);
}

void EMANE::FadingManager::configure_i(const ConfigurationUpdate & update,
                                       void (FadingAlgorithmManager::*process)(const ConfigurationUpdate&))
{
  std::map<std::string,std::tuple<ConfigurationUpdate,FadingAlgorithmManager*>> configurations{};

  ConfigurationUpdate nakagamiUpdate;

  for(const auto & entry : fadingAlgorithmManagers_)
    {
      std::string sConfigName{sPrefix_ + entry.second->name() + "."};
      configurations.insert(std::make_pair(sConfigName,
                                           std::make_tuple(ConfigurationUpdate{},
                                                           entry.second.get())));
    }

  for(const auto & item : update)
    {
      if(item.first == sPrefix_ + "model")
        {
          std::string sType{item.second[0].asString()};

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::FadingManager::%s: %s = %s",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  sType.c_str());

          if(sType == "none")
            {
              bFading_ = false;
            }
          else if(sType == "event")
            {
              bFading_ = true;
              pFadingAlgorithmManagerForAll_ = nullptr;
            }
          else
            {
              bFading_ = true;

              auto iter = fadingAlgorithmManagers_.find(sType);

              if(iter != fadingAlgorithmManagers_.end())
                {
                  pFadingAlgorithmManagerForAll_ = iter->second.get();
                }
            }
        }
      else
        {
          auto pos1 = std::string::npos;
          auto pos2 = std::string::npos;

          pos1 = item.first.find_first_of('.');

          if(pos1 != std::string::npos)
            {
              pos2 = item.first.find_first_of('.',pos1+1);

              if(pos2 != std::string::npos)
                {
                  std::string sKey{item.first.substr(0,pos2+1)};

                  auto iter = configurations.find(sKey);

                  if(iter != configurations.end())
                    {
                      std::get<0>(iter->second).push_back(item);
                    }
                }
            }

          if(pos1 == std::string::npos || pos2 == std::string::npos)
            {
              throw makeException<ConfigurationException>("FrameworkPHY::FadingManager Unexpected"
                                                          " configuration item %s",
                                                          item.first.c_str());
            }
        }
    }

  for(const auto & entry : configurations)
    {
      (std::get<1>(entry.second)->*process)(std::get<0>(entry.second));

    }
}


void EMANE::FadingManager::update(const Events::FadingSelections & fadingSelections)
{
  for(const auto & selection : fadingSelections)
    {
      switch(selection.getFadingModel())
        {
        case Events::FadingModel::NONE:
          TxNEMFadingSelections_[selection.getNEMId()] = {Events::FadingModel::NONE,
                                                          nullptr};
          break;

        case Events::FadingModel::NAKAGAMI:
          TxNEMFadingSelections_[selection.getNEMId()] = {Events::FadingModel::NAKAGAMI,
                                                          fadingAlgorithmManagers_["nakagami"].get()};
          break;
        default:
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  ERROR_LEVEL,
                                  "PHYI %03hu FadingManager::%s: unknown fading model"
                                  " selection for NEM %hu",
                                  id_,
                                  __func__,
                                  selection.getNEMId());
          break;
        }
    }
}

EMANE::FadingAlgorithmStore
EMANE::FadingManager::createFadingAlgorithmStore() const
{
  FadingAlgorithmStore store{};

  for(const auto & entry : fadingAlgorithmManagers_)
    {
      store.emplace(entry.second->type(),
                    entry.second->createFadingAlgorithm());
    }

  return store;
}

std::pair<EMANE::FadingInfo,bool>
EMANE::FadingManager::getFadingSelection(NEMId nemId) const
{
  if(bFading_)
    {
      if(pFadingAlgorithmManagerForAll_)
        {
          return {FadingInfo{pFadingAlgorithmManagerForAll_->type(),
                             pFadingAlgorithmManagerForAll_->getParameters()},
                  true};
        }
      else
        {
          const auto iter = TxNEMFadingSelections_.find(nemId);

          if(iter != TxNEMFadingSelections_.end())
            {
              if(iter->second.second)
                {
                  return {FadingInfo{iter->second.first,
                                     iter->second.second->getParameters()},
                          true};
                }
              else
                {
                  // no fading manager, so no parameters
                  return {FadingInfo{iter->second.first,
                                     nullptr},
                          true};
                }
            }
          else
            {
              return {{Events::FadingModel::NONE,nullptr},false};
            }
        }
    }

  return {{Events::FadingModel::NONE,nullptr},true};
}
