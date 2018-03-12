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

#include "fadingmanager.h"
#include "nakagamifadingalgorithm.h"

EMANE::FadingManager::FadingManager(NEMId id,
                                    PlatformServiceProvider * pPlatformService,
                                    const std::string & sPrefix):
  id_{id},
  pPlatformService_{pPlatformService},
  sPrefix_{sPrefix},
  bFading_{},
  pFadingAlgorithmForAll_{}
{
  fadingModels_.insert(std::make_pair("nakagami",
                                      std::unique_ptr<FadingAlgorithm>(new NakagamiFadingAlgorithm{id,
                                            pPlatformService,
                                            sPrefix})));
}

void EMANE::FadingManager::initialize(Registrar & registrar)
{
  auto & configRegistrar = registrar.configurationRegistrar();

  std::string sModelDescription{"Defines the fading model:"
      " none, event"};

  std::string sModelsRegex{"^(none|event"};

  for(const auto & entry : fadingModels_)
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

  for(const auto & entry : fadingModels_)
    {
      entry.second->initialize(registrar);
    }

}

void EMANE::FadingManager::configure(const ConfigurationUpdate & update)
{
  configure_i(update,
              &FadingAlgorithm::configure);
}

void EMANE::FadingManager::modify(const ConfigurationUpdate & update)
{
  configure_i(update,
              &FadingAlgorithm::modify);
}

void EMANE::FadingManager::configure_i(const ConfigurationUpdate & update,
                                       void (FadingAlgorithm::*process)(const ConfigurationUpdate&))
{
  std::map<std::string,std::tuple<ConfigurationUpdate,FadingAlgorithm*>> configurations{};

  for(const auto & entry : fadingModels_)
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
              pFadingAlgorithmForAll_ = nullptr;
            }
          else
            {
              bFading_ = true;
              auto iter = fadingModels_.find(sType);

              if(iter != fadingModels_.end())
                {
                  pFadingAlgorithmForAll_ = iter->second.get();
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
      //std::get<1>(entry.second)->configure(std::get<0>(entry.second));
      (std::get<1>(entry.second)->*process)(std::get<0>(entry.second));

    }
}

std::pair<double,EMANE::FadingManager::FadingStatus>
EMANE::FadingManager::calculate(NEMId txNEMId,
                                double dPowerdBm,
                                const std::pair<LocationInfo,bool> & location)
{

  double dOutPowerdBm{};
  FadingStatus status{FadingStatus::ERROR_LOCATIONINFO};

  if(!bFading_)
    {
      dOutPowerdBm = dPowerdBm;
      status = FadingStatus::SUCCESS;
    }
  else if(pFadingAlgorithmForAll_)
    {
      // only calculate if lcoation is known
      if(location.second)
        {
          dOutPowerdBm = (*pFadingAlgorithmForAll_)(dPowerdBm,
                                                    location.first.getDistanceMeters());
          status = FadingStatus::SUCCESS;
        }
    }
  else
    {
      auto iter =  TxNEMFadingSelections_.find(txNEMId);

      if(iter != TxNEMFadingSelections_.end())
        {
          if(iter->second)
            {
              // only calculate if location is known
              if(location.second)
                {
                  dOutPowerdBm = (*iter->second)(dPowerdBm,
                                                 location.first.getDistanceMeters());

                  status = FadingStatus::SUCCESS;
                }
            }
          else
            {
              // null algorithm indicates no fading in use from src
              dOutPowerdBm = dPowerdBm;

              status = FadingStatus::SUCCESS;
            }
        }
      else
        {
          // unknown fading algorithm selection for source
          status = FadingStatus::ERROR_SELECTION;
        }
    }

  return {dOutPowerdBm,status};
}

void EMANE::FadingManager::update(const Events::FadingSelections & fadingSelections)
{
  for(const auto & selection : fadingSelections)
    {
      switch(selection.getFadingModel())
        {
        case Events::FadingModel::NONE:
          TxNEMFadingSelections_[selection.getNEMId()] = nullptr;
          break;
        case Events::FadingModel::NAKAGAMI:
          TxNEMFadingSelections_[selection.getNEMId()] = fadingModels_["nakagami"].get();
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
