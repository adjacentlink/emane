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

#include "nakagamifadingalgorithm.h"
#include "emane/configurationexception.h"
#include "emane/utils/conversionutils.h"
#include <random>
#include <map>

EMANE::NakagamiFadingAlgorithm::NakagamiFadingAlgorithm(NEMId id,
                                                        PlatformServiceProvider * pPlatformService,
                                                        const std::string & sPrefix):
  FadingAlgorithm{"nakagami",id,pPlatformService,sPrefix},
  dm0_{},
  dm1_{},
  dm2_{},
  dDistance0Meters_{},
  dDistance1Meters_{}{}

void EMANE::NakagamiFadingAlgorithm::initialize(Registrar & registrar)
{
  auto & configRegistrar = registrar.configurationRegistrar();

  configRegistrar.registerNumeric<double>(sPrefix_ + "nakagami.m0",
                                          EMANE::ConfigurationProperties::DEFAULT |
                                          EMANE::ConfigurationProperties::MODIFIABLE,
                                          {1.25},
                                          "Defines the shape factor to use for distance"
                                          " < fading.nakagami.distance0.");

  configRegistrar.registerNumeric<double>(sPrefix_ + "nakagami.m1",
                                          EMANE::ConfigurationProperties::DEFAULT |
                                          EMANE::ConfigurationProperties::MODIFIABLE,
                                          {0.75},
                                          "Defines the shape factor to use for distance"
                                          " >= fading.nakagami.distance0 and <"
                                          " fading.nakagami.distance1.");

  configRegistrar.registerNumeric<double>(sPrefix_ + "nakagami.m2",
                                          EMANE::ConfigurationProperties::DEFAULT |
                                          EMANE::ConfigurationProperties::MODIFIABLE,
                                          {0.75},
                                          "Defines the shape factor to use for distance"
                                          " >= fading.nakagami.distance1.");

  configRegistrar.registerNumeric<double>(sPrefix_ + "nakagami.distance0",
                                          EMANE::ConfigurationProperties::DEFAULT |
                                          EMANE::ConfigurationProperties::MODIFIABLE,
                                          {80},
                                          "Defines the distance in meters used for"
                                          " lower bound shape selection.");

  configRegistrar.registerNumeric<double>(sPrefix_ + "nakagami.distance1",
                                          EMANE::ConfigurationProperties::DEFAULT |
                                          EMANE::ConfigurationProperties::MODIFIABLE,
                                          {200},
                                          "Defines the distance in meters used for"
                                          " upper bound shape selection.");

  configRegistrar.registerValidator([this](const ConfigurationUpdate & update) noexcept
                                    {
                                      std::map<std::string,std::vector<Any>>  parameters;

                                      std::transform(update.begin(),
                                                     update.end(),
                                                     std::inserter(parameters,parameters.end()),
                                                     [](const ConfigurationUpdate::value_type & p)
                                                     {
                                                       return std::make_pair(p.first,p.second);
                                                     });

                                      if(parameters[sPrefix_ + "nakagami.distance0"][0].asDouble() >=
                                         parameters[sPrefix_ + "nakagami.distance1"][0].asDouble())
                                        {
                                          return std::make_pair("nakagami.distance0 < nakagami.distance1", false);
                                        }

                                      return  std::make_pair("",true);
                                    });
}

void EMANE::NakagamiFadingAlgorithm::configure(const ConfigurationUpdate & update)
{
  configure_i(update);
}

void EMANE::NakagamiFadingAlgorithm::modify(const ConfigurationUpdate & update)
{
  configure_i(update);
}

void EMANE::NakagamiFadingAlgorithm::configure_i(const ConfigurationUpdate & update)
{
  for(const auto & item : update)
    {
      if(item.first == sPrefix_ + "nakagami.m0")
        {
          dm0_ = item.second[0].asDouble();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::NakagamiFadingAlgorithm::%s: %s = %lf",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  dm0_);
        }
      else if(item.first == sPrefix_ + "nakagami.m1")
        {
          dm1_ = item.second[0].asDouble();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::NakagamiFadingAlgorithm::%s: %s = %lf",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  dm1_);
        }
      else if(item.first == sPrefix_ + "nakagami.m2")
        {
          dm2_ = item.second[0].asDouble();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::NakagamiFadingAlgorithm::%s: %s = %lf",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  dm1_);
        }
      else if(item.first == sPrefix_ + "nakagami.distance0")
        {
          dDistance0Meters_ = item.second[0].asDouble();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::NakagamiFadingAlgorithm::%s: %s = %lf",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  dDistance0Meters_);
        }
      else if(item.first == sPrefix_ + "nakagami.distance1")
        {
          dDistance1Meters_ = item.second[0].asDouble();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::NakagamiFadingAlgorithm::%s: %s = %lf",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  dDistance1Meters_);
        }
    }
}
