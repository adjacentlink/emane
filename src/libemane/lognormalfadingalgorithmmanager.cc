/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that:
 *
 * (1) source code distributions retain this paragraph in its entirety,
 *
 * (2) distributions including binary code include this paragraph in
 *     its entirety in the documentation or other materials provided
 *     with the distribution.
 *
 *      "This product includes software written and developed
 *       by Code 5520 of the Naval Research Laboratory (NRL)."
 *
 *  The name of NRL, the name(s) of NRL  employee(s), or any entity
 *  of the United States Government may not be used to endorse or
 *  promote  products derived from this software, nor does the
 *  inclusion of the NRL written and developed software  directly or
 *  indirectly suggest NRL or United States  Government endorsement
 *  of this product.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "lognormalfadingalgorithmmanager.h"
#include "emane/configurationexception.h"
#include "emane/utils/conversionutils.h"

#include <map>

EMANE::LognormalFadingAlgorithmManager::
LognormalFadingAlgorithmManager(NEMId id,
                               PlatformServiceProvider * pPlatformService,
                               const std::string & sPrefix):
  FadingAlgorithmManager{"lognormal",
                         Events::FadingModel::LOGNORMAL,
                         id,
                         pPlatformService,
                         sPrefix},
  parameters_{}{parameters_.counter_=0;}

void EMANE::LognormalFadingAlgorithmManager::initialize(Registrar & registrar)
{
  auto & configRegistrar = registrar.configurationRegistrar();

  configRegistrar.registerNumeric<double>(sPrefix_ + "lognormal.dmu",
                                          EMANE::ConfigurationProperties::DEFAULT |
                                          EMANE::ConfigurationProperties::MODIFIABLE,
                                          {5.0},
                                          "Defines the lognormal fading depth mu (mean of"
					  " underlying normal distribution).");

  configRegistrar.registerNumeric<double>(sPrefix_ + "lognormal.dsigma",
                                          EMANE::ConfigurationProperties::DEFAULT |
                                          EMANE::ConfigurationProperties::MODIFIABLE,
                                          {1.0},
                                          "Defines the lognormal fading depth sigma (standard"
                                          " deviation of underlying normal distribution).");

  configRegistrar.registerNumeric<double>(sPrefix_ + "lognormal.dlthresh",
                                          EMANE::ConfigurationProperties::DEFAULT |
                                          EMANE::ConfigurationProperties::MODIFIABLE,
                                          {0.25},
                                          "Defines the lognormal fading depth lower threshold"
                                          " (below this threshold is 0% POR/100% loss).");

  configRegistrar.registerNumeric<double>(sPrefix_ + "lognormal.duthresh",
                                          EMANE::ConfigurationProperties::DEFAULT |
                                          EMANE::ConfigurationProperties::MODIFIABLE,
                                          {0.75},
                                          "Defines the lognormal fading depth upper threshold"
                                          " (above this threshold is 100% POR/0% loss).");

  configRegistrar.registerNumeric<double>(sPrefix_ + "lognormal.maxpathloss",
                                          EMANE::ConfigurationProperties::DEFAULT |
                                          EMANE::ConfigurationProperties::MODIFIABLE,
                                          {100},
                                          "Defines the pathloss value (in dBm) corresponding to the"
                                          " fading depth lower threshold (0% POR/100% loss).");

  configRegistrar.registerNumeric<double>(sPrefix_ + "lognormal.minpathloss",
                                          EMANE::ConfigurationProperties::DEFAULT |
                                          EMANE::ConfigurationProperties::MODIFIABLE,
                                          {0},
                                          "Defines the pathloss value (in dBm) corresponding to the"
                                          " fading depth upper threshold (100% POR/0% loss).");

  configRegistrar.registerNumeric<double>(sPrefix_ + "lognormal.lmean",
                                          EMANE::ConfigurationProperties::DEFAULT |
                                          EMANE::ConfigurationProperties::MODIFIABLE,
                                          {0.005},
                                          "Defines the lognormal fading length mean in seconds"
                                          " (normal distribution).",
                                          0.0);

  configRegistrar.registerNumeric<double>(sPrefix_ + "lognormal.lstddev",
                                          EMANE::ConfigurationProperties::DEFAULT |
                                          EMANE::ConfigurationProperties::MODIFIABLE,
                                          {0.001},
                                          "Defines the lognormal fading length standard deviation in"
                                          " seconds (normal distribution).",
                                          0.0);

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

/* Disable validation of parameters for efficiency, in favor of adjusting both to prevent issues with temporarily invalid situations while setting both parameters
                                      if(parameters[sPrefix_ + "lognormal.dlthresh"][0].asDouble() >
                                         parameters[sPrefix_ + "lognormal.duthresh"][0].asDouble())
                                        {
                                          return std::make_pair("lognormal.dlthresh <= lognormal.duthresh", false);
                                        }

                                      if(parameters[sPrefix_ + "lognormal.minpathloss"][0].asDouble() >=
                                         parameters[sPrefix_ + "lognormal.maxpathloss"][0].asDouble())
                                        {
                                          return std::make_pair("lognormal.minpathloss < lognormal.maxpathloss", false);
                                        }
*/

                                      return  std::make_pair("",true);
                                    });
}

void EMANE::LognormalFadingAlgorithmManager::configure(const ConfigurationUpdate & update)
{
  configure_i(update);
}

void EMANE::LognormalFadingAlgorithmManager::modify(const ConfigurationUpdate & update)
{
  configure_i(update);
}

void EMANE::LognormalFadingAlgorithmManager::configure_i(const ConfigurationUpdate & update)
{
  for(const auto & item : update)
    {
      if(item.first == sPrefix_ + "lognormal.dmu")
        {
          parameters_.dmu_ = item.second[0].asDouble();
          parameters_.counter_++;

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::LognormalFadingAlgorithmManager::%s: %s = %lf",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  parameters_.dmu_);
        }
      else if(item.first == sPrefix_ + "lognormal.dsigma")
        {
          parameters_.dsigma_ = item.second[0].asDouble();
          parameters_.counter_++;

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::LognormalFadingAlgorithmManager::%s: %s = %lf",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  parameters_.dsigma_);
        }
      else if(item.first == sPrefix_ + "lognormal.dlthresh")
        {
          parameters_.dlthresh_ = item.second[0].asDouble();
          // handle invalid parameter settings (hopefully temporary)
          if(parameters_.dlthresh_ > parameters_.duthresh_) parameters_.duthresh_ = parameters_.dlthresh_;
          parameters_.counter_++;

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::LognormalFadingAlgorithmManager::%s: %s = %lf",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  parameters_.dlthresh_);
        }
      else if(item.first == sPrefix_ + "lognormal.duthresh")
        {
          parameters_.duthresh_ = item.second[0].asDouble();
          // handle invalid parameter settings (hopefully temporary)
          if(parameters_.duthresh_ < parameters_.dlthresh_) parameters_.dlthresh_ = parameters_.duthresh_;
          parameters_.counter_++;

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::LognormalFadingAlgorithmManager::%s: %s = %lf",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  parameters_.duthresh_);
        }
      else if(item.first == sPrefix_ + "lognormal.maxpathloss")
        {
          parameters_.maxpathloss_ = item.second[0].asDouble();
          // handle invalid parameter settings (hopefully temporary)
          if(parameters_.maxpathloss_ <= parameters_.minpathloss_) parameters_.minpathloss_ = parameters_.maxpathloss_-.001;
          parameters_.counter_++;

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::LognormalFadingAlgorithmManager::%s: %s = %lf",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  parameters_.maxpathloss_);
        }
      else if(item.first == sPrefix_ + "lognormal.minpathloss")
        {
          parameters_.minpathloss_ = item.second[0].asDouble();
          // handle invalid parameter settings (hopefully temporary)
          if(parameters_.minpathloss_ >= parameters_.maxpathloss_) parameters_.maxpathloss_ = parameters_.minpathloss_+.001;
          parameters_.counter_++;

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::LognormalFadingAlgorithmManager::%s: %s = %lf",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  parameters_.minpathloss_);
        }
      else if(item.first == sPrefix_ + "lognormal.lmean")
        {
          parameters_.lmean_ = item.second[0].asDouble();
          parameters_.counter_++;

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::LognormalFadingAlgorithmManager::%s: %s = %lf",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  parameters_.lmean_);
        }
      else if(item.first == sPrefix_ + "lognormal.lstddev")
        {
          parameters_.lstddev_ = item.second[0].asDouble();
          parameters_.counter_++;

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::LognormalFadingAlgorithmManager::%s: %s = %lf",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  parameters_.lstddev_);
        }
    }
}

std::unique_ptr<EMANE::FadingAlgorithm>
EMANE::LognormalFadingAlgorithmManager::createFadingAlgorithm()
{
  return std::unique_ptr<FadingAlgorithm>{new LognormalFadingAlgorithm{id_,pPlatformService_}};
}

const void * EMANE::LognormalFadingAlgorithmManager::getParameters() const
{
  return &parameters_;
}
