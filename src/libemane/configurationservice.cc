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

#include "configurationservice.h"
#include "emane/registrarexception.h"

#include <pcre.h>
#include <ace/Guard_T.h>

EMANE::ConfigurationService::ConfigurationService(){}

void EMANE::ConfigurationService::registerRunningStateMutable(BuildId buildId,
                                                              RunningStateMutable * pRunningStateMutable)
{
  std::lock_guard<std::mutex> m(mutex_);
  
  runningStateMutables_.insert(std::make_pair(buildId,pRunningStateMutable));
}

void EMANE::ConfigurationService::registerNumericAny(BuildId buildId,
                                                     const std::string & sName,
                                                     Any::Type type,
                                                     const ConfigurationProperties & properties,
                                                     const std::vector<Any> & values,
                                                     const std::string & sUsage,
                                                     const Any & minValue,
                                                     const Any & maxValue,
                                                     std::size_t minOccurs,
                                                     std::size_t maxOccurs,
                                                     const std::string & sRegexPattern)
{
  registerAny(buildId,
              sName,
              ConfigurationInfo{sName,
                  type,
                  properties,
                  values,
                  sUsage,
                  minValue,
                  maxValue,
                  minOccurs,
                  maxOccurs,
                  sRegexPattern});
}

void EMANE::ConfigurationService::registerNonNumericAny(BuildId buildId,
                                                        const std::string & sName,
                                                        Any::Type type,
                                                        const ConfigurationProperties & properties,
                                                        const std::vector<Any> & values,
                                                        const std::string & sUsage,
                                                        std::size_t minOccurs,
                                                        std::size_t maxOccurs,
                                                        const std::string & sRegexPattern)
{
  registerAny(buildId,
              sName,
              ConfigurationInfo{sName,
                  type,
                  properties,
                  values,
                  sUsage,
                  minOccurs,
                  maxOccurs,
                  sRegexPattern});
}

void EMANE::ConfigurationService::registerAny(BuildId buildId,
                                              const std::string & sName,
                                              ConfigurationInfo && configurationInfo)
{
  std::lock_guard<std::mutex> m(mutex_);

  if(std::find_if_not(sName.begin(),sName.end(),[](int ch){return isalnum(ch) || ch == '.';}) != sName.end())
    {
      throw makeException<RegistrarException>("Invalid charater in the configuration name: %s",
                                              sName.c_str());
    }

  // check the regex is valid if defined
  if(!configurationInfo.getRegexPattern().empty())
    {
      pcre * pPCRE{};
      const char * pError{};
      int iErrorOffset{};

      pPCRE = pcre_compile(configurationInfo.getRegexPattern().c_str(),
                           0,
                           &pError,
                           &iErrorOffset,
                           0);
      
      if(!pPCRE)
        {
          throw makeException<RegistrarException>("Bad regex pattern defined for %s: %s (offset:%i) %s",
                                                  sName.c_str(),
                                                  configurationInfo.getRegexPattern().c_str(),
                                                  iErrorOffset,
                                                  pError);
        }
      else
        {
          pcre_free(pPCRE);
        }
      
    }
  
  auto iter = buildIdConfigurationStore_.find(buildId);
  
  if(iter !=  buildIdConfigurationStore_.end())
    {
      if(!iter->second.insert(std::make_pair(sName,configurationInfo)).second)
        {
          throw makeException<RegistrarException>("Duplicate configuration name registration detected: %s",
                                                  sName.c_str());
        }
    }
  else
    {
      ConfigurationStore store;

      store.insert(std::make_pair(sName,configurationInfo));
      
      buildIdConfigurationStore_.insert(std::make_pair(buildId,std::move(store)));
    }
}


 
EMANE::ConfigurationManifest
EMANE::ConfigurationService::getConfigurationManifest(BuildId buildId) const
{
  std::lock_guard<std::mutex> m(mutex_);
  
  ConfigurationManifest infos;

  auto iter = buildIdConfigurationStore_.find(buildId);
  
  if(iter !=  buildIdConfigurationStore_.end())
    {
      auto & store = iter->second;
      
      std::transform(store.begin(), 
                     store.end(), 
                     std::back_inserter(infos),
                     std::bind(&ConfigurationStore::value_type::second,std::placeholders::_1));
    }
  
  return infos;
}

std::vector<std::pair<std::string,std::vector<EMANE::Any>>>
  EMANE::ConfigurationService::queryConfiguration(BuildId buildId,
                                                  const std::vector<std::string> & names) const
{
  std::lock_guard<std::mutex> m(mutex_);
  
  std::vector<std::pair<std::string,std::vector<Any>>> values;

  auto iter = buildIdConfigurationStore_.find(buildId);
  
  if(iter !=  buildIdConfigurationStore_.end())
    {
      auto & store = iter->second;
      
      if(names.empty())
        {
          for_each(store.begin(), 
                   store.end(), 
                   bind([&values](const ConfigurationInfo & info)
                        {
                          values.push_back(std::make_pair(info.getName(),info.getValues()));
                        },
                        bind(&ConfigurationStore::value_type::second,
                             std::placeholders::_1)));
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
                         values.push_back(std::make_pair(s,iter->second.getValues()));
                       }
                     else
                       {
                         throw makeException<RegistrarException>("Unknown configuration parameter: %s",
                                                                 s.c_str());
                       }
                   });
        }
    }

  return values;
}


EMANE::ConfigurationUpdate
EMANE::ConfigurationService::buildUpdates(BuildId buildId,
                                          const ConfigurationUpdateRequest & parameters)
{
  std::lock_guard<std::mutex> m(mutex_);
  
  ConfigurationUpdate updates;

  auto iter = buildIdConfigurationStore_.find(buildId);
  
  if(iter !=  buildIdConfigurationStore_.end())
    {
      auto & store = iter->second;
      
      for(const auto & paramIter : parameters)
        {
          auto infoIter = store.find(paramIter.first);
          
          if(infoIter != store.end())
            {
              if(paramIter.second.size() >= infoIter->second.getMinOccurs() &&
                 paramIter.second.size() <= infoIter->second.getMaxOccurs())
                {
                  pcre * pPCRE{};
                  const char * pError{};
                  int iErrorOffset{};
                  
                  // if regex defined, verify a match
                  const std::string & sRegexPattern = infoIter->second.getRegexPattern();
                  
                  if(!sRegexPattern.empty())
                    {
                      pPCRE = pcre_compile(sRegexPattern.c_str(),
                                           0,
                                           &pError,
                                           &iErrorOffset,
                                           0);
                    }
      
                  std::vector<Any> anys;

                  for(const auto & value : paramIter.second)
                    {
                      try
                        {
                          auto anyType = infoIter->second.getType();
                          
                          Any any{Any::create(value,anyType)};
                          
                          // if numeric any verify range
                          if(anyType != Any::Type::TYPE_INET_ADDR &&
                             anyType != Any::Type::TYPE_STRING)
                            {
                              if(any < infoIter->second.getMinValue() ||
                                 any > infoIter->second.getMaxValue())
                                {
                                  throw makeException<ConfigurationException>("Out of range %s set to %s [%s,%s]",
                                                                              paramIter.first.c_str(),
                                                                              any.toString().c_str(),
                                                                              infoIter->second.getMinValue().toString().c_str(),
                                                                              infoIter->second.getMaxValue().toString().c_str());
                                }
                            }
                          
                          if(pPCRE)
                            {
                              if(pcre_exec(pPCRE,
                                           nullptr,
                                           value.c_str(),
                                           value.size(),
                                           0,
                                           0,
                                           0,
                                           0) < 0)
                                {
                                  throw makeException<ConfigurationException>("Regular expression mismatch %s set to %s (%s)",
                                                                              paramIter.first.c_str(),
                                                                              value.c_str(),
                                                                              sRegexPattern.c_str());
                                }
                            }
                          
                          anys.push_back(any);
                        }
                      catch(AnyException & exp)
                        {
                          throw makeException<ConfigurationException>("%s Parameter %s set to %s",
                                                                      exp.what(),
                                                                      paramIter.first.c_str(),
                                                                      value.c_str());
                        }
                    }

                  updates.push_back(std::make_pair(paramIter.first,anys));

                  if(pPCRE)
                    {
                      pcre_free(pPCRE);
                    }
                }
              else
                {
                  throw makeException<ConfigurationException>("Value occurrence out of range %s has %zu values [%zu,%zu]",
                                                              paramIter.first.c_str(),
                                                              paramIter.second.size(),
                                                              infoIter->second.getMinOccurs(),
                                                              infoIter->second.getMaxOccurs());
                                               
                }
            }
          else
            {
              throw makeException<ConfigurationException>("Parameter not registered %s",
                                                          paramIter.first.c_str());
            }
        }

      // determine if any missing item is required or has a default
      for(const auto & iter : store)
        {
          const auto & sParamName(iter.first);
          const auto & item(iter.second);
          
          if(item.isRequired() || item.hasDefault())
            {
              if(std::find_if(updates.begin(),
                              updates.end(),
                              std::bind(std::equal_to<std::string>(),
                                        std::bind(&ConfigurationUpdate::value_type::first,
                                                  std::placeholders::_1),
                                        sParamName)) == updates.end())
                {
                  if(item.isRequired())
                    {
                      throw makeException<ConfigurationException>("Required item not present: %s",
                                                                  sParamName.c_str()); 
                    }
                  else
                    {
                      // item has a defualt value, so use it
                      updates.push_back(std::make_pair(sParamName,item.getValues()));
                    }
                }
            }
        }


      // run any registered validators
      auto validatorIter = validatorStore_.find(buildId);

      if(validatorIter != validatorStore_.end())
        {
          for(auto validator : validatorIter->second)
            {
              auto ret = validator(updates);

              if(!ret.second)
                {
                  throw makeException<ConfigurationException>("Validator failure: %s",ret.first.c_str());
                }
            }
        }

      
      // update cache of current configuration values - we know the parameter 
      // is present, we would have thrown an
      std::for_each(updates.begin(),
               updates.end(),
               [&store](const ConfigurationUpdate::value_type & v)
               {
                 store.find(v.first)->second.setValues(v.second);
               });

    }
  else
    {
      if(!parameters.empty())
        {
          std::string sUnexpected{};
          
          for(const auto & parameter : parameters)
            {
              sUnexpected.append(parameter.first + " ");
            }
          
          throw makeException<ConfigurationException>("Parameter(s) not registered: %s",
                                                      sUnexpected.c_str());
        }
    }
  return updates;
}

void EMANE::ConfigurationService::update(BuildId buildId,
                                         const ConfigurationUpdate & updates)
{
  std::lock_guard<std::mutex> m(mutex_);
  
  auto iter = buildIdConfigurationStore_.find(buildId);
  
  if(iter !=  buildIdConfigurationStore_.end())
    {
      auto & store = iter->second;
      
      for(const auto & update : updates)
        {
          auto infoIter = store.find(update.first);
          
          if(infoIter != store.end())
            {
              // verify parameter is allowed to be updated
              if(!infoIter->second.isModifiable())
                {
                  throw makeException<ConfigurationException>("Parameter not running state modifiable %s",
                                                              update.first.c_str());
                }

              
              if(update.second.size() >= infoIter->second.getMinOccurs() &&
                 update.second.size() <= infoIter->second.getMaxOccurs())
                {
                  pcre * pPCRE{};
                  const char * pError{};
                  int iErrorOffset{};
                  
                  // if regex defined, verify a match
                  const std::string & sRegexPattern = infoIter->second.getRegexPattern();
                  
                  if(!sRegexPattern.empty())
                    {
                      pPCRE = pcre_compile(sRegexPattern.c_str(),
                                           0,
                                           &pError,
                                           &iErrorOffset,
                                           0);
                    }
      
                  std::vector<Any> anys;

                  for(const auto & value : update.second)
                    {
                      auto anyType = infoIter->second.getType();
                      
                      if(anyType!= value.getType())
                        {
                          throw makeException<ConfigurationException>("Parameter value type incorrect %s",
                                                                      update.first.c_str());
                        }

                      // if numeric any verify range
                      if(anyType != Any::Type::TYPE_INET_ADDR &&
                         anyType != Any::Type::TYPE_STRING)
                        {
                          if(value < infoIter->second.getMinValue() ||
                             value > infoIter->second.getMaxValue())
                            {
                              throw makeException<ConfigurationException>("Out of range %s set to %s [%s,%s]",
                                                                          update.first.c_str(),
                                                                          value.toString().c_str(),
                                                                          infoIter->second.getMinValue().toString().c_str(),
                                                                          infoIter->second.getMaxValue().toString().c_str());
                            }
                        }
                      
                      if(pPCRE)
                        {
                          std::string sValue{value.toString()};
                          
                          if(pcre_exec(pPCRE,
                                       nullptr,
                                       sValue.c_str(),
                                       sValue.size(),
                                       0,
                                       0,
                                       0,
                                       0) < 0)
                            {
                              throw makeException<ConfigurationException>("Regular expression mismatch %s set to %s (%s)",
                                                                          update.first.c_str(),
                                                                          sValue.c_str(),
                                                                          sRegexPattern.c_str());
                            }
                        }
                      
                    }

                  if(pPCRE)
                    {
                      pcre_free(pPCRE);
                    }
                }
              else
                {
                  throw makeException<ConfigurationException>("Value occurrence out of range %s has %zu values [%zu,%zu]",
                                                              update.first.c_str(),
                                                              update.second.size(),
                                                              infoIter->second.getMinOccurs(),
                                                              infoIter->second.getMaxOccurs());
                                               
                }
            }
          else
            {
              throw makeException<ConfigurationException>("Parameter not registered %s",
                                                          update.first.c_str());
            }
        }


      auto validatorIter = validatorStore_.find(buildId);

      if(validatorIter != validatorStore_.end())
        {
          auto canidateUpdates = updates;
      
          for(const auto & iter : store)
            {
              const auto & sParamName(iter.first);
              const auto & item(iter.second);

              const auto & values = item.getValues();

              if(!values.empty())
                {
                  if(std::find_if(canidateUpdates.begin(),
                                  canidateUpdates.end(),
                                  std::bind(std::equal_to<std::string>(),
                                            std::bind(&ConfigurationUpdate::value_type::first,
                                                      std::placeholders::_1),
                                            sParamName)) == canidateUpdates.end())
                    {
                      // item has a defualt value, so use it
                      canidateUpdates.push_back(std::make_pair(sParamName,item.getValues()));
                    }
                }
            }
    
          for(auto validator : validatorIter->second)
            {
              auto ret = validator(canidateUpdates);

              if(!ret.second)
                {
                  throw makeException<ConfigurationException>("Validator failure: %s",ret.first.c_str());
                }
            }
        }
      
      auto mutablesIter = runningStateMutables_.find(buildId);

      if(mutablesIter != runningStateMutables_.end())
        {
          // update cache of current configuration values - we know the parameter 
          // is present, we would have thrown an exception
          std::for_each(updates.begin(),
                        updates.end(),
                        [&store](const ConfigurationUpdate::value_type & v)
                        {
                          store.find(v.first)->second.setValues(v.second);
                        });
          
          mutablesIter->second->processConfiguration(updates);
        }
      else
        {
          throw makeException<ConfigurationException>("No component registered with build id %hu",buildId);
        }
      
    }
}

void EMANE::ConfigurationService::registerValidator(BuildId buildId, ConfigurationValidator validator)
{
  std::lock_guard<std::mutex> m(mutex_);

  auto iter = validatorStore_.find(buildId);

  if(iter != validatorStore_.end())
    {
      iter->second.push_back(validator);
    }
  else
    {
      validatorStore_.insert(std::make_pair(buildId,std::vector<ConfigurationValidator>{validator}));
    }
}
