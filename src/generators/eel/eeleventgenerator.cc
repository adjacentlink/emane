/*
 * Copyright (c) 2013-2014,2016 - Adjacent Link LLC, Bridgewater, New
 * Jersey
 * Copyright (c) 2010-2012 - DRS CenGen, LLC, Columbia, Maryland
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
 * * Neither the name of DRS CenGen, LLC nor the names of its
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
 *
 */

#include "eeleventgenerator.h"
#include "eelinputparser.h"

#include "emane/generators/eel/formatexception.h"
#include "emane/configureexception.h"

#include "emane/utils/parameterconvert.h"

#include <sstream>
#include <fstream>

EMANE::Generators::EEL::Generator::Generator(PlatformServiceProvider *pPlatformService):
  EventGenerator(pPlatformService),
  bCancel_{}
{}

EMANE::Generators::EEL::Generator::~Generator(){}

void EMANE::Generators::EEL::Generator::initialize(Registrar & registrar)
{
  auto & configRegistrar = registrar.configurationRegistrar();

  configRegistrar.registerNonNumeric<std::string>("inputfile",
                                                  ConfigurationProperties::REQUIRED,
                                                  {},
                                                  "EEL Text input file.",
                                                  1,
                                                  1024);

  configRegistrar.registerNonNumeric<std::string>("loader",
                                                  ConfigurationProperties::REQUIRED,
                                                  {},
                                                  "EEL Loader plugin.",
                                                  1,
                                                  1024);
}

void EMANE::Generators::EEL::Generator::configure(const ConfigurationUpdate & update)
{
  for(const auto & item : update)
    {
      if(item.first == "inputfile")
        {
          for(const auto & any : item.second)
            {
              std::string sInputFile = any.asString();

              LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                      INFO_LEVEL,
                                      "EEL::Generator::start %s: %s",
                                      item.first.c_str(),
                                      sInputFile.c_str());

              inputFileNameVector_.push_back(sInputFile);
            }
        }
      else if(item.first == "loader")
        {
          try
            {
              for(const auto & any : item.second)
                {
                  std::string sLoaderPlugin = any.asString();

                  size_t pos = sLoaderPlugin.find(':');

                  if(pos != std::string::npos)
                    {
                      std::string sEventTypes = sLoaderPlugin.substr(0,pos);

                      size_t pos2 = sLoaderPlugin.find(':',pos + 1);

                      std::string sLibraryName = sLoaderPlugin.substr(pos + 1,pos2- pos -1);

                      EventPublishMode publishMode = DELTA;

                      if(pos2 != std::string::npos)
                        {
                          std::string sPublishMode = sLoaderPlugin.substr(pos2 + 1);

                          if(sPublishMode == "delta")
                            {
                              publishMode = DELTA;
                            }
                          else if(sPublishMode == "full")
                            {
                              publishMode = FULL;
                            }
                          else
                            {
                              throw makeException<ConfigureException>("EEL::Generator: Unkown 'loader' "
                                                                      "publish mode %s",
                                                                      sPublishMode.c_str());
                            }
                        }

                      LoaderPluginFactory * pPluginFactory = new LoaderPluginFactory();

                      pPluginFactory->construct("lib" + sLibraryName + ".so");

                      pluginFactoryList_.push_back(pPluginFactory);

                      std::pair<LoaderPlugin *,EventPublishMode> loaderEntry =
                        std::make_pair(pPluginFactory->createPlugin(),publishMode);

                      size_t pos1 = 0;

                      pos2 = sLoaderPlugin.find(',');

                      while(pos2 != std::string::npos)
                        {
                          std::string sEventType = sEventTypes.substr(pos1,pos2 - pos1);

                          eventPluginMap_.insert(std::make_pair(sEventType,loaderEntry));

                          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                                  INFO_LEVEL,
                                                  "EEL::Generator::configure %s: plugin %s will load %s sentences mode:%s",
                                                  item.first.c_str(),
                                                  sLoaderPlugin.c_str(),
                                                  sEventType.c_str(),
                                                  publishMode == DELTA ? "delta" : "full");
                          pos1 = pos2 + 1;

                          pos2 = sEventTypes.find(',',pos1);
                        }

                      std::string sEventType = sEventTypes.substr(pos1);

                      eventPluginMap_.insert(std::make_pair(sEventType,loaderEntry));

                      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                              INFO_LEVEL,
                                              "EEL::Generator::configure %s: plugin %s will load %s sentences mode:%s",
                                              item.first.c_str(),
                                              sLoaderPlugin.c_str(),
                                              sEventType.c_str(),
                                              publishMode == DELTA ? "delta" : "full");
                    }
                  else
                    {
                      throw makeException<ConfigureException>("EEL::Generator: Bad configuration 'loader' "
                                                              "format %s",
                                                              sLoaderPlugin.c_str());
                    }
                }
            }
          catch(Utils::FactoryException & exp)
            {
              throw makeException<ConfigureException>("EEL::Generator: Factory exception %s",
                                                      exp.what());
            }
        }
      else
        {
          throw makeException<ConfigureException>("EEL::Generator: "
                                                  "Unexpected configuration item %s",
                                                  item.first.c_str());
        }
    }
}

void EMANE::Generators::EEL::Generator::start()
{
  thread_ = std::thread{&Generator::generate,this};
}

void EMANE::Generators::EEL::Generator::stop()
{
  if(thread_.joinable())
    {
      mutex_.lock();
      bCancel_ = true;
      cond_.notify_one();
      mutex_.unlock();
      thread_.join();
    }
}

void EMANE::Generators::EEL::Generator::destroy()
  throw()
{}

void EMANE::Generators::EEL::Generator::generate()
{
  char buf[2048];
  float fCurrentTime = 0;
  float fEventTime = 0;
  unsigned long ulCurrentLine = 1;
  InputParser parser;
  InputFileNameVector::const_iterator iterFileName = inputFileNameVector_.begin();

  try
    {
      auto testStartTime =  Clock::now();

      for(; iterFileName != inputFileNameVector_.end(); ++iterFileName)
        {
          std::ifstream eelInputStream;

          eelInputStream.open(iterFileName->c_str());

          if(!eelInputStream)
            {
              std::stringstream sstream;
              sstream<<"EEL::Generator: Unable to open "<<*iterFileName<<std::ends;
              throw FormatException(sstream.str());
            }
          else
            {
              LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                     DEBUG_LEVEL,
                                     "EEL::Generator: Parsing %s",
                                     iterFileName->c_str());

              ulCurrentLine = 1;
            }

          // parse the next EEL entrty
          while(eelInputStream.getline(buf,sizeof(buf)))
            {
              std::string sEventType;
              std::string sModuleId;
              InputArguments inputArguments;

              LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(), DEBUG_LEVEL,"%s",buf);

              if(parser.parse(std::string(buf),
                              fEventTime,
                              sEventType,
                              sModuleId,
                              inputArguments))
                {
                  ModuleType sModuleType(sModuleId);
                  ModuleId u16ModuleId = 0;
                  size_t pos = sModuleId.find(':');

                  if(pos != std::string::npos)
                    {
                      sModuleType = sModuleId.substr(0,pos);

                      u16ModuleId =
                        Utils::ParameterConvert(sModuleId.substr(pos + 1)).toUINT16();
                    }

                  if(fEventTime != fCurrentTime)
                    {
                      if(!waitAndSendEvents(testStartTime,fCurrentTime))
                        {
                          return;
                        }

                      fCurrentTime = fEventTime;
                    }

                  EventPluginMap::iterator iterPlugin;

                  if((iterPlugin = eventPluginMap_.find(sEventType)) !=
                     eventPluginMap_.end())
                    {
                      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                             DEBUG_LEVEL,
                                             "Registered EEL event type '%s' at time %f for %s",
                                             sEventType.c_str(),
                                             fEventTime,
                                             sModuleId.c_str());

                      iterPlugin->second.first->load(sModuleType,
                                                     u16ModuleId,
                                                     sEventType,
                                                     inputArguments);
                    }
                  else
                    {
                      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                             DEBUG_LEVEL,
                                             "Unregistered EEL event type '%s' at time %f for %s",
                                             sEventType.c_str(),
                                             fEventTime,
                                             sModuleId.c_str());
                    }
                }

              ++ulCurrentLine;
            }
        }

      waitAndSendEvents(testStartTime,fCurrentTime);

    }
  catch(Exception & exp)
    {
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              ABORT_LEVEL,"%s (%s:%lu)",
                              exp.what(),
                              iterFileName->c_str(),
                              ulCurrentLine);
    }
}

bool EMANE::Generators::EEL::Generator::waitAndSendEvents(const TimePoint & testStartTime,
                                                          float fCurrentTime)
{
  EventInfoList currentTimeEventList;

  EventPluginMap::iterator iterPlugin = eventPluginMap_.begin();

  for(;iterPlugin != eventPluginMap_.end(); ++iterPlugin)
    {
      EventInfoList eventList =
        iterPlugin->second.first->getEvents(iterPlugin->second.second);

      currentTimeEventList.insert(currentTimeEventList.end(),
                                  eventList.begin(),
                                  eventList.end());
    }

  // time to schedule the next event publications

  if(!currentTimeEventList.empty())
    {
      std::unique_lock<std::mutex> lock{mutex_};

      std::cv_status status{};

      while(!bCancel_ && status != std::cv_status::timeout)
        {
          status = cond_.wait_until(lock,
                                    testStartTime +
                                    DoubleSeconds{fCurrentTime});
        }

      if(bCancel_)
        {
          return false;
        }

      lock.unlock();

      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                             DEBUG_LEVEL,
                             "Sending all events for time %f",
                             fCurrentTime);

      EventInfoList::iterator eventIter =
        currentTimeEventList.begin();

      for(; eventIter != currentTimeEventList.end(); ++eventIter)
        {
          pPlatformService_->eventService().sendEvent(eventIter->getNEMId(),
                                                      eventIter->getEventId(),
                                                      eventIter->getSerialization());
        }
    }

  return true;
}

DECLARE_EVENT_GENERATOR(EMANE::Generators::EEL::Generator);
