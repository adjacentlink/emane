/*
 * Copyright (c) 2013,2017 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2010 - DRS CenGen, LLC, Columbia, Maryland
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

#include "eelloaderfadingselection.h"
#include "emane/events/fadingselectionevent.h"
#include "emane/utils/parameterconvert.h"
#include "emane/generators/eel/formatexception.h"

#include <vector>
#include <sstream>
#include <cstring>

EMANE::Generators::EEL::LoaderFadingSelection::LoaderFadingSelection()
{}

EMANE::Generators::EEL::LoaderFadingSelection::~LoaderFadingSelection()
{}

void EMANE::Generators::EEL::LoaderFadingSelection::load(const ModuleType & moduleType,
                                                         const ModuleId   & moduleId,
                                                         const EventType  & ,
                                                         const InputArguments & args)
{
  if(moduleType == "nem")
    {
      if(args.size() < 1)
        {
          throw FormatException("LoaderFadingSelection expects at least 1 argument");
        }
      else
        {
          InputArguments::const_iterator iterArg = args.begin();

          // parse the individual fading selection entry params
          for(; iterArg != args.end(); ++iterArg)
            {
              InputArguments  params;

              size_t posStart = 0;
              size_t posEnd = 0;

              // build a parameters vector holding all the
              // comma separeted elements
              // <txModuleID>,none|nakagami|lognormal ...
              while(posEnd != std::string::npos)
                {
                  posEnd = iterArg->find_first_of(",",posStart);

                  params.push_back(iterArg->substr(posStart, posEnd - posStart));

                  posStart = posEnd + 1;
                }

              // you must at specify a tx nem and fading model
              if(params.size() != 2)
                {
                  throw FormatException("LoaderFadingSelection expects 2 parameters");
                }
              else
                {
                  NEMId targetNEM{moduleId};
                  NEMId txNEM{};
                  Events::FadingModel model{};

                  // convert the strings into the appropriate types
                  for(size_t i = 0; i < params.size(); ++i)
                    {
                      try
                        {
                          switch(i)
                            {
                            case 0:
                              // <txModuleID> which must be nem:UINT16
                              {
                                size_t pos = params[i].find(':');

                                if(pos != std::string::npos)
                                  {
                                    if(params[i].substr(0,pos) == "nem")
                                      {
                                        txNEM = Utils::ParameterConvert(params[i].substr(pos + 1)).toUINT16();
                                      }
                                    else
                                      {
                                        std::stringstream sstream;
                                        throw FormatException("LoaderFadingSelection only supports 'nem' module type");
                                      }
                                  }
                              }
                              break;

                            case 1:
                              {
                                std::string sModel{params[i]};

                                if(sModel == "none")
                                  {
                                    model = Events::FadingModel::NONE;
                                  }
                                else if(sModel == "nakagami")
                                  {
                                    model = Events::FadingModel::NAKAGAMI;
                                  }
                                else if(sModel == "lognormal")
                                  {
                                    model = Events::FadingModel::LOGNORMAL;
                                  }
                                else
                                  {
                                    throw makeException<FormatException>("LoaderFadingSelection loader unknown fading model: %s",
                                                                         sModel.c_str());
                                  }
                              }
                              break;

                            default:
                              throw FormatException("LoaderFadingSelection loader too many parameters");
                              break;
                            }
                        }
                      catch(Utils::ParameterConvert::ConversionException & exp)
                        {
                          std::stringstream sstream;
                          sstream<<"LoaderFadingSelection loader: Parameter conversion error. "<<exp.what()<<std::ends;
                          throw FormatException(sstream.str());
                        }
                    }

                  // load the full  cache
                  loadFadingSelectionCache(targetNEM,txNEM,model,fadingSelectionEntryCache_);

                  // load the delta cache
                  loadFadingSelectionCache(targetNEM,txNEM,model,fadingSelectionDeltaEntryCache_);
                }
            }
        }
    }
}

EMANE::Generators::EEL::EventInfoList EMANE::Generators::EEL::LoaderFadingSelection::getEvents(EventPublishMode mode)
{
  EventInfoList eventInfoList;

  FadingSelectionEntryCache * pCache = 0;

  if(mode == DELTA)
    {
      // in DELTA mode events *only* contain entries that have
      // changes since the last update
      pCache = &fadingSelectionDeltaEntryCache_;
    }
  else
    {
      // in FULL mode events contain all the entries regardless
      // of whether they contain updated data since the last
      // getEvents() call
      pCache = &fadingSelectionEntryCache_;
    }

  Events::FadingSelections fadingSelections;

  FadingSelectionEntryCache::iterator iter = pCache->begin();

  for(;iter != pCache->end(); ++iter)
    {
      FadingSelectionEntryMap::iterator iterEntry = iter->second.begin();

      for(; iterEntry !=  iter->second.end(); ++iterEntry)
        {
          fadingSelections.push_back(iterEntry->second);
        }

      if(!fadingSelections.empty())
        {
          eventInfoList.push_back({iter->first,
                Events::FadingSelectionEvent::IDENTIFIER,
                Events::FadingSelectionEvent(fadingSelections).serialize()});
        }

      fadingSelections.clear();
    }

  fadingSelectionDeltaEntryCache_.clear();

  return eventInfoList;
}

void EMANE::Generators::EEL::LoaderFadingSelection::loadFadingSelectionCache(NEMId targetNEM,
                                                                             NEMId txNEM,
                                                                             Events::FadingModel model,
                                                                             FadingSelectionEntryCache & cache)
{
  Events::FadingSelection fadingSelection{txNEM,model};

  FadingSelectionEntryCache::iterator iter = cache.end();

  if((iter = cache.find(targetNEM)) !=  cache.end())
    {
      std::pair<FadingSelectionEntryMap::iterator,bool> ret =
        iter->second.insert(std::make_pair(txNEM,fadingSelection));

      if(!ret.second)
        {
          ret.first->second = fadingSelection;
        }
    }
  else
    {
      FadingSelectionEntryMap entryMap;
      entryMap.insert(std::make_pair(txNEM,fadingSelection));
      cache.insert(std::make_pair(targetNEM,entryMap));
    }
}

DECLARE_EEL_LOADER_PLUGIN(EMANE::Generators::EEL::LoaderFadingSelection)
