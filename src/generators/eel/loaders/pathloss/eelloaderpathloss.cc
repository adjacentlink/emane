/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "eelloaderpathloss.h"
#include "emane/events/pathlossevent.h"
#include "emane/utils/parameterconvert.h"
#include "emane/generators/eel/formatexception.h"

#include <vector>
#include <sstream>
#include <cstring>

EMANE::Generators::EEL::LoaderPathloss::LoaderPathloss()
{}
    
EMANE::Generators::EEL::LoaderPathloss::~LoaderPathloss()
{}

void EMANE::Generators::EEL::LoaderPathloss::load(const ModuleType & moduleType, 
                                                  const ModuleId   & moduleId, 
                                                  const EventType  & ,
                                                  const InputArguments & args)
{
  if(moduleType == "nem")
    {
      if(args.size() < 1)
        {
          throw FormatException("LoaderPathloss expects at least 1 argument");
        }
      else
        {
          InputArguments::const_iterator iterArg = args.begin();
          
          // parse the individual pathloss entry params
          for(; iterArg != args.end(); ++iterArg)
            {
              InputArguments  params;
              
              size_t posStart = 0;
              size_t posEnd = 0;
              
              // build a parameters vector holding all the
              // comma separeted elements
              // <dstModuleID>,<dB>[,dBrev] ...
              while(posEnd != std::string::npos)
                {
                  posEnd = iterArg->find_first_of(",",posStart);
                  
                  params.push_back( iterArg->substr(posStart, posEnd - posStart));
                  
                  posStart = posEnd + 1;
                }
              
              // you must at least specify a destination and pathloss
              if(params.size() < 2)
                {
                  throw FormatException("LoaderPathloss expects at least 1 parameter");
                }
              else
                {
                  NEMId srcNEM{moduleId};
                  NEMId dstNEM{};
                  float fReversePathlossdb{};
                  float fForwardPathlossdb{};
                  
                  // convert the strings into the appropriate types
                  for(size_t i = 0; i < params.size(); ++i)
                    {
                      try
                        {
                          switch(i)
                            {
                            case 0:
                              // <dstModuleID> which must be nem:UINT16
                              {
                                size_t pos = params[i].find(':');
                                
                                if(pos != std::string::npos)
                                  {
                                    if(params[i].substr(0,pos) == "nem")
                                      {
                                        dstNEM = Utils::ParameterConvert(params[i].substr(pos + 1)).toUINT16();
                                      }
                                    else
                                      {
                                        std::stringstream sstream;
                                        throw FormatException("LoaderPathloss only supports 'nem' module type");
                                      }
                                  }
                              }
                              break;
                              
                            case 1:
                              fForwardPathlossdb = Utils::ParameterConvert(params[i]).toFloat();

                              // store the value as the reverse pathloss as well
                              // if reverse pathloss is given it will ovewrite this value
                              fReversePathlossdb = fForwardPathlossdb;
                              break;

                            case 2:
                              fReversePathlossdb = Utils::ParameterConvert(params[i]).toFloat();
                              break;
                              
                            default:
                              throw FormatException("LoaderPathloss loader too many parameters");
                              break;
                            }
                        }
                      catch(Utils::ParameterConvert::ConversionException & exp)
                        {
                          std::stringstream sstream;
                          sstream<<"LoaderPathloss loader: Parameter conversion error. "<<exp.what()<<std::ends;
                          throw FormatException(sstream.str());
                        }
                    }

                  // load the full pathloss cache
                  loadPathlossCache(dstNEM,srcNEM,fForwardPathlossdb,fReversePathlossdb,pathlossEntryCache_);

                  // load the delta update cache
                  loadPathlossCache(dstNEM,srcNEM,fForwardPathlossdb,fReversePathlossdb,pathlossDeltaEntryCache_);
                }
            }
        }
    }
}
    
EMANE::Generators::EEL::EventInfoList EMANE::Generators::EEL::LoaderPathloss::getEvents(EventPublishMode mode)
{
  EventInfoList eventInfoList;
  
  PathlossEntryCache * pCache = 0;
  
  if(mode == DELTA)
    {
      // in DELTA mode events *only* contain entries that have
      // changes since the last update
      pCache = &pathlossDeltaEntryCache_;
    }
  else
    {
      // in FULL mode events contain all the entries regardless
      // of whether they contain updated data since the last 
      // getEvents() call
      pCache = &pathlossEntryCache_;
    }
  
  Events::Pathlosses pathlosses;

  PathlossEntryCache::iterator iter = pCache->begin();
  
  for(;iter != pCache->end(); ++iter)
    {
      PathlossEntryMap::iterator iterEntry = iter->second.begin();
      
      for(; iterEntry !=  iter->second.end(); ++iterEntry)
        {
          pathlosses.push_back(iterEntry->second);
        }

      if(!pathlosses.empty())
        {
          eventInfoList.push_back({iter->first,
                Events::PathlossEvent::IDENTIFIER,
                Events::PathlossEvent(pathlosses).serialize()});
        }
      
      pathlosses.clear();
    }
  
  pathlossDeltaEntryCache_.clear();
  
  return eventInfoList;
}

void EMANE::Generators::EEL::LoaderPathloss::loadPathlossCache(NEMId dstNEM,
                                                               NEMId srcNEM,
                                                               float fForwardPathloss,
                                                               float fReversePathloss,
                                                               PathlossEntryCache & cache)
{
  // swap entry info to create a reverse pathloss entry
  Events::Pathloss pathlossForward{srcNEM,fForwardPathloss,fReversePathloss};
  Events::Pathloss pathlossReverse{dstNEM,fReversePathloss,fForwardPathloss};

  PathlossEntryCache::iterator iter = cache.end();
                  
  if((iter = cache.find(dstNEM)) !=  cache.end())
    {
      std::pair<PathlossEntryMap::iterator,bool> ret =
        iter->second.insert(std::make_pair(pathlossForward.getNEMId(),pathlossForward));
      
      if(!ret.second)
        {
          ret.first->second = pathlossForward;
        }
    }
  else
    {
      PathlossEntryMap entryMap;
      entryMap.insert(std::make_pair(pathlossForward.getNEMId(),pathlossForward));
      cache.insert(std::make_pair(dstNEM,entryMap));
    }
  
  
  if((iter = cache.find(srcNEM)) !=  cache.end())
    {
      std::pair<PathlossEntryMap::iterator,bool> ret =
        iter->second.insert(std::make_pair(pathlossReverse.getNEMId(),pathlossReverse));
      
      if(!ret.second)
        {
          ret.first->second = pathlossReverse;
        }
    }
  else
    {
      PathlossEntryMap entryMap;
      entryMap.insert(std::make_pair(pathlossReverse.getNEMId(),pathlossReverse));
      cache.insert(std::make_pair(srcNEM,entryMap));
    }
}

DECLARE_EEL_LOADER_PLUGIN(EMANE::Generators::EEL::LoaderPathloss)
