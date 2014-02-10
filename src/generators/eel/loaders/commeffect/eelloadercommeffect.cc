/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2012 - DRS CenGen, LLC, Columbia, Maryland
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

#include "eelloadercommeffect.h"
#include "emane/utils/parameterconvert.h"
#include "emane/events/commeffectevent.h"
#include "emane/generators/eel/formatexception.h"

#include <vector>
#include <cstring>

EMANE::Generators::EEL::LoaderCommEffect::LoaderCommEffect()
{}
    
EMANE::Generators::EEL::LoaderCommEffect::~LoaderCommEffect()
{}

void EMANE::Generators::EEL::LoaderCommEffect::load(const ModuleType & moduleType, 
                                                    const ModuleId   & moduleId, 
                                                    const EventType  & ,
                                                    const InputArguments & args)
{
  if(moduleType == "nem")
    {
      if(args.size() < 1)
        {
          throw FormatException("LoaderCommEffect expects at least 1 argument");
        }
      else
        {
          InputArguments::const_iterator iterArg = args.begin();
          
          // parse the individual commeffect entry params
          for(; iterArg != args.end(); ++iterArg)
            {
              InputArguments  params;
              
              size_t posStart = 0;
              size_t posEnd = 0;
              
              // build a parameters vector holding all the
              // comma separeted elements
              // <dstModuleID>,<latency seconds>,<jitter seconds>,
              //  <loss>,<duplicates>,<unicast bps>,<broadcast bps> ...
              while(posEnd != std::string::npos)
                {
                  posEnd = iterArg->find_first_of(",",posStart);
                  
                  params.push_back( iterArg->substr(posStart, posEnd - posStart));
                  
                  posStart = posEnd + 1;
                }
              
              // you must at least specify a destination and commeffect
              if(params.size() != 7)
                {
                  throw FormatException("LoaderCommEffect expects 7 parameters "
                                        "<dstModuleID>,<latency seconds>,<jitter seconds>, "
                                        "<loss>,<duplicates>,<unicast bps>,<broadcast bps>");
                }
              else
                {
                  NEMId srcNEM{moduleId};
                  NEMId dstNEM{};
                  float fLatencySeconds{};
                  float fJitterSeconds{};
                  float fProbabilityLoss{};
                  float fProbabilityDuplicate{};
                  std::uint64_t u64UnicastBitRate{};
                  std::uint64_t u64BroadcastBitRate{};

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
                                        dstNEM =
                                          Utils::ParameterConvert(params[i].substr(pos + 1)).toUINT16();
                                      }
                                    else
                                      {
                                        std::stringstream sstream;
                                        throw FormatException("LoaderCommEffect only supports 'nem' module type");
                                      }
                                  }
                              }
                              break;
                              
                            case 1:
                              fLatencySeconds = Utils::ParameterConvert(params[i]).toFloat();
                              break;

                            case 2:
                              fJitterSeconds= Utils::ParameterConvert(params[i]).toFloat();
                              break;

                            case 3:
                              fProbabilityLoss = Utils::ParameterConvert(params[i]).toFloat();
                              break;

                            case 4:
                              fProbabilityDuplicate = Utils::ParameterConvert(params[i]).toFloat();
                              break;
                              
                            case 5:
                              u64UnicastBitRate = Utils::ParameterConvert(params[i]).toUINT64();
                              break;
                              
                            case 6:
                              u64BroadcastBitRate = Utils::ParameterConvert(params[i]).toUINT64();
                              break;
                             
                            default:
                              throw FormatException("LoaderCommEffect loader too many parameters");
                              break;
                            }
                        }
                      catch(Utils::ParameterConvert::ConversionException & exp)
                        {
                          std::stringstream sstream;
                          sstream<<"LoaderCommEffect loader: Parameter conversion error. "<<exp.what()<<std::ends;
                          throw FormatException(sstream.str());
                        }
                    }

                  Events::CommEffect commEffect{srcNEM,
                      std::chrono::duration_cast<Microseconds>(DoubleSeconds{fLatencySeconds}),
                      std::chrono::duration_cast<Microseconds>(DoubleSeconds{fJitterSeconds}),
                      fProbabilityLoss,
                      fProbabilityDuplicate,
                      u64UnicastBitRate,
                      u64BroadcastBitRate};

                  // load the full commeffect cache
                  loadCommEffectCache(dstNEM,commEffect,commeffectEntryCache_);

                  // load the delta update cache
                  loadCommEffectCache(dstNEM,commEffect,commeffectDeltaEntryCache_);
                }
            }
        }
    }
}
    
EMANE::Generators::EEL::EventInfoList EMANE::Generators::EEL::LoaderCommEffect::getEvents(EventPublishMode mode)
{
  EventInfoList eventInfoList;
  
  CommEffectEntryCache * pCache = 0;
  
  if(mode == DELTA)
    {
      // in DELTA mode events *only* contain entries that have
      // changes since the last update
      pCache = &commeffectDeltaEntryCache_;
    }
  else
    {
      // in FULL mode events contain all the entries regardless
      // of whether they contain updated data since the last 
      // getEvents() call
      pCache = &commeffectEntryCache_;
    }
  
  Events::CommEffects commEffects;
  
  CommEffectEntryCache::iterator iter = pCache->begin();
  
  for(;iter != pCache->end(); ++iter)
    {
      CommEffectEntryMap::iterator iterEntry = iter->second.begin();
      
      for(; iterEntry !=  iter->second.end(); ++iterEntry)
        {
          commEffects.push_back(iterEntry->second);
        }

      if(!commEffects.empty())
        {
          eventInfoList.push_back({iter->first,
                Events::CommEffectEvent::IDENTIFIER,
                Events::CommEffectEvent(commEffects).serialize()});

        }
      
      commEffects.clear();
    }
  
  commeffectDeltaEntryCache_.clear();
  
  return eventInfoList;
}

void EMANE::Generators::EEL::LoaderCommEffect::loadCommEffectCache(NEMId dstNEM,
                                                                   const Events::CommEffect & commEffect,
                                                                   CommEffectEntryCache & cache)
{
  CommEffectEntryCache::iterator iter = cache.end();
                  
  if((iter = cache.find(dstNEM)) !=  cache.end())
    {
      std::pair<CommEffectEntryMap::iterator,bool> ret =
        iter->second.insert(std::make_pair(commEffect.getNEMId(),commEffect));
      
      if(!ret.second)
        {
          ret.first->second = commEffect;
        }
    }
  else
    {
      CommEffectEntryMap entryMap;
      entryMap.insert(std::make_pair(commEffect.getNEMId(),commEffect));
      cache.insert(std::make_pair(dstNEM,entryMap));
    }

}

DECLARE_EEL_LOADER_PLUGIN(EMANE::Generators::EEL::LoaderCommEffect)
