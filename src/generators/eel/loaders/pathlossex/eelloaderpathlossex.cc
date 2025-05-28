/*
 * Copyright (c) 2025 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "eelloaderpathlossex.h"
#include "emane/events/pathlossexevent.h"
#include "emane/utils/parameterconvert.h"
#include "emane/generators/eel/formatexception.h"

#include <vector>
#include <sstream>
#include <cstring>

EMANE::Generators::EEL::LoaderPathlossEx::LoaderPathlossEx()
{}

EMANE::Generators::EEL::LoaderPathlossEx::~LoaderPathlossEx()
{}

void
EMANE::Generators::EEL::LoaderPathlossEx::load(const ModuleType & moduleType,
                                               const ModuleId   & moduleId,
                                               const EventType  & ,
                                               const InputArguments & args)
{
  // Specifies one or more pathloss values to use at one or more
  // DestId for messages received from SrcId at the given transmit
  // frequency. Frequency 0 is the default. If present, the associated
  // pathloss will be used when no exact frequency match is found.
  //
  // <time> nem:<SrcId> pathlossex
  //    [nem:<DestId>[,<frequencyHz>:<pathlossdB>[:<reversePathlossdB>]]+]+
  //
  if(moduleType == "nem")
    {
      if(args.size() < 1)
        {
          throw FormatException("LoaderPathlossEx expects at least 1 argument");
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

                  params.push_back(iterArg->substr(posStart, posEnd - posStart));

                  posStart = posEnd + 1;
                }

              if(params.size() < 1)
                {
                  throw FormatException("LoaderPathlossEx expects at least 1 parameter");
                }
              else
                {
                  NEMId srcNEM{moduleId};
                  NEMId dstNEM{};
                  Events::PathlossEx::FrequencyPathlossMap frequencyPathlossMap_;

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
                                size_t posEnd = params[i].find(':');

                                if(posEnd != std::string::npos)
                                  {
                                    if(params[i].substr(0,posEnd) == "nem")
                                      {
                                        dstNEM =
                                          Utils::ParameterConvert(params[i].substr(posEnd + 1)).toUINT16();
                                      }
                                    else
                                      {
                                        std::stringstream sstream;
                                        throw FormatException("LoaderPathlossEx only supports 'nem' module type");
                                      }
                                  }
                              }
                              break;

                            default:
                              {
                                InputArguments pathlossParams;
                                size_t posStart = 0;
                                size_t posEnd = 0;

                                while(posEnd != std::string::npos)
                                  {
                                    posEnd = params[i].find_first_of(":",posStart);

                                    pathlossParams.push_back(params[i].substr(posStart, posEnd - posStart));
                                    posStart = posEnd + 1;
                                  }

                                if(pathlossParams.size() == 2 || pathlossParams.size() == 3)
                                  {
                                    std::uint64_t u64FrequencyHz = Utils::ParameterConvert(pathlossParams[0]).toUINT64();

                                    float fForwardPathlossdB = Utils::ParameterConvert(pathlossParams[1]).toFloat();
                                    float fReversePathlossdB{};

                                    if(pathlossParams.size() == 3)
                                      {
                                        fReversePathlossdB = Utils::ParameterConvert(pathlossParams[2]).toFloat();
                                      }
                                    else
                                      {
                                        fReversePathlossdB = fForwardPathlossdB;
                                      }

                                    // load the full pathloss cache
                                    loadPathlossExCache(dstNEM,
                                                        srcNEM,
                                                        u64FrequencyHz,
                                                        fForwardPathlossdB,
                                                        fReversePathlossdB,
                                                        pathlossEntryCache_);

                                    // load the delta update cache
                                    loadPathlossExCache(dstNEM,
                                                        srcNEM,
                                                        u64FrequencyHz,
                                                        fForwardPathlossdB,
                                                        fReversePathlossdB,
                                                        pathlossDeltaEntryCache_);
                                  }
                                else
                                  {
                                    throw FormatException("LoaderPathlossEx loader malformed parameters");
                                  }

                                break;
                              }
                            }
                        }
                      catch(Utils::ParameterConvert::ConversionException & exp)
                        {
                          std::stringstream sstream;
                          sstream<<"LoaderPathlossEx loader: Parameter conversion error. "<<exp.what()<<std::ends;
                          throw FormatException(sstream.str());
                        }
                    }
                }
            }
        }
    }
}

EMANE::Generators::EEL::EventInfoList
EMANE::Generators::EEL::LoaderPathlossEx::getEvents(EventPublishMode mode)
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

  Events::PathlossExs pathlossExs{};

  for(const auto & entry : *pCache)
    {
      NEMId dstNEM{entry.first};
      const auto & pathlossEntryMap{entry.second};

      for(const auto & entry : pathlossEntryMap)
        {
          NEMId srcNEM{entry.first};
          const auto & frequencyPathlossMap{entry.second};

          pathlossExs.push_back(Events::PathlossEx{srcNEM,frequencyPathlossMap});
        }

      if(!pathlossExs.empty())
        {
          eventInfoList.push_back({dstNEM,
              Events::PathlossExEvent::IDENTIFIER,
              Events::PathlossExEvent(pathlossExs).serialize()});
        }

      pathlossExs.clear();
    }

  pathlossDeltaEntryCache_.clear();

  return eventInfoList;
}

void EMANE::Generators::EEL::LoaderPathlossEx::loadPathlossExCache(NEMId dstNEM,
                                                                   NEMId srcNEM,
                                                                   std::uint64_t u64FrequencyHz,
                                                                   float fForwardPathlossdB,
                                                                   float fReversePathlossdB,
                                                                   PathlossEntryCache & cache)
{
  auto iter = cache.end();

  if((iter = cache.find(dstNEM)) == cache.end())
    {
      iter =
        cache.emplace(dstNEM,
                      PathlossEntryMap{}).first;
    }

  auto piter = iter->second.find(srcNEM);

  if(piter == iter->second.end())
    {
      piter = iter->second.emplace(srcNEM,
                                   Events::PathlossEx::FrequencyPathlossMap{}).first;
    }

  piter->second[u64FrequencyHz] = fForwardPathlossdB;

  if((iter = cache.find(srcNEM)) == cache.end())
    {
      iter =
        cache.emplace(srcNEM,
                      PathlossEntryMap{}).first;
    }

  piter = iter->second.find(dstNEM);

  if(piter == iter->second.end())
    {
      piter = iter->second.emplace(dstNEM,
                                   Events::PathlossEx::FrequencyPathlossMap{}).first;
    }

  piter->second[u64FrequencyHz] = fReversePathlossdB;
}

DECLARE_EEL_LOADER_PLUGIN(EMANE::Generators::EEL::LoaderPathlossEx)
