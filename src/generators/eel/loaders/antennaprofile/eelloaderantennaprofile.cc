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

#include "eelloaderantennaprofile.h"
#include "emane/events/antennaprofileevent.h"
#include "emane/utils/parameterconvert.h"
#include "emane/generators/eel/formatexception.h"

#include <sstream>
#include <cstring>

EMANE::Generators::EEL::LoaderAntennaProfile::LoaderAntennaProfile()
{}
    
EMANE::Generators::EEL::LoaderAntennaProfile::~LoaderAntennaProfile()
{}

void EMANE::Generators::EEL::LoaderAntennaProfile::load(const ModuleType & moduleType, 
                                                        const ModuleId   & moduleId, 
                                                        const EventType  & ,
                                                        const InputArguments & args)
{
  if(moduleType == "nem")
    {
      if(args.size() != 1)
        {
          throw FormatException("LoaderAntennaProfile loader expects 1 argument");
        }
      else
        {
          InputArguments  params;

          size_t posStart = 0;
          size_t posEnd = 0;

 
          
          while(posEnd != std::string::npos)
            {
              posEnd = args[0].find_first_of(",",posStart);
              
              params.push_back(args[0].substr(posStart, posEnd - posStart));

              posStart = posEnd + 1;
            }
 
          AntennaProfileId profileId{};
          double dAntennaAzimuthDegrees{};
          double dAntennaElevationDegrees{};

          try
            {
              for(size_t i = 0; i < params.size(); i++)
                {
                  switch(i)
                    {
                    case 0:
                      profileId = Utils::ParameterConvert(params[i]).toUINT16();
                      break;

                    case 1:
                      dAntennaAzimuthDegrees = Utils::ParameterConvert(params[i]).toDouble();
                      break;

                    case 2:
                      dAntennaElevationDegrees = Utils::ParameterConvert(params[i]).toDouble();
                      break;
                          
                    default:
                      throw FormatException("LoaderAntennaProfile too many arguments");
                    }
                }
            }
          catch(Utils::ParameterConvert::ConversionException & exp)
            {
              std::stringstream sstream;
              sstream<<"EELEventGenerator: Parameter conversion error. "<<exp.what()<<std::ends;
              throw FormatException(sstream.str());
            }

          Events::AntennaProfile antennaProfile{moduleId,
              profileId,
              dAntennaAzimuthDegrees,
              dAntennaElevationDegrees};


          // update the full cache
          std::pair<AntennaProfileEntryMap::iterator,bool> ret =
            antennaProfileEntryMap_.insert(std::make_pair(moduleId,antennaProfile));

          if(!ret.second)
            {
              ret.first->second = antennaProfile;
            }

          // update the delta cache
          ret = antennaProfileEntryDeltaMap_.insert(std::make_pair(moduleId,antennaProfile));

          if(!ret.second)
            {
              ret.first->second = antennaProfile;
            }
        }

    }
}
    
EMANE::Generators::EEL::EventInfoList EMANE::Generators::EEL::LoaderAntennaProfile::getEvents(EventPublishMode mode)
{
  EventInfoList eventInfoList;
  AntennaProfileEntryMap * pCache = 0;

  if(mode == DELTA)
    {
      pCache = &antennaProfileEntryDeltaMap_;
    }
  else
    {
      pCache = &antennaProfileEntryMap_;
    }

  AntennaProfileEntryMap::const_iterator iter =  pCache->begin();
  Events::AntennaProfiles profiles;
  
  for(const auto & entry : *pCache)
    {
      profiles.push_back(entry.second);
    }
  
  if(!profiles.empty())
    {
      eventInfoList.push_back({0,
            Events::AntennaProfileEvent::IDENTIFIER,
            Events::AntennaProfileEvent(profiles).serialize()});
      
    }
  
  antennaProfileEntryDeltaMap_.clear();

  return eventInfoList;
}


DECLARE_EEL_LOADER_PLUGIN(EMANE::Generators::EEL::LoaderAntennaProfile)
