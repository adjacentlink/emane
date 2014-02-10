/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "eelloaderlocation.h"
#include "emane/events/locationevent.h"
#include "emane/utils/parameterconvert.h"
#include "emane/generators/eel/formatexception.h"

#include <sstream>
#include <cstring>

EMANE::Generators::EEL::LoaderLocation::LoaderLocation()
{}
    
EMANE::Generators::EEL::LoaderLocation::~LoaderLocation()
{}

void EMANE::Generators::EEL::LoaderLocation::load(const ModuleType & moduleType, 
                                                  const ModuleId   & moduleId, 
                                                  const EventType  & eventType,
                                                  const InputArguments & args)
{
  if(moduleType == "nem")
    {
      if(eventType == "location")
        {
          if(args[0] == "gps")
            {
              InputArguments  params;

              size_t posStart = 0;
              size_t posEnd = 0;
          
              while(posEnd != std::string::npos)
                {
                  posEnd = args[1].find_first_of(",",posStart);
              
                  params.push_back(args[1].substr(posStart, posEnd - posStart));

                  posStart = posEnd + 1;
                }
          
              double dLatitudeDegress{};
              double dLongitudeDegress{};
              double dAltitudeMeters{};

              try
                {
                  for(size_t i = 0; i < params.size(); ++i)
                    {
                      switch(i)
                        {
                        case 0:
                          dLatitudeDegress = Utils::ParameterConvert(params[i]).toDouble();
                          break;
                          
                        case 1:
                          dLongitudeDegress = Utils::ParameterConvert(params[i]).toDouble();
                          break;
                          
                        case 2:
                          dAltitudeMeters = Utils::ParameterConvert(params[i]).toDouble();
                          break;
                          
                        case 3:
                          if(params[i] != "msl" && params[i] != "agl")
                            {
                              throw FormatException("LoaderLocation gps unkown altitude type");
                            }
                          break;
                          
                        default:
                          throw FormatException("LoaderLocation too many arguments");
                        }
                    }
                }
              catch(Utils::ParameterConvert::ConversionException & exp)
                {
                  std::stringstream sstream;
                  sstream<<"EELEventGenerator: Parameter conversion error. "<<exp.what()<<std::ends;
                  throw FormatException(sstream.str());
                }
              
              LocationEntry locationEntry{};

              Position position{dLatitudeDegress,dLongitudeDegress,dAltitudeMeters};
              
              locationEntry.setPosition(position);

              // update the full cache
              auto ret = locationEntryMap_.insert(std::make_pair(moduleId,locationEntry));
              
              if(!ret.second)
                {
                  ret.first->second.setPosition(position);
                }

              // update the delta cache
              ret = locationEntryDeltaMap_.insert(std::make_pair(moduleId,ret.first->second));

              if(!ret.second)
                {
                  // update gps position information only
                  ret.first->second.setPosition(position);
                }
            }
          else
            {
              throw FormatException("EELLoaderLocation only support 'gps' location type");
            }
        }
      else if(eventType == "orientation")
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

          double dPitchDegrees{};
          double dRollDegrees{};
          double dYawDegrees{};
          
          try
            {
              for(size_t i = 0; i < params.size(); ++i)
                {
                  switch(i)
                    {
                    case 0:
                      dPitchDegrees = Utils::ParameterConvert(params[i]).toDouble();
                      break;
                          
                    case 1:
                      dRollDegrees = Utils::ParameterConvert(params[i]).toDouble();
                      break;
                          
                    case 2:
                      dYawDegrees = Utils::ParameterConvert(params[i]).toDouble();
                      break;
                          
                    default:
                      if(params[i] != "degrees" && params[i] != "relative")
                        {
                          throw FormatException("EELLoaderLocation orientation unkown or unsupported keyword. "
                                                "Only degrees and relative keywords supported.");
                        }
                    }
                }
            }
          catch(Utils::ParameterConvert::ConversionException & exp)
            {
              std::stringstream sstream;
              sstream<<"EELEventGenerator: Parameter conversion error. "<<exp.what()<<std::ends;
              throw FormatException(sstream.str());
            }

          LocationEntry locationEntry{};

          Orientation orientation{dRollDegrees,dPitchDegrees,dYawDegrees};
              
          locationEntry.setOrientation(orientation);

          // update the full cache
          auto ret = locationEntryMap_.insert(std::make_pair(moduleId,locationEntry));

          if(!ret.second)
            {
              // update orientation information only
              ret.first->second.setOrientation(orientation);
            }

          // update the delta cache
          ret = locationEntryDeltaMap_.insert(std::make_pair(moduleId,ret.first->second));

          if(!ret.second)
            {
              // update orientation position information only
              ret.first->second.setOrientation(orientation);
            }
        }
      else if(eventType == "velocity")
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
          
          double dAzimuthDegrees{};
          double dElevationDegrees{};
          double dMagnitudeMetersPerSecond{};
          
          try
            {
              for(size_t i = 0; i < params.size(); ++i)
                {
                  switch(i)
                    {
                    case 0:
                      dAzimuthDegrees = Utils::ParameterConvert(params[i]).toDouble();
                      break;
                      
                    case 1:
                      dElevationDegrees = Utils::ParameterConvert(params[i]).toDouble();
                      break;
                      
                    case 2:
                      dMagnitudeMetersPerSecond = Utils::ParameterConvert(params[i]).toDouble();
                      break;
                      
                    default:
                      if(params[i] != "degrees" && 
                         params[i] != "mps" && 
                         params[i] != "azimuth" && 
                         params[i] != "relative")
                        {
                          throw FormatException("EELLoaderLocation velocity unkown or unsupported keyword. "
                                             "Only degrees, relative, mps and azimuth keywords supported.");
                        }
                    }
                }
            }
          catch(Utils::ParameterConvert::ConversionException & exp)
            {
              std::stringstream sstream;
              sstream<<"EELEventGenerator: Parameter conversion error. "<<exp.what()<<std::ends;
              throw FormatException(sstream.str());
            }


          LocationEntry locationEntry{};

          Velocity velocity{dAzimuthDegrees,dElevationDegrees,dMagnitudeMetersPerSecond};
              
          locationEntry.setVelocity(velocity);
          
              
          // update the full cache
          auto ret = locationEntryMap_.insert(std::make_pair(moduleId,locationEntry));

          if(!ret.second)
            {
              // update velocity information only
              ret.first->second.setVelocity(velocity);
            }

          
          // update the delta cache
          ret = locationEntryDeltaMap_.insert(std::make_pair(moduleId,ret.first->second));

          if(!ret.second)
            {
              // update velocity information only
              ret.first->second.setVelocity(velocity);
            }
        }
    }
}

    
EMANE::Generators::EEL::EventInfoList EMANE::Generators::EEL::LoaderLocation::getEvents(EventPublishMode mode)
{
  EventInfoList eventInfoList;
  LocationEntryMap * pCache = 0;

   
  if(!locationEntryDeltaMap_.empty())
    {
      if(mode == DELTA)
        {
          pCache = &locationEntryDeltaMap_;
        }
      else
        {
          pCache = &locationEntryMap_;
        }
      
      LocationEntryMap::const_iterator iter =  pCache->begin();

      Events::Locations locations;

      for(const auto & entry : *pCache)
        {
          const Position & position = entry.second.getPosition();
          auto optionalOrientation = entry.second.getOrientation();
          auto optionalVelocity = entry.second.getVelocity();
          
          locations.push_back({entry.first,position,optionalOrientation,optionalVelocity});
        }
      
      if(!locations.empty())
        {
          eventInfoList.push_back(EventInfo{0,
                Events::LocationEvent::IDENTIFIER,
                Events::LocationEvent(locations).serialize()});
        }
      
      locationEntryDeltaMap_.clear();
    }
  
  return eventInfoList;
}

DECLARE_EEL_LOADER_PLUGIN(EMANE::Generators::EEL::LoaderLocation)
