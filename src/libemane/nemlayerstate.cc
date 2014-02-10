/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2008 - DRS CenGen, LLC, Columbia, Maryland
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
 */

#include "nemlayerstate.h"
#include "nemstatefullayer.h"
#include "logservice.h"

EMANE::NEMLayerState::NEMLayerState(const char * pzStateName):
  pzStateName_(pzStateName){}

EMANE::NEMLayerState::~NEMLayerState(){}
    
void EMANE::NEMLayerState::handleInitialize(NEMStatefulLayer *,
                                            NEMLayer *,
                                            Registrar &)
{
  LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                          ERROR_LEVEL,
                          "NEMLayer invalid %s transition in %s state",
                          "initialize",
                          getStateName().c_str()); 
}
    
void EMANE::NEMLayerState::handleConfigure(NEMStatefulLayer *,
                                           NEMLayer *,
                                           const ConfigurationUpdate &)
{
  LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                          ERROR_LEVEL,
                          "NEMLayer invalid %s transition in %s state",
                          "configure",
                          getStateName().c_str()); 
}

void EMANE::NEMLayerState:: handleStart(NEMStatefulLayer *,
                                        NEMLayer *)
{
  LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                          ERROR_LEVEL,
                          "NEMLayer invalid %s transition in %s state",
                          "start",
                          getStateName().c_str()); 
}

void EMANE::NEMLayerState:: handlePostStart(NEMStatefulLayer *,
                                            NEMLayer *)
{
  LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                          ERROR_LEVEL,
                          "NEMLayer invalid %s transition in %s state",
                          "postStart",
                          getStateName().c_str()); 
}

void EMANE::NEMLayerState::handleStop(NEMStatefulLayer *,
                                      NEMLayer *)
{
  LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                          ERROR_LEVEL,
                          "NEMLayer invalid %s transition in %s state",
                          "stop",
                          getStateName().c_str()); 
}
    
void EMANE::NEMLayerState::handleDestroy(NEMStatefulLayer *,
                                         NEMLayer *)
  throw()
{
  LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                          ERROR_LEVEL,
                          "NEMLayer invalid %s transition in %s state",
                          "destroy",
                          getStateName().c_str()); 
}

void EMANE::NEMLayerState::processConfiguration(NEMStatefulLayer *, 
                                                NEMLayer *, 
                                                const ConfigurationUpdate &)
{
  LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                          ERROR_LEVEL,
                          "NEMLayer %s not valid in %s state",
                          "processConfigurationUpdate",
                          getStateName().c_str());
}

void EMANE::NEMLayerState::processDownstreamControl(NEMStatefulLayer *,
                                                    NEMLayer *,
                                                    const ControlMessages &)
{
  LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                          ERROR_LEVEL,
                          "NEMLayer %s not valid in %s state",
                          "processDownstreamControl",
                          getStateName().c_str());
}
   
void EMANE::NEMLayerState::processDownstreamPacket(NEMStatefulLayer *, 
                                                   NEMLayer *,  
                                                   DownstreamPacket &, 
                                                   const ControlMessages &)
{
  LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                          ERROR_LEVEL,
                          "NEMLayer %s not valid in %s state",
                          "processDownstreamPacket",
                          getStateName().c_str());
}
    
void EMANE::NEMLayerState::processUpstreamPacket(NEMStatefulLayer *, 
                                                 NEMLayer *, 
                                                 UpstreamPacket &,
                                                 const ControlMessages &)
{
  LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                          ERROR_LEVEL,
                          "NEMLayer %s not valid in %s state",
                          "processUpstreamPacket",
                          getStateName().c_str());
}

void EMANE::NEMLayerState::processUpstreamControl(NEMStatefulLayer *,
                                                  NEMLayer *,
                                                  const ControlMessages &)
{
  LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                          ERROR_LEVEL,
                          "NEMLayer %s not valid in %s state",
                          "processUpstreamControl",
                          getStateName().c_str());
}

void EMANE::NEMLayerState::processEvent(NEMStatefulLayer *,
                                        NEMLayer *,
                                        const EventId &,
                                        const Serialization &)
{
  LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                          ERROR_LEVEL,
                          "NEMLayer %s not valid in %s state",
                          "processEvent",
                          getStateName().c_str()); 
}


void EMANE::NEMLayerState::processTimedEvent(NEMStatefulLayer *,
                                             NEMLayer *,
                                             TimerEventId,
                                             const TimePoint &,
                                             const TimePoint &,
                                             const TimePoint &,
                                             const void *)
{
  LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                          ERROR_LEVEL,
                          "NEMLayer %s not valid in %s state",
                          "processTimedEvent",
                          getStateName().c_str()); 
}


void EMANE::NEMLayerState::changeState(NEMStatefulLayer * pStatefulLayer,
                                       NEMLayerState * pState)
{
  pStatefulLayer->changeState(pState);
}

std::string EMANE::NEMLayerState::getStateName() const
{
  return pzStateName_;
}
