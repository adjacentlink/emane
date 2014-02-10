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

#include "nemstatefullayer.h"
#include "nemlayerstate.h"
#include "nemlayerstateuninitialized.h"

EMANE::NEMStatefulLayer::NEMStatefulLayer(NEMId id, 
                                          NEMLayer * pLayer, 
                                          EMANE::PlatformServiceProvider *pPlatformService):
  NEMLayer(id, pPlatformService),
  pLayer_(pLayer),
  pState_(NEMLayerStateUninitializedSingleton::instance()){}

EMANE::NEMStatefulLayer::~NEMStatefulLayer(){}
    
void EMANE::NEMStatefulLayer::initialize(Registrar & registrar)
{
  pState_->handleInitialize(this,pLayer_.get(),registrar);
}

void EMANE::NEMStatefulLayer::configure(const ConfigurationUpdate & update)
{
  pState_->handleConfigure(this,pLayer_.get(),update);
}

void EMANE::NEMStatefulLayer::start()
{
  pState_->handleStart(this,pLayer_.get());
}

void EMANE::NEMStatefulLayer::postStart()
{
  pState_->handlePostStart(this,pLayer_.get());
}

void EMANE::NEMStatefulLayer::stop()
{
  pState_->handleStop(this,pLayer_.get());
}

void EMANE::NEMStatefulLayer::destroy()
  throw()
{
  pState_->handleDestroy(this,pLayer_.get());
}

void EMANE::NEMStatefulLayer::processConfiguration(const ConfigurationUpdate & update)
{
  pState_->processConfiguration(this,pLayer_.get(),update);
}

void EMANE::NEMStatefulLayer::processDownstreamControl(const ControlMessages & msgs)
{
  pState_->processDownstreamControl(this,pLayer_.get(),msgs);
}

void EMANE::NEMStatefulLayer::processDownstreamPacket(DownstreamPacket & pkt,
                                                      const ControlMessages & msgs)
{
  pState_->processDownstreamPacket(this,pLayer_.get(),pkt,msgs);
}

void EMANE::NEMStatefulLayer::processUpstreamPacket(UpstreamPacket & pkt,
                                                    const ControlMessages & msgs)
{
  pState_->processUpstreamPacket(this,pLayer_.get(),pkt,msgs);
}

void EMANE::NEMStatefulLayer::processUpstreamControl(const ControlMessages & msgs)
{
  pState_->processUpstreamControl(this,pLayer_.get(),msgs);
}

void EMANE::NEMStatefulLayer::processEvent(const EventId & id,
                                           const Serialization & serialization)
{
  pState_->processEvent(this,pLayer_.get(),id,serialization);
}

void EMANE::NEMStatefulLayer::setUpstreamTransport(UpstreamTransport * pUpstreamTransport)
{
  pLayer_->setUpstreamTransport(pUpstreamTransport);
}

void EMANE::NEMStatefulLayer::setDownstreamTransport(DownstreamTransport * pDownstreamTransport)
{
  pLayer_->setDownstreamTransport(pDownstreamTransport);
}

void EMANE::NEMStatefulLayer::changeState(NEMLayerState * pState)
{
  pState_ = pState;
}

void  EMANE::NEMStatefulLayer::processTimedEvent(TimerEventId eventId,
                                                 const TimePoint & expireTime,
                                                 const TimePoint & scheduleTime,
                                                 const TimePoint & fireTime,
                                                 const void * arg)
{
  pState_->processTimedEvent(this, pLayer_.get(),eventId,expireTime,scheduleTime,fireTime,arg);
}

