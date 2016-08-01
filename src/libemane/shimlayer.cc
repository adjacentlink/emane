/*
 * Copyright (c) 2013-2014,2016 - Adjacent Link LLC, Bridgewater,
 * New Jersey
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

#include "shimlayer.h"

EMANE::ShimLayer::ShimLayer(NEMId id,
                            NEMLayer * pImplementor,
                            PlatformServiceProvider * pPlatformService) :
  NEMQueuedLayer{id,pPlatformService},
  pImplementor_{pImplementor},
  pPlatformService_{pPlatformService}
{}

EMANE::ShimLayer::~ShimLayer()
{}


void EMANE::ShimLayer::initialize(Registrar & registrar)
{
  NEMQueuedLayer::initialize(registrar);

  pImplementor_->initialize(registrar);
}


void EMANE::ShimLayer::configure(const ConfigurationUpdate & update)
{
  pImplementor_->configure(update);
}


void EMANE::ShimLayer::start()
{
  // start the queue processing thread
  NEMQueuedLayer::start();

  pImplementor_->start();
}


void EMANE::ShimLayer::postStart()
{
  pImplementor_->postStart();
}


void EMANE::ShimLayer::stop()
{
  pImplementor_->stop();

  // stop the queue processing thread
  NEMQueuedLayer::stop();
}


void EMANE::ShimLayer::destroy() throw()
{
  pImplementor_->destroy();
}


void EMANE::ShimLayer::doProcessConfiguration(const ConfigurationUpdate & update)
{
  pImplementor_->processConfiguration(update);
}


void EMANE::ShimLayer::doProcessUpstreamPacket(UpstreamPacket & pkt,
                                               const ControlMessages & msgs)
{
  pImplementor_->processUpstreamPacket(pkt,msgs);
}


void EMANE::ShimLayer::doProcessDownstreamPacket(DownstreamPacket & pkt,
                                                 const ControlMessages & msgs)
{
  pImplementor_->processDownstreamPacket(pkt,msgs);
}


void EMANE::ShimLayer::doProcessUpstreamControl(const ControlMessages & msgs)
{
  pImplementor_->processUpstreamControl(msgs);
}


void EMANE::ShimLayer::doProcessDownstreamControl(const ControlMessages & msgs)
{
  pImplementor_->processDownstreamControl(msgs);
}


void EMANE::ShimLayer::setUpstreamTransport(UpstreamTransport * pUpstreamTransport)
{
  pImplementor_->setUpstreamTransport(pUpstreamTransport);
}


void EMANE::ShimLayer::setDownstreamTransport(DownstreamTransport * pDownstreamTransport)
{
  pImplementor_->setDownstreamTransport(pDownstreamTransport);
}


void EMANE::ShimLayer::doProcessEvent(const EventId & eventId,
                                      const Serialization & serialization)
{
  pImplementor_->processEvent(eventId,serialization);
}


void EMANE::ShimLayer::doProcessTimedEvent(TimerEventId eventId,
                                           const TimePoint & expireTime,
                                           const TimePoint & scheduleTime,
                                           const TimePoint & fireTime,
                                           const void * arg)
{
  pImplementor_->processTimedEvent(eventId,expireTime,scheduleTime,fireTime,arg);
}
