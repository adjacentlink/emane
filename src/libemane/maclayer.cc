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

#include "maclayer.h"
#include "logservice.h"

EMANE::MACLayer::MACLayer(NEMId id,
                          MACLayerImplementor * pImplementor,
                          PlatformServiceProvider *pPlatformService):
  NEMQueuedLayer{id, pPlatformService},
  pImplementor_{pImplementor},
  pPlatformService_{pPlatformService}
{}

EMANE::MACLayer::~MACLayer(){}

void EMANE::MACLayer::initialize(Registrar & registrar)
{
  NEMQueuedLayer::initialize(registrar);

  pImplementor_->initialize(registrar);
}

void EMANE::MACLayer::configure(const ConfigurationUpdate & update)
{
  pImplementor_->configure(update);
}

void EMANE::MACLayer::start()
{
  // start the queue processing thread
  NEMQueuedLayer::start();

  pImplementor_->start();
}

void EMANE::MACLayer::postStart()
{
  pImplementor_->postStart();
}

void EMANE::MACLayer::stop()
{
  pImplementor_->stop();

  // stop the queue processing thread
  NEMQueuedLayer::stop();
}

void EMANE::MACLayer::destroy()
  throw()
{
  pImplementor_->destroy();
}

void EMANE::MACLayer::doProcessConfiguration(const ConfigurationUpdate & update)
{
  pImplementor_->processConfiguration(update);
}

void EMANE::MACLayer::doProcessUpstreamPacket(UpstreamPacket & pkt,
                                              const ControlMessages & msgs)
{
  static_cast<UpstreamTransport *>(pImplementor_.get())->processUpstreamPacket(pkt,msgs);
}

void EMANE::MACLayer::doProcessDownstreamPacket(DownstreamPacket & pkt,
                                                const ControlMessages & ctrl)
{
  pImplementor_->processDownstreamPacket(pkt,ctrl);
}

void EMANE::MACLayer::doProcessUpstreamControl(const ControlMessages & msg)
{
  pImplementor_->processUpstreamControl(msg);
}

void EMANE::MACLayer::doProcessDownstreamControl(const ControlMessages & msg)
{
  pImplementor_->processDownstreamControl(msg);
}

void EMANE::MACLayer::setUpstreamTransport(UpstreamTransport * pUpstreamTransport)
{
  pImplementor_->setUpstreamTransport(pUpstreamTransport);
}

void EMANE::MACLayer::setDownstreamTransport(DownstreamTransport * pDownstreamTransport)
{
  pImplementor_->setDownstreamTransport(pDownstreamTransport);
}

void EMANE::MACLayer::doProcessEvent(const EventId & eventId, 
                                     const Serialization & serialization)
{
  pImplementor_->processEvent(eventId,serialization);
}


void EMANE::MACLayer::doProcessTimedEvent(TimerEventId eventId,
                                          const TimePoint & requestExpireTime,
                                          const TimePoint & scheduleTime,
                                          const TimePoint & fireTime,
                                          const void * arg)
{
  pImplementor_->processTimedEvent(eventId,requestExpireTime,scheduleTime,fireTime,arg);
}
