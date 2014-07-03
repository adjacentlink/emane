/*
 * Copyright (c) 2014 - Adjacent Link LLC, Bridgewater, New Jersey
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
 * * Neither the name of Adjacent Link LLC nor the names of its
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

#include "transportlayer.h"

EMANE::TransportLayer::TransportLayer(NEMId id,
                                      Transport * pImplementor,
                                      PlatformServiceProvider * pPlatformService) :
  NEMQueuedLayer{id, pPlatformService},
  pImplementor_{pImplementor},
  pPlatformService_{pPlatformService}
{}

EMANE::TransportLayer::~TransportLayer()
{}

void EMANE::TransportLayer::initialize(Registrar & registrar)
{
  NEMQueuedLayer::initialize(registrar);
  
  pImplementor_->initialize(registrar);
}


void EMANE::TransportLayer::configure(const ConfigurationUpdate & update)
{
  pImplementor_->configure(update);
}


void EMANE::TransportLayer::start()
{
  // start the queue processing thread
  NEMQueuedLayer::start();

  pImplementor_->start();
}


void EMANE::TransportLayer::postStart()
{
  pImplementor_->postStart();
}


void EMANE::TransportLayer::stop()
{
  pImplementor_->stop();

  // stop the queue processing thread
  NEMQueuedLayer::stop();
}


void EMANE::TransportLayer::destroy() throw()
{
  pImplementor_->destroy();
}

void EMANE::TransportLayer::setDownstreamTransport(DownstreamTransport * pDownstreamTransport)
{
  pImplementor_->setDownstreamTransport(pDownstreamTransport);
}

void EMANE::TransportLayer::doProcessConfiguration(const ConfigurationUpdate & update)
{
  pImplementor_->processConfiguration(update);
}

void EMANE::TransportLayer::doProcessUpstreamPacket(UpstreamPacket & pkt,
                                              const ControlMessages & ctrl)
{
  static_cast<UpstreamTransport *>(pImplementor_.get())->processUpstreamPacket(pkt,ctrl);
}


void EMANE::TransportLayer::doProcessUpstreamControl(const ControlMessages & msgs)
{
  pImplementor_->processUpstreamControl(msgs);
}


void EMANE::TransportLayer::doProcessEvent(const EventId & eventId,
                                     const Serialization & serialization)
{
  pImplementor_->processEvent(eventId,serialization);
}


void EMANE::TransportLayer::doProcessTimedEvent(TimerEventId eventId,
                                          const TimePoint & expireTime,
                                          const TimePoint & scheduleTime,
                                          const TimePoint & fireTime,
                                          const void * arg)
{
  pImplementor_->processTimedEvent(eventId,expireTime,scheduleTime,fireTime,arg);
}

// method meaningless for a Transport layer
void EMANE::TransportLayer::setUpstreamTransport(UpstreamTransport *)
{}

void EMANE::TransportLayer::doProcessDownstreamPacket(DownstreamPacket &,
                                                      const ControlMessages &)
{}


void EMANE::TransportLayer::doProcessDownstreamControl(const ControlMessages &)
{}
