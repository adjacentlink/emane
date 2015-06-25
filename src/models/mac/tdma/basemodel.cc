/*
 * Copyright (c) 2015 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "emane/models/tdma/basemodel.h"
#include "basemodelimpl.h"

EMANE::Models::TDMA::BaseModel::BaseModel(NEMId id,
                                          PlatformServiceProvider * pPlatformServiceProvider,
                                          RadioServiceProvider * pRadioServiceProvider,
                                          Scheduler * pScheduler,
                                          QueueManager * pQueueManager):
  MACLayerImplementor{id, pPlatformServiceProvider, pRadioServiceProvider},
  pImpl_{new Implementation{id,
        pPlatformServiceProvider,
        pRadioServiceProvider,
        pScheduler,
        pQueueManager,
        this}}
{}



EMANE::Models::TDMA::BaseModel::~BaseModel()
{}


void
EMANE::Models::TDMA::BaseModel::initialize(Registrar & registrar)
{
  pImpl_->initialize(registrar);
}



void
EMANE::Models::TDMA::BaseModel::configure(const ConfigurationUpdate & update)
{
  pImpl_->configure(update);
}

void
EMANE::Models::TDMA::BaseModel::start()
{
  pImpl_->start();
}


void
EMANE::Models::TDMA::BaseModel::postStart()
{
  pImpl_->postStart();
}


void
EMANE::Models::TDMA::BaseModel::stop()
{
  pImpl_->stop();
}



void
EMANE::Models::TDMA::BaseModel::destroy()
  throw()
{
  pImpl_->destroy();
}

void EMANE::Models::TDMA::BaseModel::processUpstreamControl(const ControlMessages & msgs)
{
  pImpl_->processUpstreamControl(msgs);
}


void EMANE::Models::TDMA::BaseModel::processUpstreamPacket(const CommonMACHeader & hdr,
                                                           UpstreamPacket & pkt,
                                                           const ControlMessages & msgs)
{
  pImpl_->processUpstreamPacket(hdr,pkt,msgs);
}

void EMANE::Models::TDMA::BaseModel::processDownstreamControl(const ControlMessages & msgs)
{
  pImpl_->processDownstreamControl(msgs);
}


void EMANE::Models::TDMA::BaseModel::processDownstreamPacket(DownstreamPacket & pkt,
                                                             const ControlMessages & msgs)
{
  pImpl_->processDownstreamPacket(pkt,msgs);
}


void EMANE::Models::TDMA::BaseModel::processEvent(const EventId & eventId,
                                                  const Serialization & serialization)
{
  pImpl_->processEvent(eventId,serialization);
}

void EMANE::Models::TDMA::BaseModel::processTimedEvent(TimerEventId eventId,
                                            const TimePoint & expireTime,
                                            const TimePoint & scheduleTime,
                                            const TimePoint & fireTime,
                                            const void * arg)
{
  pImpl_->processTimedEvent(eventId,
                            expireTime,
                            scheduleTime,
                            fireTime,
                            arg);
}


void EMANE::Models::TDMA::BaseModel::processConfiguration(const ConfigurationUpdate & update)
{
  pImpl_->processConfiguration(update);
}

void EMANE::Models::TDMA::BaseModel::notifyScheduleChange(const Frequencies & frequencies,
                                                          std::uint64_t u64BandwidthHz,
                                                          const Microseconds & slotDuration,
                                                          const Microseconds & slotOverhead)
{
  pImpl_->notifyScheduleChange(frequencies,u64BandwidthHz,slotDuration,slotOverhead);
}


void EMANE::Models::TDMA::BaseModel::processSchedulerPacket(DownstreamPacket & pkt)
{
  pImpl_->processSchedulerPacket(pkt);
}


void EMANE::Models::TDMA::BaseModel::processSchedulerControl(const ControlMessages & msgs)
{
  pImpl_->processSchedulerControl(msgs);
}


EMANE::Models::TDMA::QueueInfos EMANE::Models::TDMA::BaseModel::getPacketQueueInfo() const
{
  return pImpl_->getPacketQueueInfo();
}
