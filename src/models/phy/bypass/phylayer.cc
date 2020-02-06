/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2008-2009 - DRS CenGen, LLC, Columbia, Maryland
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

#include "phylayer.h"
#include "emane/commonphyheader.h"
#include "emane/configureexception.h"

EMANE::Models::Bypass::PHYLayer::PHYLayer(NEMId id,
                                          PlatformServiceProvider* pPlatformService,
                                          RadioServiceProvider * pRadioService):
  PHYLayerImplementor(id, pPlatformService, pRadioService),
  commonLayerStatistics_{{}}
{}

EMANE::Models::Bypass::PHYLayer::~PHYLayer()
{}

void EMANE::Models::Bypass::PHYLayer::processDownstreamControl(const ControlMessages &)
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "PHYI %03hu Models::Bypass::PHYLayer::%s",
                          id_,
                          __func__);
}

void EMANE::Models::Bypass::PHYLayer::processDownstreamPacket(DownstreamPacket & pkt,
                                                              const ControlMessages &)
{
  TimePoint beginTime{Clock::now()};

  commonLayerStatistics_.processInbound(pkt);

  LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                         DEBUG_LEVEL,
                         "PHYI %03hu Models::Bypass::PHYLayer::%s",
                         id_,
                         __func__);

  CommonPHYHeader hdr{type_, // phy registration id
      0,                     // subid
      0,                     // tx power dBm
      0,                     // bandwidth Hz
      Clock::now(),          // time stamp
      FrequencySegments{{0,Microseconds::zero()}},
      Transmitters{{id_,0}},
        {0,false}};                     // antenna gain dBi

  commonLayerStatistics_.processOutbound(pkt,
                                         std::chrono::duration_cast<Microseconds>(Clock::now() - beginTime));

  sendDownstreamPacket(hdr,pkt);
}

void EMANE::Models::Bypass::PHYLayer::processUpstreamPacket(const CommonPHYHeader &,
                                                            UpstreamPacket & pkt,
                                                            const ControlMessages &)
{
  TimePoint beginTime{Clock::now()};

  commonLayerStatistics_.processInbound(pkt);

  LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                         DEBUG_LEVEL,
                         "PHYI %03hu Models::Bypass::PHYLayer::%s",
                         id_,
                         __func__);

  commonLayerStatistics_.processOutbound(pkt,
                                         std::chrono::duration_cast<Microseconds>(Clock::now() - beginTime));

  sendUpstreamPacket(pkt);
}

void EMANE::Models::Bypass::PHYLayer::initialize(Registrar & registrar)
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "PHYI %03hu Models::Bypass::PHYLayer::%s",
                          id_,
                          __func__);

  commonLayerStatistics_.registerStatistics(registrar.statisticRegistrar());;
}

void EMANE::Models::Bypass::PHYLayer::configure(const ConfigurationUpdate & update)
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "PHYI %03hu Models::Bypass::PHYLayer::%s",
                          id_,
                          __func__);

  if(!update.empty())
    {
      throw ConfigureException("Models::Bypass::PHYLayer: Unexpected configuration items.");
    }
}

void EMANE::Models::Bypass::PHYLayer::start()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "PHYI %03hu Models::Bypass::PHYLayer::%s",
                          id_,
                          __func__);
}

void EMANE::Models::Bypass::PHYLayer::stop()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "PHYI %03hu Models::Bypass::PHYLayer::%s",
                          id_,
                          __func__);
}

void EMANE::Models::Bypass::PHYLayer::destroy() throw()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "PHYI %03hu Models::Bypass::PHYLayer::%s",
                          id_,
                          __func__);

}

void EMANE::Models::Bypass::PHYLayer::processEvent(const EventId & ,
                                                   const Serialization & )
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "PHYI %03hu Models::Bypass::PHYLayer::%s",
                          id_,
                          __func__);
}

void EMANE::Models::Bypass::PHYLayer::processTimedEvent(TimerEventId,
                                                        const TimePoint &,
                                                        const TimePoint &,
                                                        const TimePoint &,
                                                        const void *)
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "PHYI %03hu Models::Bypass::PHYLayer::%s",
                          id_,
                          __func__);
}

DECLARE_PHY_LAYER(EMANE::Models::Bypass::PHYLayer);
