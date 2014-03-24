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

#include "maclayer.h"
#include "emane/configureexception.h"

namespace {
  const std::uint16_t DROP_CODE_REGISTRATION_ID = 1;

  EMANE::StatisticTableLabels STATISTIC_TABLE_LABELS {"Reg Id"};
}

EMANE::Models::Bypass::MACLayer::MACLayer(NEMId id,
                                          PlatformServiceProvider* pPlatformService,
                                          RadioServiceProvider * pRadioServiceProvider):
  MACLayerImplementor{id, pPlatformService, pRadioServiceProvider},
  u16SequenceNumber_{},
  commonLayerStatistics_{STATISTIC_TABLE_LABELS}
{}

EMANE::Models::Bypass::MACLayer::~MACLayer()
{}

void EMANE::Models::Bypass::MACLayer::processUpstreamControl(const ControlMessages &)
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "MACI %03hu Models::Bypass::MACLayer::%s",
                          id_, 
                          __func__);
}

void EMANE::Models::Bypass::MACLayer::processDownstreamControl(const ControlMessages &)
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "MACI %03hu Models::Bypass::MACLayer::%s",
                          id_, 
                          __func__);

}

void EMANE::Models::Bypass::MACLayer::processUpstreamPacket(const CommonMACHeader & hdr,
                                                            UpstreamPacket & pkt, 
                                                            const ControlMessages &)
{
  TimePoint beginTime{Clock::now()};

  commonLayerStatistics_.processInbound(pkt);

  if(hdr.getRegistrationId() != type_)
    {
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              ERROR_LEVEL, 
                              "MACI %03hu Models::Bypass::MACLayer::%s: MAC Registration Id %hu does not match our Id %hu, drop.",
                              id_, 
                              __func__,
                              hdr.getRegistrationId(), 
                              type_);

      commonLayerStatistics_.processOutbound(pkt, 
                                             std::chrono::duration_cast<Microseconds>(Clock::now() - beginTime), 
                                             DROP_CODE_REGISTRATION_ID);

      // drop
      return;
    }

  LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                         DEBUG_LEVEL,
                         "MACI %03hu Models::Bypass::MACLayer::%s src %hu dst %hu dscp %hhu",
                         id_, 
                         __func__,
                         pkt.getPacketInfo().getSource(),
                         pkt.getPacketInfo().getDestination(),
                         pkt.getPacketInfo().getPriority());

  commonLayerStatistics_.processOutbound(pkt, 
                                         std::chrono::duration_cast<Microseconds>(Clock::now() - beginTime));

  // pass the pkt upstream
  sendUpstreamPacket(pkt);
}


void EMANE::Models::Bypass::MACLayer::processDownstreamPacket(DownstreamPacket & pkt,
                                                              const ControlMessages &)
{
  TimePoint beginTime{Clock::now()};

  commonLayerStatistics_.processInbound(pkt);

  LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                         DEBUG_LEVEL,
                         "MACI %03hu Models::Bypass::MACLayer::%s src %hu dst %hu dscp %hhu",
                         id_, 
                         __func__,
                         pkt.getPacketInfo().getSource(),
                         pkt.getPacketInfo().getDestination(),
                         pkt.getPacketInfo().getPriority());

  commonLayerStatistics_.processOutbound(pkt, 
                                         std::chrono::duration_cast<Microseconds>(Clock::now() - beginTime));

  // pass the pkt downstream
  sendDownstreamPacket(CommonMACHeader{type_, u16SequenceNumber_++}, pkt);
}

void EMANE::Models::Bypass::MACLayer::initialize(Registrar & registrar)
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "MACI %03hu Models::Bypass::MACLayer::%s",
                          id_, 
                          __func__);

  commonLayerStatistics_.registerStatistics(registrar.statisticRegistrar());
}

void EMANE::Models::Bypass::MACLayer::configure(const ConfigurationUpdate & update)
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "MACI %03hu Models::Bypass::MACLayer::%s",
                          id_, 
                          __func__);

  if(!update.empty())
    {
      throw ConfigureException("Models::Bypass::MACLayer: Unexpected configuration items.");
    }
}

void EMANE::Models::Bypass::MACLayer::start()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "MACI %03hu Models::Bypass::MACLayer::%s",
                          id_, 
                          __func__);
}

void EMANE::Models::Bypass::MACLayer::stop()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "MACI %03hu Models::Bypass::MACLayer::%s",
                          id_, 
                          __func__);
}

void EMANE::Models::Bypass::MACLayer::destroy()
  throw()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "MACI %03hu Models::Bypass::MACLayer::%s",
                          id_, 
                          __func__);
}

void EMANE::Models::Bypass::MACLayer::processEvent(const EventId &, const Serialization &)
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "MACI %03hu Models::Bypass::MACLayer::%s",
                          id_, 
                          __func__);
}

void EMANE::Models::Bypass::MACLayer::processTimedEvent(TimerEventId,
                                                        const TimePoint &,
                                                        const void *)
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "MACI %03hu Models::Bypass::MACLayer::%s",
                          id_, 
                          __func__);
}

DECLARE_MAC_LAYER(EMANE::Models::Bypass::MACLayer);
