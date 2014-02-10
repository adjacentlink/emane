/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANEMODELSBYPASSPHYLAYER_HEADER_
#define EMANEMODELSBYPASSPHYLAYER_HEADER_

#include "emane/phylayerimpl.h"
#include "emane/phytypes.h"
#include "emane/utils/commonlayerstatistics.h"

#include "locationmanager.h"
#include "spectrummonitor.h"
#include "gainmanager.h"
#include "propagationmodelalgorithm.h"
#include "eventtablepublisher.h"

#include <set>
#include <cstdint>
#include <memory>

namespace EMANE
{
  class FrameworkPHY : public PHYLayerImplementor
  {
  public:
    FrameworkPHY(NEMId id,
                 PlatformServiceProvider* pPlatformService,
                 SpectrumMonitor * pSpectrumMonitor);
    
    ~FrameworkPHY();
    
    void initialize(Registrar & registrar) override;
    
    void configure(const ConfigurationUpdate & update) override;
    
    void start() override;
    
    void stop() override;
    
    void destroy() throw() override;
    
    void processUpstreamPacket(const CommonPHYHeader & hdr,
                               UpstreamPacket & pkt,
                               const ControlMessages & msgs) override;  

    // provides test harness access
    void processUpstreamPacket_i(const TimePoint & now,
                                 const CommonPHYHeader & hdr,
                                 UpstreamPacket & pkt,
                                 const ControlMessages & msgs);
    
    void processDownstreamControl(const ControlMessages & msgs) override;
    
    void processDownstreamPacket(DownstreamPacket & pkt,
                                 const ControlMessages & msgs) override;

    void processEvent(const EventId & eventId,
                      const Serialization & serialization) override;


    SpectrumMonitor & getSpectrumMonitor();

  private:
    SpectrumMonitor * pSpectrumMonitor_;
    GainManager gainManager_;
    LocationManager locationManager_;
    std::uint64_t u64BandwidthHz_;
    std::set<std::uint64_t> foi_;
    double dTxPowerdBm_;
    std::uint64_t u64TxFrequencyHz_;
    double dReceiverSensitivitydBm_;
    SpectrumMonitor::NoiseMode noiseMode_;
    std::uint16_t u16SubId_;
    std::uint16_t u16TxSequenceNumber_;
    std::pair<double,bool> optionalFixedAntennaGaindBi_;
    std::unique_ptr<PropagationModelAlgorithm> pPropagationModelAlgorithm_;
    Utils::CommonLayerStatistics commonLayerStatistics_;
    EventTablePublisher eventTablePublisher_;
  };
}

#endif // EMANEMODELSBYPASSPHYLAYER_HEADER_
