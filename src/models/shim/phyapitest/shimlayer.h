/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2012 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANEMODELSPHYAPITESTSHIMLAYER_HEADER_
#define EMANEMODELSPHYAPITESTSHIMLAYER_HEADER_

#include "emane/shimlayerimpl.h"
#include "emane/types.h"
#include "emane/transmitter.h"
#include "emane/frequencysegment.h"

namespace EMANE
{
  namespace Models
  {
    namespace PHYAPITest
    {
      /**
       * class PHYAPITestShimLayer
       *
       * @brief shim layer used to test the universal phy send/recv api
       *
       */
      class ShimLayer : public ShimLayerImplementor
      {
      public:
        
        ShimLayer(NEMId id,
                  PlatformServiceProvider * pPlatformService,
                  RadioServiceProvider * pRadioService);
        
        ~ShimLayer();
        
        void initialize(Registrar & registrar) override;
        
        void configure(const ConfigurationUpdate & update) override;
        
        void start() override;
        
        void postStart() override;
        
        void stop() override;
        
        void destroy() throw() override;
        
        void processUpstreamControl(const ControlMessages & msgs) override;
        
        void processDownstreamControl(const ControlMessages & msgs) override;
        
        void processUpstreamPacket(UpstreamPacket & pkt,
                                   const ControlMessages & msgs) override;
        
        void processDownstreamPacket(DownstreamPacket & pkt,
                                     const ControlMessages & msgs) override;
        
        void processTimedEvent(TimerEventId eventId,
                               const TimePoint & expireTime,
                               const TimePoint & scheduleTime,
                               const TimePoint & fireTime,
                               const void * arg) override;
        
      private:
        // the pkt payload size
        std::uint16_t u16PacketSize_;
        
        // the pkt tx interval (1 / pps)
        DoubleSeconds txInterval_;
        
        // the tx timed event id
        long txTimedEventId_;
        
        // the pkt destination
        NEMId dst_;
        
        AntennaProfileId antennaProfileId_;                                      
        
        double dAntennaAzimuthDegrees_;                                                         
        
        double dAntennaElevationDegrees_;
        
        float fTxPowerdBm_;
        
        std::uint64_t u64BandwidthHz_; 
        
        // additional transmitter info (ati)
        Transmitters transmitters_;
        
        // frequency info (fi)
        FrequencySegments frequencySegments_;
      };
    }
  }
}

#endif  // EMANEMODELSPHYAPITESTSHIMLAYER_HEADER_
