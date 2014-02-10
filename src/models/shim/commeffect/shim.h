/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2009-2010 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANEMODELSCOMMEFFECTSHIM_HEADER_
#define EMANEMODELSCOMMEFFECTSHIM_HEADER_

#include "emane/shimlayerimpl.h"

#include "emane/utils/randomnumberdistribution.h"
#include "emane/utils/commonlayerstatistics.h"

#include "profilemanager.h"

#include <random>

namespace EMANE
{
  namespace Models
  {
    namespace CommEffect
    {
      class Shim : public ShimLayerImplementor
      {
     
      public:
        Shim(NEMId id,
             PlatformServiceProvider * pPlatformService,
             RadioServiceProvider * pRadioService);
      
        ~Shim() override;
      
        void initialize(Registrar & registrar) override;
      
        void configure(const ConfigurationUpdate & update) override;
      
        void start() override;
      
        void stop() override;
      
        void destroy() throw() override;
      
        void processUpstreamControl(const ControlMessages & msgs) override;
      
        void processDownstreamControl(const ControlMessages & msgs) override;
      
        void processUpstreamPacket(UpstreamPacket & pkt,
                                   const ControlMessages & msgs) override;
      
        void processDownstreamPacket(DownstreamPacket & pkt,
                                     const ControlMessages & msgs) override;
      
        void processEvent(const EventId &,
                          const Serialization &) override;
      
        void processTimedEvent(TimerEventId eventId,
                               const TimePoint & expireTime,
                               const TimePoint & scheduleTime,
                               const TimePoint & fireTime,
                               const void * arg) override;

    
      private:
        using EORTimeMap = std::map<NEMId, TimePoint>;
      
        ProfileManager profileManager_;
      
        std::uint32_t u32UpstreamSequenceNumber_;
      
        std::uint32_t u32DownstreamSequenceNumber_;
      
        bool bDefaultConnectivity_;
      
        bool bEnablePromiscousMode_;

        bool bEnableTightTimingMode_;
      
        std::uint32_t u32GroupId_;
      
        EORTimeMap EORTimeMap_;
      
        Microseconds receiveBufferPeriod_;

        std::string sFilterFile_;

        Utils::RandomNumberDistribution<std::mt19937, 
                                 std::uniform_real_distribution<float>> RNDZeroToOneHundred_;

        std::mt19937 RNG_;

        Utils::CommonLayerStatistics commonLayerStatistics_;

        Microseconds randomize(const Microseconds & duration);
      
        size_t getTaskCount(float fLoss, float fDups);
      
        void setEORTime (NEMId src, const TimePoint & tvEORTime);
      
        std::pair<TimePoint,bool> getEORTime(NEMId src);
      };
    }
  }
}

#endif // EMANEMODELSCOMMEFFECTSHIM_HEADER_
