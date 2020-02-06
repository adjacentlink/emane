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

#ifndef EMANEMODELSBYPASSPHYLAYER_HEADER_
#define EMANEMODELSBYPASSPHYLAYER_HEADER_

#include "emane/phylayerimpl.h"
#include "emane/phytypes.h"
#include "emane/utils/commonlayerstatistics.h"

namespace EMANE
{
  namespace Models
  {
    namespace Bypass
    {

      /**
       * @class PHYLayer
       *
       * @brief Implementation of the bypass PHY layer
       */
      class PHYLayer : public PHYLayerImplementor
      {
      public:
        /**
         *
         * @brief constructor
         *
         */
        PHYLayer(NEMId id,
                 PlatformServiceProvider* pPlatformService,
                 RadioServiceProvider * pRadioService);

        /**
         *
         * @brief destructor
         *
         */
        ~PHYLayer();

        void initialize(Registrar & registrar) override;

        void configure(const ConfigurationUpdate & update) override;

        void start() override;

        void stop() override;

        void destroy() throw() override;

        void processUpstreamPacket(const CommonPHYHeader & hdr,
                                   UpstreamPacket & pkt,
                                   const ControlMessages & msgs) override;

        void processDownstreamControl(const ControlMessages & msgs) override;

        void processDownstreamPacket(DownstreamPacket & pkt,
                                     const ControlMessages & msgs) override;

        void processEvent(const EventId & id,
                          const Serialization & serialization) override;

        void processTimedEvent(TimerEventId eventId,
                               const TimePoint & expireTime,
                               const TimePoint & scheduleTime,
                               const TimePoint & fireTime,
                               const void * arg) override;

      private:
        /* Phy Layer Type */
        static const RegistrationId type_ = REGISTERED_EMANE_PHY_BYPASS;

        Utils::CommonLayerStatistics commonLayerStatistics_;
      };
    }
  }
}

#endif // EMANEMODELSBYPASSPHYLAYER_HEADER_
