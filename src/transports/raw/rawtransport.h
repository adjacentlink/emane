/*
 * Copyright (c) 2013,2016 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2008-2010 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANETRANSPORTSRAWRAWTRANSPORT_HEADER_
#define EMANETRANSPORTSRAWRAWTRANSPORT_HEADER_

#include <pcap.h>
#include <memory.h>

#include "ethernettransport.h"
#include "emane/utils/netutils.h"
#include "emane/utils/bitpool.h"

#include <thread>

#include <set>

namespace EMANE
{
  namespace Transports
  {
    namespace Raw
    {
      /**
       * @class RawTransport
       *
       * @brief Raw Device EMANE Transport
       */
      class RawTransport : public Ethernet::EthernetTransport
      {
      public:
        RawTransport(NEMId id,
                     PlatformServiceProvider *pPlatformService);

        ~RawTransport();

        void initialize(Registrar & registrar) override;

        void configure(const ConfigurationUpdate & items) override;

        void start() override;

        void stop() override;

        void destroy() throw() override;

        void processUpstreamPacket(UpstreamPacket & pkt,
                                   const ControlMessages & msgs) override;

        void processUpstreamControl(const ControlMessages & msgs) override;


      private:
        std::string sTargetDevice_;

        Utils::EtherAddr macAddr_;

        std::thread thread_;

        pcap_t *pPcapHandle_;

        Utils::BitPool *pBitPool_;

        std::uint64_t u64BitRate_;

        /**
         *
         * @brief read device method reads from raw device and sends packets downstream.
         *
         * @retval NULL
         *
         */
        void readDevice();


        void handleUpstreamControl(const ControlMessages & msgs);
      };
    }
  }
}

#endif // EMANETRANSPORTSRAWRAWTRANSPORT_HEADER_
