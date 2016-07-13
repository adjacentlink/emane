/*
 * Copyright (c) 2013,2015-2016 - Adjacent Link LLC, Bridgewater,
 * New Jersey
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

#ifndef EMANETRANSPORTSVIRTUALVIRTUALTRANSPORT_HEADER_
#define EMANETRANSPORTSVIRTUALVIRTUALTRANSPORT_HEADER_

#include "ethernettransport.h"

#include "emane/inetaddr.h"
#include "emane/utils/bitpool.h"
#include "emane/flowcontrolclient.h"

#include "emane/utils/commonlayerstatistics.h"

#include "tuntap.h"

namespace EMANE
{
  namespace Transports
  {
    namespace Virtual
    {

      /**
       * @class VirtualTransport
       *
       * @brief Virtual Device EMANE Transport
       */
      class VirtualTransport : public Ethernet::EthernetTransport
      {
      public:
        VirtualTransport(NEMId id,
                         PlatformServiceProvider *pPlatformService);

        ~VirtualTransport();

        void initialize(Registrar & registrar) override;

        void configure(const ConfigurationUpdate & update) override;

        void start() override;

        void postStart() override;

        void stop() override;

        void destroy() throw() override;

        void processUpstreamPacket(UpstreamPacket & pkt,
                                   const ControlMessages & msgs);

        void processUpstreamControl(const ControlMessages & msgs);

      private:
        INETAddr address_;

        INETAddr mask_;

        std::string sDeviceName_;

        std::string sDevicePath_;

        bool bARPMode_;

        TunTap  * pTunTap_;

        Utils::BitPool * pBitPool_;

        bool bCanceled_;

        FlowControlClient flowControlClient_;

        bool bFlowControlEnable_;

        std::uint64_t u64BitRate_;

        Utils::CommonLayerStatistics commonLayerStatistics_;

        void readDevice(int);

        void handleUpstreamControl(const ControlMessages & msgs);
      };
    }
  }
}

#endif // EMANETRANSPORTSVIRTUALVIRTUALTRANSPORT_HEADER_
