/*
 * Copyright (c) 2026 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANETRANSPORTSTRAFFICTRANSPORT_HEADER_
#define EMANETRANSPORTSTRAFFICTRANSPORT_HEADER_

#include "session.h"

#include "emane/transport.h"
#include "emane/utils/raiizmq.h"

#include <vector>
#include <map>

namespace EMANE
{
  namespace Transports
  {
    namespace Traffic
    {
      class Transport : public EMANE::Transport
      {
      public:
        Transport(NEMId id,
                  PlatformServiceProvider *pPlatformService);

        ~Transport();

        void initialize(Registrar & registrar) override;

        void configure(const ConfigurationUpdate & update) override;

        void start() override;

        void postStart() override;

        void stop() override;

        void destroy() throw() override;

        void processUpstreamPacket(UpstreamPacket & pkt,
                                   const ControlMessages & msgs) override;

        void processUpstreamControl(const ControlMessages & msgs) override;

      private:
        INETAddr listenEndpoint_;

        using LabelToNEMMap = std::map<std::string,NEMId>;
        using NEMToLabelMap = std::map<NEMId,std::string>;
        LabelToNEMMap labelToNEMMap_;
        NEMToLabelMap nemToLabelMap_;
        std::string sLocalLabel_;

        Utils::RAIIZMQContext pZMQContext_;
        Utils::RAIIZMQSocket pStreamSocket_;

        using StreamIdentity = std::vector<std::uint8_t>;
        StreamIdentity remoteStreamIdentity_;
        int iStreamSocketFd_;
        Session session_;
        std::uint64_t u64SequenceNumber_;

        void process(int);
      };
    }
  }
}

#endif // EMANETRANSPORTSTRAFFICTRANSPORT_HEADER_
