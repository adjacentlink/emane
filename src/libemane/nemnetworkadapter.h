/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2008-2012 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANENEMNETWORKADAPTER_HEADER_
#define EMANENEMNETWORKADAPTER_HEADER_

#include "networkadapterexception.h"
#include "boundarymessagemanager.h"

#include "emane/upstreamtransport.h"

namespace EMANE
{
  /**
   * @class NEMNetworkAdapter
   *
   * @brief Entry point for packets and crontrol sourced and sinked
   * from the network layer
   */
  class NEMNetworkAdapter : public UpstreamTransport,
                            public BoundaryMessageManager
  {
  public:
    NEMNetworkAdapter(NEMId id);
    
    ~NEMNetworkAdapter();
    
    void processUpstreamPacket(UpstreamPacket & pkt,
                               const ControlMessages & msgs) override;
    
    void processUpstreamControl(const ControlMessages & msgs) override;

  private:
    void doProcessPacketMessage(const PacketInfo &,
                                const void * pPacketData,
                                size_t packetLength,
                                const ControlMessages & msgs) override;
    
    void doProcessControlMessage(const ControlMessages & msgs) override;
  };
}

#endif //EMANENETNETWORKOTAADAPTER_HEADER_
