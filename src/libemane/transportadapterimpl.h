/*
 * Copyright (c) 2013-2015 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2011-2012 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANEAPPLICATIONTANSPORTADAPATERIMPL_HEADER_
#define EMANEAPPLICATIONTANSPORTADAPATERIMPL_HEADER_

#include "boundarymessagemanager.h"
#include "emane/application/transportadapter.h"

#include <memory>

namespace EMANE
{
  namespace Application
  {
    /**
     * @class TransportAdapterImpl
     *
     * @brief Implementation of the Transport Adapter interface. Connects
     *        a Transport implemenation with its respective NEM stack.
     */
    class TransportAdapterImpl : public TransportAdapter,
                                 public BoundaryMessageManager

    {
    public:
      TransportAdapterImpl(NEMId id);

      ~TransportAdapterImpl();

      void initialize(Registrar & registrar) override;

      void configure(const ConfigurationUpdate & update) override;

      void start() override;

      void postStart() override;

      void stop() override;

      void destroy() throw() override;

      void processDownstreamPacket(DownstreamPacket & pkt,
                                   const ControlMessages & msgs);

      void processDownstreamControl(const ControlMessages & msgs);

      void setTransport(std::unique_ptr<Transport> & pTransport) override;

      NEMId getNEMId() const;

    private:
      NEMId id_;
      std::unique_ptr<Transport> pTransport_;
      INETAddr transportEndpointAddr_;
      INETAddr platformEndpointAddr_;

      void doProcessPacketMessage(const PacketInfo &,
                                  const void * pPacketData,
                                  size_t packetLength,
                                  const ControlMessages & msgs);

      void doProcessControlMessage(const ControlMessages & msgs);
    };
  }
}

#endif // EMANEAPPLICATIONTANSPORTADAPATERIMPL_HEADER_
