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

#ifndef EMANEUPSTREAMTRANSPORT_HEADER_
#define EMANEUPSTREAMTRANSPORT_HEADER_

#include "emane/controlmessage.h"
#include "emane/upstreampacket.h"
#include "emane/downstreampacket.h"

namespace EMANE
{
  class DownstreamTransport;

  /**
   * @class UpstreamTransport
   *
   * @brief UpstreamTransport allows for processing upstream data and 
   * control messages. Upstream packets and control messages are placed
   * on the NEMQueuedLayer functor queue for processing.
   */
  class UpstreamTransport
  {
  public:
    virtual ~UpstreamTransport(){}

    /**
     * Process upstream packet
     *
     * @param pkt reference to the UpstreamPacket to process
     * @param msgs optional reference to the ControlMessages
     */
    virtual void processUpstreamPacket(UpstreamPacket & pkt, 
                                       const ControlMessages & msgs = empty) = 0;
    /**
     * Process upstream control message
     *
     * @param msgs reference to the ControlMessages
     * 
     */
    virtual void processUpstreamControl(const ControlMessages & msgs) = 0;
    

    /**
     * Set the downstream transport.
     *
     * @param pDownstreamTransport Pointer to the downstream transport
     * of this upstream transport.
     *
     * @note: This method is for internal framework use only.
     */
    virtual void setDownstreamTransport(DownstreamTransport * pDownstreamTransport)
    {
      pDownstreamTransport_ = pDownstreamTransport; 
    }


    /**
     * Send downsteam packet
     *
     * @param pkt reference to the DownstreamPacket to process
     * @param msgs optional reference to the ControlMessages
     */
    void sendDownstreamPacket(DownstreamPacket & pkt, 
                              const ControlMessages & msgs = empty);
    

    /**
     * Send downstream control message
     *
     * @param msg reference to the ControlMessage
     * 
     */
    void sendDownstreamControl(const ControlMessages & msgs);

    static const ControlMessages empty;

  protected:

    UpstreamTransport():
      pDownstreamTransport_(0){}

                             
  private:
    DownstreamTransport * pDownstreamTransport_;
  };
}

#include "emane/upstreamtransport.inl"

#endif //EMANEUPSTREAMTRANSPORT_HEADER_
