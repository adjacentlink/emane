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

#ifndef EMANEPHYLAYERIMPL_HEADER_
#define EMANEPHYLAYERIMPL_HEADER_

#include "emane/nemlayer.h"
#include "emane/commonphyheader.h"
#include "emane/radioserviceuser.h"

namespace EMANE
{
  /**
   * @class PHYLayerImplementor
   *
   * @brief Interface used to create a PHY layer plugin implementation.
   *
   * @deprecated For almost all use cases physical layer plugin creation
   * should be avoided in favor of using the emulator's physical layer.
   *
   * @attention Need additional features added to the emulator's physical layer?
   * Send an email to emane-info at adjacentlink dot com.
   */
  class PHYLayerImplementor : public NEMLayer,
                              public RadioServiceUser
  {
  public:
    virtual ~PHYLayerImplementor(){}

    /**
     * Process an upstream packet
     *
     * @param hdr  PHY message header
     * @param pkt Upstream packet reference
     * @param msgs Optional control messages
     *
     * @note Control message ownership is transferred with this call.
     * Control messages should not be accessed after this point.
     */

    virtual void processUpstreamPacket(const CommonPHYHeader & hdr,
                                       UpstreamPacket & pkt,
                                       const ControlMessages & msgs = UpstreamTransport::empty) = 0;

    /**
     * Sends a downstream packet
     *
     * @param hdr PHY message header
     * @param pkt Downstream packet reference
     * @param msgs Optional control messages
     *
     * @note Control message ownership is transferred with this call.
     * Control messages should not be accessed after this point.
     */
    void sendDownstreamPacket(const CommonPHYHeader & hdr,
                              DownstreamPacket & pkt,
                              const ControlMessages & msgs  = DownstreamTransport::empty);

  protected:
    PHYLayerImplementor(NEMId id,
                        PlatformServiceProvider *p,
                        RadioServiceProvider * pRadioServiceProvider):
      NEMLayer(id, p),
      RadioServiceUser{pRadioServiceProvider}
   {}

  private:
    // method meaningless for a PHY layer
    void processUpstreamControl(const ControlMessages &){}

    void processUpstreamPacket(UpstreamPacket & pkt, const ControlMessages & msgs);

    void sendDownstreamPacket(DownstreamPacket & pkt,
                              const ControlMessages & msgs = DownstreamTransport::empty);

  };

  typedef  PHYLayerImplementor * (*createPHYFunc)(NEMId id, PlatformServiceProvider *p);
  typedef void (*destroyPHYFunc)( PHYLayerImplementor*);
}

#define DECLARE_PHY_LAYER(X)                                                                           \
  extern "C"  EMANE::PHYLayerImplementor * create(EMANE::NEMId id,                                     \
                                                  EMANE::PlatformServiceProvider *p,                   \
                                                  EMANE::RadioServiceProvider * r)                     \
  {return new X(id,p,r);}                                                                                \
  extern "C"  void destroy(EMANE::PHYLayerImplementor * p)                                             \
  {delete p;}

#include "emane/phylayerimpl.inl"

#endif //EMANEPHYLAYERIMPL_HEADER_
