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

#ifndef EMANEMACLAYERIMPL_HEADER_
#define EMANEMACLAYERIMPL_HEADER_

#include "emane/nemlayer.h"
#include "emane/radioserviceuser.h"
#include "emane/commonmacheader.h"

namespace EMANE
{
  /**
   * @class MACLayerImplementor
   *
   * @brief Interface used to create a MAC layer plugin implementation.
   */
  class MACLayerImplementor : public NEMLayer,
                              public RadioServiceUser
  {
  public:
    /**
     * Destroys an instance
     */
    virtual ~MACLayerImplementor(){}

    /**
     * Process an upstream packet
     *
     * @param hdr  MAC message header
     * @param pkt Upstream packet reference
     * @param msgs Optional control messages
     *
     * @note Control message ownership is transferred with this call.
     * Control messages should not be accessed after this point.
     */
    virtual void processUpstreamPacket(const CommonMACHeader & hdr,
                                       UpstreamPacket & pkt, 
                                       const ControlMessages & msgs = UpstreamTransport::empty) = 0;
    
    /**
     * Sends a downstream packet
     *
     * @param hdr MAC message header
     * @param pkt Downstream packet reference
     * @param msgs Optional control messages
     *
     * @note Control message ownership is transferred with this call.
     * Control messages should not be accessed after this point.
     */
    void sendDownstreamPacket(const CommonMACHeader & hdr,
                              DownstreamPacket & pkt, 
                              const ControlMessages & msgs = DownstreamTransport::empty);

  protected:
    MACLayerImplementor(NEMId id,
                        PlatformServiceProvider * pPlatformServiceProvider,
                        RadioServiceProvider * pRadioServiceProvider):
      NEMLayer{id,pPlatformServiceProvider},
      RadioServiceUser{pRadioServiceProvider}
      {}

  private:
    void processUpstreamPacket(UpstreamPacket & pkt, const ControlMessages & msgs);
    
    void sendDownstreamPacket(DownstreamPacket & pkt, 
                              const ControlMessages & msgs =  DownstreamTransport::empty);
  };
  
  typedef  MACLayerImplementor * (*createMACFunc)(NEMId id, PlatformServiceProvider *p); 
  typedef void (*destroyMACFunc)( MACLayerImplementor*); 
}

#define DECLARE_MAC_LAYER(X)                                                                           \
  extern "C"  EMANE::MACLayerImplementor * create(EMANE::NEMId id,                                     \
                                                  EMANE::PlatformServiceProvider *p,                   \
                                                  EMANE::RadioServiceProvider * r)                     \
  {return new X{id,p,r};}                                               \
  extern "C"  void destroy(EMANE::MACLayerImplementor * p)                                             \
  {delete p;}

#include "emane/maclayerimpl.inl"

#endif //EMANEMACLAYERIMPL_HEADER_
