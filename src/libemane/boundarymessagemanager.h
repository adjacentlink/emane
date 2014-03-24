/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANEBOUNDARYMESSAGEMANAGER_HEADER_
#define EMANEBOUNDARYMESSAGEMANAGER_HEADER_

#include "emane/packetinfo.h"
#include "emane/controlmessage.h"
#include "emane/types.h"
#include "emane/utils/vectorio.h"

#include <ace/SOCK_Dgram.h>
#include <ace/Thread.h>

namespace EMANE
{
  class BoundaryMessageManager
  {
  public:
    virtual ~BoundaryMessageManager();

    /**
     * @throw BoundaryMessageManagerException
     */
    void open(const ACE_INET_Addr & localAddress,
              const ACE_INET_Addr & remoteAddress);
    
    void close();
    
    void processBoundaryMessage(const void * pData,
                                size_t length);
    
    void sendPacketMessage(const PacketInfo & packetInfo,
                           const void * pPacketData,
                           size_t packetLength,
                           const ControlMessages & msgs);
    
    void sendPacketMessage(const PacketInfo & packetInfo,
                           const Utils::VectorIO & packetIO,
                           size_t overallLength,
                           const ControlMessages & msgs);
      
    void sendControlMessage(const ControlMessages & msgs);
    
  protected:
    BoundaryMessageManager(NEMId id);
    
    virtual void doProcessPacketMessage(const PacketInfo &,
                                        const void * pPacketData,
                                        size_t packetLength,
                                        const ControlMessages & msgs) = 0;
    
    virtual void doProcessControlMessage(const ControlMessages & msgs) = 0;

  private:
    NEMId id_;
    ACE_SOCK_Dgram udp_;
    ACE_INET_Addr localAddress_;
    ACE_INET_Addr remoteAddress_;
    ACE_thread_t thread_;
    bool bOpen_;
    ACE_INET_Addr addr_;
    
    ACE_THR_FUNC_RETURN processNetworkMessage();
  };
}

#endif //EMANEBOUNDARYMESSAGEMANAGER_HEADER_
