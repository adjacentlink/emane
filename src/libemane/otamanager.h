/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANEOTAMANAGER_HEADER_
#define EMANEOTAMANAGER_HEADER_

#include "otaprovider.h"
#include "emane/utils/singleton.h"

#include <map>
#include <queue>
#include <atomic>

#include <uuid.h>

#include <ace/Thread.h>
#include <ace/SOCK_Dgram_Mcast.h>

namespace EMANE
{
  /**
   * @class OTAManager
   *
   * @brief Provides OTA access to all platform NEMs and
   * handles intra and inter platform OTA message dissemination
   */
  class OTAManager : public OTAProvider,
                     public Utils::Singleton<OTAManager>
  {
  public:
    OTAManager();
    
    ~OTAManager();
   
    /**
     * Send OTA packet
     *
     * @param id NEM identifier
     * @param pkt Downstream packet
     * @param msg Control Message
     */
    void sendOTAPacket(NEMId id,
                       DownstreamPacket & pkt,
                       const ControlMessages & msgs) const override;
    
    void registerOTAUser(NEMId id, OTAUser * pOTAUser) override;

    void unregisterOTAUser(NEMId id) override;

    /**
     * Open the event server channel
     *
     * @param otaGroupAddress Multicast group address of event service OTA channel
     * @param otaManagerDevice Name of the OTA device
     */
    void open(const ACE_INET_Addr & otaGroupAddress,
              const ACE_TCHAR* otaManagerDevice,
              bool bLoopback,
              int iTTL,
              const uuid_t & uuid);
    
  private:
    typedef std::map<NEMId,OTAUser *> NEMUserMap;
    ACE_thread_t thread_;
    NEMUserMap nemUserMap_;
    ACE_INET_Addr otaGroupAddress_;
    ACE_SOCK_Dgram_Mcast mcast_;
    bool bOpen_;
    ACE_INET_Addr addr_;
    uuid_t uuid_;
    mutable std::atomic<std::uint64_t> u64SequenceNumber_;

    ACE_THR_FUNC_RETURN processOTAMessage();
  };

  using OTAManagerSingleton = OTAManager;
}

#endif //EMANEOTAMANAGER_HEADER_
