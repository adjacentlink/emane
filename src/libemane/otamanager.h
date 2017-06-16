/*
 * Copyright (c) 2013-2017 - Adjacent Link LLC, Bridgewater, New Jersey
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
#include "emane/inetaddr.h"
#include "multicastsocket.h"
#include "otastatisticpublisher.h"
#include "eventstatisticpublisher.h"

#include "emane/utils/singleton.h"

#include <map>
#include <queue>
#include <atomic>
#include <thread>
#include <tuple>
#include <array>
#include <uuid.h>


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

    void sendOTAPacket(NEMId id,
                       const DownstreamPacket & pkt,
                       const ControlMessages & msgs) const override;

    void registerOTAUser(NEMId id, OTAUser * pOTAUser) override;

    void unregisterOTAUser(NEMId id) override;

    /**
     * Open the event server channel
     *
     * @param otaGroupAddress Multicast group address of event service OTA channel
     * @param sDevice Name of the OTA device
     * @param bLoopback Flag indicating whether to set LOOPBACK socket option
     * @param iTTL Mutlicast TTL
     * @param uuid Emulator instance UUID
     */
    void open(const INETAddr & otaGroupAddress,
              const std::string & sDevice,
              bool bLoopback,
              int iTTL,
              const uuid_t & uuid,
              size_t otaMTU,
              Seconds partCheckThreshold,
              Seconds partTimeoutThreshold);

    void setStatPacketCountRowLimit(size_t rows);

    void setStatEventCountRowLimit(size_t rows);

  private:
    typedef std::map<NEMId,OTAUser *> NEMUserMap;
    std::thread thread_;
    NEMUserMap nemUserMap_;
    INETAddr otaGroupAddress_;
    MulticastSocket mcast_;
    bool bOpen_;
    uuid_t uuid_;
    size_t otaMTU_;
    Seconds partCheckThreshold_;
    Seconds partTimeoutThreshold_;

    mutable OTAStatisticPublisher otaStatisticPublisher_;
    mutable EventStatisticPublisher eventStatisticPublisher_;
    mutable std::atomic<std::uint64_t> u64SequenceNumber_;

    using PartKey = std::tuple<NEMId, // source NEM
                               std::uint64_t>; // sequence number

    using Parts = std::map<size_t, // offset
                           std::vector<std::uint8_t>>;

    using PartsData = std::tuple<size_t, // total size in map
                                 size_t, // events size bytes
                                 size_t, // controls size bytes
                                 size_t, // data size bytes
                                 Parts, // parts map
                                 TimePoint, // last part time
                                 std::array<uint8_t,sizeof(uuid_t)>>; // emulator uuid instance

    using PartStore = std::map<PartKey,PartsData>;
    PartStore partStore_;
    TimePoint lastPartCheckTime_;


    void processOTAMessage();

    void handleOTAMessage(NEMId source,
                          NEMId destination,
                          const uuid_t & remoteUUID,
                          const TimePoint & now,
                          size_t eventsSize,
                          size_t controlsSize,
                          size_t dataSize,
                          const Utils::VectorIO & vectorIO);
  };

  using OTAManagerSingleton = OTAManager;
}

#endif //EMANEOTAMANAGER_HEADER_
