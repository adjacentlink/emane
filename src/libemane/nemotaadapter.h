/*
 * Copyright (c) 2013-2014,2016 - Adjacent Link LLC, Bridgewater,
 * New Jersey
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

#ifndef EMANENEMOTAADAPTER_HEADER_
#define EMANENEMOTAADAPTER_HEADER_

#include "otauser.h"

#include "emane/downstreamtransport.h"

#include <deque>
#include <thread>
#include <mutex>
#include <condition_variable>

namespace EMANE
{
  /**
   * @class NEMOTAAdapter
   *
   * @brief Over-the-Air adapter handles the intra
   * and inter platform OTA transport
   */
  class NEMOTAAdapter : public OTAUser,
                        public DownstreamTransport
  {
  public:
    NEMOTAAdapter(NEMId id);

    ~NEMOTAAdapter();

    /**
     * process OTA packet
     *
     * @param pkt Downstream packet
     * @param msgs Control Messages
     */

    void processOTAPacket(UpstreamPacket & pkt, const ControlMessages & msgs);

    /**
     *
     * process downstream packet
     *
     * @param pkt reference to the DownstreamPacket
     * @param msgs referenceto the ControlMessage
     *
     */
    void processDownstreamPacket(DownstreamPacket & pkt,
                                 const ControlMessages & msgs);

    void open();

    void close();

  private:
    using DownstreamQueueEntry = std::pair<DownstreamPacket, ControlMessages>;

    using DownstreamPacketQueue = std::deque<DownstreamQueueEntry>;

    NEMId id_;
    std::thread thread_;
    DownstreamPacketQueue queue_;
    std::mutex mutex_;
    std::condition_variable cond_;
    bool bCancel_;

    void processPacketQueue();

    void processDownstreamControl(const ControlMessages &){}
  };
}

#endif //EMANENEMOTAADAPTER_HEADER_
