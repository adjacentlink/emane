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

#ifndef EMANEPACKETINFO_HEADER_
#define EMANEPACKETINFO_HEADER_

#include "emane/types.h"
#include <uuid.h>

namespace EMANE
{
  /**
   * @class PacketInfo
   *
   * @brief Store source, destination, creation time and priority
   * information for a packet.
   *
   * @note Instances are immutable
   */
  class PacketInfo
  {
  public:
    /**
     * Creates a PacketInfo instance
     *
     * @param source The src NEM
     * @param destination The destination NEM
     * @param priority The priority
     * @param creationTime Creation time of the packet
     */
    PacketInfo(NEMId source,
               NEMId destination,
               Priority priority,
               TimePoint creationTime);

    /**
     * Creates a PacketInfo instance
     *
     * @param source The src NEM
     * @param destination The destination NEM
     * @param priority The priority
     * @param creationTime Creation time of the packet
     * @param uuid Application UUID (upstream only)
     */
    PacketInfo(NEMId source,
               NEMId destination,
               Priority priority,
               TimePoint creationTime,
               const uuid_t & uuid);

    
    /**
     * Destroys an instance
     */
    ~PacketInfo();

    /**
     * Gets the source
     *
     * @return souce NEM id
     */
    NEMId getSource() const;

    /**
     * Gets the destination
     * 
     * @return Destination NEM
     *
     * @note May be NEM_BROADCAST_MAC_ADDRESS
     */
    NEMId getDestination() const;

    /**
     * Gets the priority
     *
     * @ return prioroty
     */
    Priority getPriority() const;
    
    /**
     * Gets the creation time
     *
     * @return creation time
     */
    TimePoint getCreationTime() const;
    
    /**
     * Gets the application UUID
     *
     * @return uuid
     */
    const uuid_t & getUUID() const;
    
  private:
    NEMId source_;      
    NEMId destination_;
    Priority priority_; 
    TimePoint creationTime_;
    uuid_t uuid_;
  };
}

#include "emane/packetinfo.inl"

#endif //EMANEPACKETINFO_HEADER_ 
