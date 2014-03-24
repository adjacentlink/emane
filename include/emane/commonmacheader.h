/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANECOMMONMACHEADER_HEADER_
#define EMANECOMMONMACHEADER_HEADER_

#include "emane/types.h"
#include "emane/frequencysegment.h"
#include "emane/transmitter.h"
#include "emane/downstreampacket.h"
#include "emane/upstreampacket.h"
#include "emane/serializationexception.h"

#include <memory>

namespace EMANE
{
  class CommonMACHeader
  {
  public:
    /**
     * Create a CommonMACHeader
     *
     * @param registrationId Unique MAC registration id
     * @param u64SequenceNumber Packet sequence number
     */
    CommonMACHeader(RegistrationId registrationId, 
                    std::uint64_t u64SequenceNumber);


    /**
     * Creates a CommonMACHeader instance by stripping an UpstreamPacket
     *
     * @param pkt UpstreamPacket that is stripped in order to create the instance
     *
     * @throw SerializationException when the data stripped cannot be de-serialized
     * into a CommonMACPHYHeader
     */
    CommonMACHeader(UpstreamPacket & pkt);

    /**
     * Creates a CommonMACHeader instance by moving another instance
     */
    CommonMACHeader(CommonMACHeader && rvalue);

    /**
     * Destroys an instance
     */
    ~CommonMACHeader();

    /**
     * Gets the MAC registration id
     *
     * @return id
     */
    RegistrationId getRegistrationId() const;
    
    /**
     * Gets the MAC header packet sequence number
     *
     * @return sequence number
     */
    std::uint64_t getSequenceNumber() const;

    /**
     * Prepends CommonMACHeader to downstream packet
     *
     * @param pkt Packet to prepend header
     *
     * @throw SerializationException
     */
    void prependTo(DownstreamPacket & pkt) const;
    
    Strings format() const;

  private:
    class Implementation;
    std::unique_ptr<Implementation>  pImpl_;

    CommonMACHeader(const CommonMACHeader & r) = delete;
    
    CommonMACHeader & 
    operator=(const CommonMACHeader &) = delete;
  };
}

#endif //EMANECOMMONMACHEADER_HEADER_
