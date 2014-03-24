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

#ifndef EMANEUPSTREAMPACKET_HEADER_
#define EMANEUPSTREAMPACKET_HEADER_

#include "emane/packetinfo.h"
#include "emane/utils/vectorio.h"

#include <memory>

namespace EMANE
{
  /**
   * @class UpstreamPacket
   *
   * @brief A Packet class that allows upstream processing
   * to strip layer headers as the packet travels up the stack towards
   * the emulation/application boundary (transport).
   */
  class UpstreamPacket
  {
  public:
    /**
     * Creates an UpstreamPacket instance from a buffer
     *
     * @param info The PacketInfo to use
     * @param buf Pointer to the buffer
     * @param len Length of the buffer
     */
    UpstreamPacket(const PacketInfo & info,const void * buf,size_t len);
    
    /**
     * Creates an UpstreamPacket instance from a Utils::VectorIO
     *
     * @param info The PacketInfo to use
     * @param vectorIO Vectored IO object
     *
     * @note A deep copy is performed and the internal packet data
     * is stored contiguously
     */
    UpstreamPacket(const PacketInfo & info,const Utils::VectorIO & vectorIO);

    /**
     * Creates an UpstreamPacket instance by copying another instance
     */
    UpstreamPacket(const UpstreamPacket &);

    /**
     * Sets an UpstreamPacket instance to a copy of another instance
     */
    UpstreamPacket & operator=(const UpstreamPacket & pkt);


    /**
     * Creates an UpstreamPacket instance by moving another instance
     */
    UpstreamPacket(UpstreamPacket && pkt);

    /**
     * Sets and UpstreamPacket instance by moving another instance
     */
    UpstreamPacket & operator=(UpstreamPacket && pkt);


    /**
     * Destroys an UpstreamPacket instance
     */
    ~UpstreamPacket();
    
    /**
     * Removes @a size bytes from the beginning of a packet.  This method is used to 
     * remove layer specific headers from a packet after they are processed.
     *
     * @param size Number of bytes to strip from the head of the packet.
     *
     * @return bytes stripped
     */
    size_t strip(size_t size);


    /**
     * Removes 2 bytes for the beginning of the packet and returns them
     * as an unsigned 16-bit integer in host byte order.
     *
     * @return The prefixed length value
     */
    std::uint16_t stripLengthPrefixFraming();

    /**
     * Gets a pointer to the internal buffer holding the message
     *
     * @return @c Pointer to message or @c 0 if the packet was not combined or 
     * if there is no data
     *
     */
    const void * get() const;
    
    /**
     * Gets the packet length in bytes
     *
     * @return length in bytes
     */
    size_t length() const;
    
    /**
     * Gets a reference to the packet information
     *
     * @return packet information
     */
    const PacketInfo & getPacketInfo() const;
    
  private:
    class Implementation;
    std::unique_ptr<Implementation> pImpl_;
  };
}

#endif // EMANEDOWNSTREAMPACKET_HEADER_
