/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2008-2011 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANEDOWNSTREAMPACKET_HEADER_
#define EMANEDOWNSTREAMPACKET_HEADER_

#include "emane/packetinfo.h"
#include "emane/serializable.h"
#include "emane/utils/vectorio.h"

#include <memory>

namespace EMANE
{
  class Event;

  /**
   * @class DownstreamPacket
   *
   * @brief Specialized packet the allows downstream processing to add layer
   * specific headers as the packet travels down the stack. Optionally, events
   * can be attached to the packet to guarantee delivery before the packet.
   */
  class DownstreamPacket
  {
  public:
    /**
     * Creates a DownstreamPacket instance from a buffer
     *
     * @param info The PacketInfo to use
     * @param buf Pointer to the buffer
     * @param size Length of the buffer
     */
    DownstreamPacket(const PacketInfo & info, const void * buf, size_t size);

    /**
     * Creates a DownstreamPacket instance by copying another instance
     */
    DownstreamPacket(const DownstreamPacket &);

    /**
     * Creates a DownstreamPacket instance by copying another instance
     */
    DownstreamPacket & operator=(const DownstreamPacket & pkt);

    /**
     * Creates a DownstreamPacket instance by moving another instance
     */
    DownstreamPacket(DownstreamPacket && pkt);

    /**
     * Creates a DownstreamPacket instance by moving another instance
     */
    DownstreamPacket & operator=(DownstreamPacket && pkt);

    /**
     * Destroys an instance
     */
    ~DownstreamPacket();
    
    /**
     * Prepends @a bytes of @a buf to the beginning of packet.  This method is used
     * to add layer specific headers to a packet
     *
     * @param buf Pointer to the start of the layer specific data
     * @param size Number of bytes to prepend
     */
    void prepend(const void * buf, size_t size);

    /**
     * Prepends an unsigned 16 bit length value to the beginning of the packet. This
     * method is used in conjunction with prefix length framing.
     *
     * @param u16Length value to be prepended
     *
     * @note Value prepended in network byte order
     */
    void prependLengthPrefixFraming(std::uint16_t u16Length);
    
    /**
     * Gets a vectored IO representation of the packet.
     *
     * @return vectored IO instance 
     *
     * @note Packet data is not copied.
     */
    Utils::VectorIO getVectorIO() const;

    /**
     * Gets the overall packet length which is a summation of all the
     * vectored IO segments.
     *
     * @return size in bytes
     */
    size_t length() const;
    
    /**
     * Get a reference to the packet information
     *
     * @return packet information
     */
    const PacketInfo & getPacketInfo() const;

    /**
     * Attach an event to the packet.
     *
     * @param nemId NEM id of the event target. Use 0 for all NEMs
     * @param event Event to be attached
     */
    void attachEvent(NEMId nemId, const Event & event);

    using EventSerializations = 
      std::list<std::tuple<EMANE::NEMId,EMANE::EventId, Serialization>>;

    /**
     * Gets any attached events in serialized form
     *
     * @return reference to the event serializations
     */
    const EventSerializations & getEventSerializations() const;

  private:
    class Implementation;
    std::unique_ptr<Implementation> pImpl_;
  };
}

#endif // EMANEDOWNSTREAMPACKET_HEADER_
