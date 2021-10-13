/*
 * Copyright (c) 2013-2014,2020 - Adjacent Link LLC, Bridgewater,
 * New Jersey
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

#ifndef EMANECOMMONPHYHEADER_HEADER_
#define EMANECOMMONPHYHEADER_HEADER_

#include "emane/types.h"
#include "emane/transmitter.h"
#include "emane/frequencysegment.h"
#include "emane/downstreampacket.h"
#include "emane/upstreampacket.h"
#include "emane/serializationexception.h"
#include "emane/antenna.h"

#include <memory>

namespace EMANE
{
  /**
   * @class CommonPHYHeader
   *
   * @brief The common physical layer header used to facilitate heterogeneous
   * radio model experimentation
   */

  using Durations = std::vector<Microseconds>;

  class CommonPHYHeader
  {
  public:
    /**
     * Creates a CommonPHYHeader instance
     *
     * @param registrationId Registration id of the physical layer
     * @param subId Sub id used to delineate different waveforms
     * @param u16SequenceNumber Sequence number
     * @param txTime Transmit time stamp
     * @param frequencyGroups Frequency segment groups
     * @param transmitAntennas List of transmit antennas
     * @param transmitters List of transmitters
     * @param optionalFilterData Optional filter data
     *
     * @note The transmit time stamp is used as the start of transmission (SoT)
     * time at the receiver. If servers used during the emulation are not tightly
     * time synced the receiver will use message reception time as the SoT.
     *
     * @note MAC layers can use the Controls::TimeStampControlMessage to set the txTime.
     */
    CommonPHYHeader(RegistrationId registrationId,
                    std::uint16_t subId,
                    std::uint16_t u16SequenceNumber,
                    const TimePoint & txTime,
                    const FrequencyGroups & frequencyGroups,
                    const Antennas & transmitAntennas,
                    const Transmitters & transmitters,
                    const std::pair<FilterData,bool> & optionalFilterData);

    /**
     * Creates a CommonPHYHeader instance by stripping an UpstreamPacket
     *
     * @param pkt UpstreamPacket that is stripped in order to create the instance
     *
     * @throw SerializationException when the data stripped cannot be de-serialized
     * into a CommonPHYHeader
     */
    CommonPHYHeader(UpstreamPacket & pkt);


    /**
     * Creates a CommonPHYHeader instance by moving another instance
     *
     */
    CommonPHYHeader(CommonPHYHeader && rvalue);

    /**
     * Destroys an instance
     */
    ~CommonPHYHeader();

    /**
     * Gets the physical layer registration id
     *
     * @return registration id
     */
    RegistrationId getRegistrationId() const;

    /**
     * Gets the sub id
     *
     * @return sub id
     */
    std::uint16_t getSubId() const;

    /**
     * Gets the optional filter data
     *
     * @return optional filter data as a pair, where @c first is the
     * data and @c second is a boolean flag indicating whether
     * the data is valid (present)
     *
     */
    const std::pair<FilterData,bool> & getOptionalFilterData() const;

    /**
     * Gets the transmission time stamp
     *
     * @return transmission time stamp
     */
    const TimePoint & getTxTime() const;

    /**
     * Gets the message duration
     *
     * @return message duration
     *
     * @note Duration is determined by taking the delta between the
     * earliest frequency segment offset and the latest ending frequency
     * segment.
     */
    Durations getDurations() const;

    /**
     * Gets the sequence number
     *
     * @return sequence number
     */
    std::uint16_t getSequenceNumber() const;

    /**
     * Gets a reference to the antenna frequency segment list
     *
     * @return frequency segments
     */
    const FrequencyGroups & getFrequencyGroups() const;

    /**
     * Gets a reference to the transmit antenna list
     *
     * @return transmit antennas
     */
    const Antennas & getTransmitAntennas() const;

    /**
     * Gets a reference to the transmitters list
     *
     * @return transmitters list
     */
    const Transmitters & getTransmitters() const;

    /**
     * Prepends CommonPHYHeader to downstream packet
     *
     * @param pkt Packet to prepend header
     *
     * @throw SerializationException
     */
    void prependTo(DownstreamPacket & pkt) const;

    /**
     * Returns format suitable for logger callable
     *
     * @return format strings
     */
    Strings format() const;

  private:
    class Implementation;
    std::unique_ptr<Implementation>  pImpl_;

    CommonPHYHeader(const CommonPHYHeader & r) = delete;

    CommonPHYHeader &
    operator=(const CommonPHYHeader &) = delete;
  };
}

#endif //EMANECOMMONPHYHEADER_HEADER_
