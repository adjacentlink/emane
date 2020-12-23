/*
 * Copyright (c) 2020 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANECONTROLSMIMOTRANSMITPROPERTIESMESSAGE_HEADER_
#define EMANECONTROLSMIMOTRANSMITPROPERTIESMESSAGE_HEADER_

#include "emane/controlmessage.h"
#include "emane/types.h"
#include "emane/frequencysegment.h"
#include "emane/antenna.h"
#include "emane/controls/controlmessageids.h"

#include <memory>

namespace EMANE
{
  namespace Controls
  {
    /**
     * @class MIMOTransmitPropertiesControlMessage
     *
     * @brief MIMO Transmit Properties Control Message is sent to the
     * emulator physical layer with every downstream packet when
     * compatibility mode > 1. It contains antenna transmit
     * information.
     *
     * @note The Start of Transmission (SoT) is the tx time  + propagation delay + offset
     * of the first frequency segment.
     *
     * @note Instances are immutable
     */
    class MIMOTransmitPropertiesControlMessage : public ControlMessage
    {
    public:
      /**
       * Creates a MIMOTransmitPropertiesControlMessage on the heap
       *
       * @param frequencyGroups Frequency groups to use
       * @param transmitAntennas Transmit antenna to use
       *
       * @note Once a control message is passed to another NEM layer using
       * EMANE::UpstreamTransport::processUpstreamPacket(),
       * EMANE::UpstreamTransport::processUpstreamControl(),
       * EMANE::DownstreamTransport::processDownstreamPacket() or
       * EMANE::DownstreamTransport::processDownstreamControl() object ownership is
       * transferred to the emulator infrastructure along with deallocation responsibility.
       * It is not valid to use a control message instance after it has been passed to another
       * layer.
       */
      static
      MIMOTransmitPropertiesControlMessage * create(const FrequencyGroups & frequencyGroups,
                                                    const Antennas & transmitAntennas);

      static
      MIMOTransmitPropertiesControlMessage * create(FrequencyGroups && frequencyGroups,
                                                    const Antennas & transmitAntennas);

      static
      MIMOTransmitPropertiesControlMessage * create(FrequencyGroups && frequencyGroups,
                                                    Antennas && transmitAntennas);

      /**
       * Clones the control message on the heap
       *
       * @return cloned message
       *
       * @note Caller assumes ownership of the clone
       */
      MIMOTransmitPropertiesControlMessage * clone() const override;

      /**
       * Destroys an instance
       */
      ~MIMOTransmitPropertiesControlMessage();

      /**
       * Gets the frequency groups
       *
       * @return time
       */
      const FrequencyGroups & getFrequencyGroups() const;


      /**
       * Gets the transmit antennas
       *
       * @return propagation delay in microseconds
       */
      const Antennas & getTransmitAntennas() const;


      enum {IDENTIFIER = EMANE_CONTROL_MEASSGE_MIMO_TRANSMIT_PROPERTIES};

    private:
      class Implementation;
      std::shared_ptr<Implementation> pImpl_;

      MIMOTransmitPropertiesControlMessage(const FrequencyGroups & frequencyGroups,
                                           const Antennas & transmitAntennas);

      MIMOTransmitPropertiesControlMessage(FrequencyGroups && frequencyGroups,
                                           const Antennas & transmitAntennas);

      MIMOTransmitPropertiesControlMessage(FrequencyGroups && frequencyGroups,
                                           Antennas && transmitAntennas);

      MIMOTransmitPropertiesControlMessage(const MIMOTransmitPropertiesControlMessage &);

      MIMOTransmitPropertiesControlMessage &
      operator=(const MIMOTransmitPropertiesControlMessage &) = delete;
    };
  }
}

#endif // EMANECONTROLSMIMOTRANSMITPROPERTIESMESSAGE_HEADER_
