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

#ifndef EMANECONTROLSOTATRANSMISSIONCONTROLMESSAGE_HEDAER_
#define EMANECONTROLSOTATRANSMISSIONCONTROLMESSAGE_HEDAER_

#include "emane/controlmessage.h"
#include "emane/controls/controlmessageids.h"

#include <memory>
#include <set>

namespace EMANE
{
  namespace Controls
  {
    using OTATransmitters = std::set<NEMId>;
    
    /**
     * @class OTATransmitterControlMessage
     *
     * @brief The OTA Transmitter Control Message is by the emulator physical
     * layer to specify the NEM Id of the source or sources transmitting
     * the over-the-air message. The OTAManager needs to be aware of each transmitter
     * that is part of a collaborative transmission in order to prevent delivery of the 
     * message to a transmitter.
     *
     * @note Instances are immutable
     */

    class OTATransmitterControlMessage : public ControlMessage
    {
    public:
      /**
       * Creates an OTATransmitterControlMessage instance from a serialization on the heap
       *
       * @param serialization Message serialization
       *
       * @throw SerializationException when a valid message cannot be de-serialized
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
      OTATransmitterControlMessage * create(const Serialization & serialization);

      /**
       * Creates an OTATransmitterControlMessage instance on the heap
       *
       * @param transmitters Message Transmitters
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
      OTATransmitterControlMessage * create(const OTATransmitters & transmitters);

      /**
       * Destroys an instance
       */
      ~OTATransmitterControlMessage();
      
      /**
       * Gets the OTA transmitters
       *
       * @return transmitters
       */
      const OTATransmitters & getOTATransmitters() const;
      
      /**
       * Serializes the instance
       *
       * @throw SerializationException if the instance cannot be serialized
       */    
      std::string serialize() const override;
      
      enum {IDENTIFIER = EMANE_CONTROL_MEASSGE_OTA_TRANSMITTER};
      
    private:
      class Implementation;
      std::unique_ptr<Implementation> pImpl_;
      
      OTATransmitterControlMessage(const OTATransmitters & transmitters);
      
      OTATransmitterControlMessage(const OTATransmitterControlMessage &) = delete;
      
      OTATransmitterControlMessage & 
      operator=(const OTATransmitterControlMessage &) = delete;
    };
  }
}

#endif // EMANECONTROLSOTATRANSMISSIONCONTROLMESSAGE_HEDAER_
