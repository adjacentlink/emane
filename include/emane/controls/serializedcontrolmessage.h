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

#ifndef EMANECONTROLSSERIALIZEDCONTROLMESSAGE_HEADER_
#define EMANECONTROLSSERIALIZEDCONTROLMESSAGE_HEADER_

#include "emane/controlmessage.h"
#include "emane/controls/controlmessageids.h"

#include <memory>

namespace EMANE
{
  namespace Controls
  { 
    /**
     * @class SerializedControlMessage
     *
     * @brief A Serialized Control Message is used to encapsulate Serializable
     * control messages as they traverse process boundaries via a network connection.
     *
     * @note Instances are immutable
     *
     * @details Encapulated control messages must be restored before use:
     * @snippet models/mac/rfpipe/maclayer.cc serializedcontrolmessage-flowcontrol-snibbet
     *
     */
    class SerializedControlMessage : public ControlMessage
    {
    public:
      /**
       * Creates a SerializedControlMessage instance on the heap
       *
       * @param id Control message id of the message being serialized
       * @param pData Pointer to the serialized data
       * @param length Serialized data length
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
      SerializedControlMessage * create(ControlMessageId id,
                                        const void * pData,
                                        size_t length);

      /**
       * Clones the control message on the heap
       *
       * @return cloned message
       *
       * @note Caller assumes ownership of the clone
       */
      SerializedControlMessage * clone() const override;

      /**
       * Destroys an instance
       */
      ~SerializedControlMessage();

      /**
       * Gets the control message id of the serialized message
       *
       * @return control message id
       */
      ControlMessageId getSerializedId() const;
      
      /**
       * Gets the message serialization
       *
       * @return serialization
       */
      std::string getSerialization() const;
      
      enum {IDENTIFIER = EMANE_CONTROL_MEASSGE_SERIALIZED};
      
    private:
      class Implementation;
      std::unique_ptr<Implementation> pImpl_;
      
      SerializedControlMessage(ControlMessageId id,
                               const void * pData,
                               size_t length);

      SerializedControlMessage(const SerializedControlMessage &);
      
      SerializedControlMessage & 
      operator=(const SerializedControlMessage &) = delete;
    };
  }
}

#endif // EMANECONTROLSSERIALIZEDCONTROLMESSAGE_HEADER_
