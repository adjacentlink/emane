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

#ifndef EMANECONTROLSFLOWCONTROLCONTROLMESSAGE_HEDAER_
#define EMANECONTROLSFLOWCONTROLCONTROLMESSAGE_HEDAER_

#include "emane/controlmessage.h"
#include "emane/controls/controlmessageids.h"

#include <cstdint>
#include <memory>

namespace EMANE
{
  namespace Controls
  {
    /**
     * @class FlowControlControlMessage
     *
     * @brief Flow Control Control Messages are sent between a MAC layer
     * and a transport in order to communicate data rate and congestion.
     *
     * @note This control message will be encapsulated in a SerializedControlMessage
     * when delivered as via EMANE::DownstreamTransport::processDownstreamControl() or
     * EMANE::UpstreamTransport::processUpstreamControl().
     *
     * @note Instances are immutable
     */
    class FlowControlControlMessage : public ControlMessage
    {
    public:
      /**
       * Creates a FlowControlControlMessage instance from a serialization on the heap
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
      FlowControlControlMessage * create(const Serialization & serialization);
      
      /**
       * Creates a FlowControlControlMessage instance on the heap
       *
       * @param u16Tokens The current number of tokens
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
      FlowControlControlMessage * create(std::uint16_t u16Tokens);


      /**
       * Clones the control message on the heap
       *
       * @return cloned message
       *
       * @note Caller assumes ownership of the clone
       */
      FlowControlControlMessage * clone() const override;
      
      /**
       * Destroys an instance
       */
      ~FlowControlControlMessage();
      
      /**
       * Gets the token count
       */
      std::uint16_t getTokens() const;

      /**
       * Serializes the instance
       *
       * @throw SerializationException if the instance cannot be serialized
       */
      Serialization serialize() const override;
      
      enum {IDENTIFIER = EMANE_CONTROL_MEASSGE_FLOW_CONTROL};
      
    private:
      class Implementation;
      std::unique_ptr<Implementation> pImpl_;
      
      FlowControlControlMessage(std::uint16_t u16Tokens);
      
      FlowControlControlMessage(const FlowControlControlMessage & r);
      
      FlowControlControlMessage & 
      operator=(const FlowControlControlMessage &) = delete;
      
    };
  }
}

#endif // EMANECONTROLSFLOWCONTROLCONTROLMESSAGE_HEDAER_
