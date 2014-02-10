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

#ifndef EMANECONTROLSR2RISELFMETRICCONTROLMESSAGE_HEADER_
#define EMANECONTROLSR2RISELFMETRICCONTROLMESSAGE_HEADER_

#include "emane/types.h"
#include "emane/controlmessage.h"
#include "emane/controls/controlmessageids.h"

#include <memory>

namespace EMANE
{
  namespace Controls
  {
    /**
     * @class R2RISelfMetricControlMessage
     *
     * @brief R2RI Self Metric Control Message is sent to an NEM's transport layer to
     * specify the parameters necessary to interpret R2RI neighbor and queue metric
     * control messages.
     *
     * @note This control message will be encapsulated in a SerializedControlMessage
     * when delivered as via EMANE::DownstreamTransport::processDownstreamControl() or
     * EMANE::UpstreamTransport::processUpstreamControl().
     *
     * @note Instances are immutable
     */ 
   class R2RISelfMetricControlMessage : public ControlMessage
    {
    public:
      /**
       * Creates an R2RISelfMetricControlMessage from a serialization on the heap
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
      R2RISelfMetricControlMessage * create(const Serialization & serialization);

      /**
       * Creates an R2RISelfMetricControlMessage the heap
       *
       * @param u64BroadcastDataRatebps Broadcast datarate in bps
       * @param u64MaxDataRatebps Max unicast datarate in bps
       * @param reportInteral Message report interval in microseconds
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
      R2RISelfMetricControlMessage * create(std::uint64_t u64BroadcastDataRatebps,
                                            std::uint64_t u64MaxDataRatebps,
                                            const Microseconds & reportInteral);
      
      /**
       * Destroys an instance
       */
      ~R2RISelfMetricControlMessage();

      /**
       * Gets the broadcast datarate in bps
       *
       * @return datarate
       */
      std::uint64_t getBroadcastDataRatebps() const;
      
      /**
       * Gets the max unicast datarate in bps
       *
       *@return datarate
       */
      std::uint64_t getMaxDataRatebps() const;
      
      /**
       * Gets the report interval in microseconds
       *
       * @return interval
       */
      const Microseconds & getReportInterval() const;

      /**
       * Serializes the instance
       *
       * @throw SerializationException if the instance cannot be serialized
       */
      Serialization serialize() const override;
      
      enum {IDENTIFIER = EMANE_CONTROL_MEASSGE_R2RI_SELF_METRIC};
      
    private:
      class Implementation;
      std::unique_ptr<Implementation> pImpl_;
      
      
      R2RISelfMetricControlMessage(std::uint64_t u64BroadcastDataRatebps,
                                   std::uint64_t u64MaxDataRatebps,
                                   const Microseconds & tvReportInteral);
      
      R2RISelfMetricControlMessage(const R2RISelfMetricControlMessage &) = delete;
      
      R2RISelfMetricControlMessage & 
      operator=(const R2RISelfMetricControlMessage &) = delete;
    };
  }
}

#endif // EMANECONTROLSR2RISELFMETRICCONTROLMESSAGE_HEADER_
