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

#ifndef EMANECONTROLSRECEIVEPROPERTIESMESSAGE_HEADER_
#define EMANECONTROLSRECEIVEPROPERTIESMESSAGE_HEADER_

#include "emane/controlmessage.h"
#include "emane/controls/controlmessageids.h"
#include "emane/frequencysegment.h"

#include <memory>
#include <cstdint>

namespace EMANE
{
  namespace Controls
  {
    /**
     * @class ReceivePropertiesControlMessage
     *
     * @brief Recieve Properties Control Message is sent from the emulator physical
     * layer with every upstream packet. It contains the tx time stamp, propagation
     * delay, span and reciever sensitivity associated with the packet and the reciever.
     *
     * @note The Start of Transmission (SoT) is the tx time  + propagation delay + offset
     * of the first frequency segment.
     *
     * @note Instances are immutable
     */
    class ReceivePropertiesControlMessage : public ControlMessage
    {
    public:
      /**
       * Creates a ReceivePropertiesControlMessage on the heap
       *
       * @param txTime Time value used as the tx time in the common PHY Header
       * @param propagation Propagation delay in microseconds
       * @param span Length of time in microseconds between the earliest
       * start-of-reception (SoR) frequency segment and latest end-of-reception (EoR) segment.
       * @param dReceiverSensitivitydBm Receiver sensitivity in dBm
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
      ReceivePropertiesControlMessage * create(const TimePoint & txTime,
                                               const Microseconds & propagation,
                                               const Microseconds & span,
                                               double dReceiverSensitivitydBm);

      /**
       * Destroys an instance
       */
      ~ReceivePropertiesControlMessage();
      
      /**
       * Gets the Tx time
       *
       * @return time
       */
      TimePoint getTxTime() const;

      
      /**
       * Gets the propagation delay in microseconds
       *
       * @return propagation delay in microseconds
       */
      Microseconds getPropagationDelay() const;

      /**
       * Gets the message span
       *
       * @return message span in microseconds
       *
       * @note The message span is the length of time between the earliest
       * start-of-reception (SoR) frequency segment and latest end-of-reception (EoR)
       * segment. The purpose of the span is to provide a simple mechanism to query
       * the spectrum service for the noise window that is guaranteed to cover the total
       * duration of all the segments combined. The span will "span" any gaps between
       * segments allowing for creating a logical two dimensional matrix of noise,
       * frequencies vs time, for quick access to the noise information occurring during
       * the entire message duration.
       *
       * @note The main consideration is that one large noise window request is 
       * less expensive than many small requests.
       *
       * @note The span is the length of time after transmit time, propagation delay and
       * segment offset have been accounted for.
       *
       * @note Since the first frequency segment always contains the earliest offset, a query
       * to the Spectrum Service should use a calcualted SoR time and the span as the query duration:
       *
       * @snippet models/mac/rfpipe/maclayer.cc startofreception-calculation-snibbet
       */
      Microseconds getSpan() const;
      
      /**
       * Gets the receiver sensitivity in dBm
       * 
       * @return receiver sensitivity id dBm
       */
      double getReceiverSensitivitydBm() const;
      
      enum {IDENTIFIER = EMANE_CONTROL_MEASSGE_RECEIVE_PROPERTIES};
      
    private:
      class Implementation;
      std::unique_ptr<Implementation> pImpl_;
      
      ReceivePropertiesControlMessage(const TimePoint & sot,
                                      const Microseconds & propagation,
                                      const Microseconds & span,
                                      double dReceiverSensitivitydBm);
      
      ReceivePropertiesControlMessage(const ReceivePropertiesControlMessage &) = delete;

      ReceivePropertiesControlMessage &
      operator=(const ReceivePropertiesControlMessage &) = delete;
    };
  }
}

#endif // EMANECONTROLSRECEIVEPROPERTIESMESSAGE_HEADER_
