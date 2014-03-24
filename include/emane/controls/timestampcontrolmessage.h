/*
 * Copyright (c) 2014 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANECONTROLSTIMESTAMPMESSAGE_HEADER_
#define EMANECONTROLSTIMESTAMPMESSAGE_HEADER_

#include "emane/controlmessage.h"
#include "emane/controls/controlmessageids.h"

#include <memory>

namespace EMANE
{
  namespace Controls
  {
    /**
     * @class TimeStampControlMessage
     *
     * @brief Time Stamp Control Message is sent to the emulator physical
     * layer to specify the time that should be used for the tx time stamp in the 
     * EMANE::CommonPHYHeader.
     *
     * @note If this control message is not sent along with the downstream packet
     * the emulator physical layer will use the current time for the tx time stamp.
     *
     * @note The Start of Transmission (SoT) is the tx time stamp + propagation delay + offset
     * of the first frequency segment.
     *
     * @note Instances are immutable
     */
    class TimeStampControlMessage : public ControlMessage
    {
    public:
      /**
       * Creates a TimeStampControlMessage on the heap
       *
       * @param timestamp Time value to use as the tx time in the common PHY Header
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
      TimeStampControlMessage * create(const TimePoint & timestamp);

      /**
       * Clones the control message on the heap
       *
       * @return cloned message
       *
       * @note Caller assumes ownership of the clone
       */
      TimeStampControlMessage * clone() const override;

      /**
       * Destroys an instance
       */
      ~TimeStampControlMessage();
      
      /**
       * Gets the time stamp
       *
       * @return time stamp
       */
      TimePoint getTimeStamp() const;
      
      enum {IDENTIFIER = EMANE_CONTROL_MEASSGE_TIME_STAMP};
      
    private:
      class Implementation;
      std::unique_ptr<Implementation> pImpl_;
      
      TimeStampControlMessage(const TimePoint & timestamp);
      
      TimeStampControlMessage(const TimeStampControlMessage &);

      TimeStampControlMessage &
      operator=(const TimeStampControlMessage &) = delete;
    };
  }
}

#endif // EMANECONTROLSTIMESTAMPMESSAGE_HEADER_
