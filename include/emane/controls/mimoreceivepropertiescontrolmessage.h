/*
 * Copyright (c) 2020-2021 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANECONTROLSMIMORECEIVEPROPERTIESMESSAGE_HEADER_
#define EMANECONTROLSMIMORECEIVEPROPERTIESMESSAGE_HEADER_

#include "emane/controlmessage.h"
#include "emane/types.h"
#include "emane/controls/controlmessageids.h"
#include "emane/controls/antennareceiveinfo.h"
#include "emane/controls/dopplershifts.h"

#include <memory>
#include <map>

namespace EMANE
{
  namespace Controls
  {
    /**
     * @class MIMOReceivePropertiesControlMessage
     *
     * @brief MIMO Receive Properties Control Message is sent from the
     * emulator physical layer with every upstream packet when
     * compatibility mode > 1. It contains the tx time stamp,
     * propagation delay, and per antenna receive information.
     *
     * @note The Start of Transmission (SoT) is the tx time  + propagation delay + offset
     * of the first frequency segment.
     *
     * @note Instances are immutable
     */
    class MIMOReceivePropertiesControlMessage : public ControlMessage
    {
    public:
      /**
       * Creates a MIMOReceivePropertiesControlMessage on the heap
       *
       * @param txTime Time value used as the tx time in the common PHY Header
       * @param propagation Propagation delay in microseconds
       * @param antennaReceiveInfos Receive antenna infomration.
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
      MIMOReceivePropertiesControlMessage * create(const TimePoint & txTime,
                                                   const Microseconds & propagation,
                                                   const AntennaReceiveInfos & antennaReceiveInfos,
                                                   const DopplerShifts & dopplerShifts);


      static
      MIMOReceivePropertiesControlMessage * create(const TimePoint & txTime,
                                                   const Microseconds & propagation,
                                                   AntennaReceiveInfos && antennaReceiveInfos,
                                                   DopplerShifts && dopplerShifts);
      /**
       * Clones the control message on the heap
       *
       * @return cloned message
       *
       * @note Caller assumes ownership of the clone
       */
      MIMOReceivePropertiesControlMessage * clone() const override;

      /**
       * Destroys an instance
       */
      ~MIMOReceivePropertiesControlMessage();

      /**
       * Gets the Tx time
       *
       * @return time
       */
      const TimePoint & getTxTime() const;


      /**
       * Gets the propagation delay in microseconds
       *
       * @return propagation delay in microseconds
       */
      const Microseconds & getPropagationDelay() const;


      /**
       * Gets the antenna receive information.
       *
       * @return antenna receive information.
       */
      const AntennaReceiveInfos & getAntennaReceiveInfos() const;


      /**
       * Gets the  Doppler shift frequency map
       *
       * @return Doppler shift frequency map
       */
      const DopplerShifts & getDopplerShifts() const;

      enum {IDENTIFIER = EMANE_CONTROL_MEASSGE_MIMO_RECEIVE_PROPERTIES};

    private:
      class Implementation;
      std::shared_ptr<Implementation> pImpl_;

      MIMOReceivePropertiesControlMessage(const TimePoint & sot,
                                          const Microseconds & propagation,
                                          const AntennaReceiveInfos & antennaReceiveInfos,
                                          const DopplerShifts & dopplerShifts);

      MIMOReceivePropertiesControlMessage(const TimePoint & sot,
                                          const Microseconds & propagation,
                                          AntennaReceiveInfos && antennaReceiveInfos,
                                          DopplerShifts && dopplerShifts );

      MIMOReceivePropertiesControlMessage(const MIMOReceivePropertiesControlMessage &);

      MIMOReceivePropertiesControlMessage &
      operator=(const MIMOReceivePropertiesControlMessage &) = delete;
    };
  }
}

#endif // EMANECONTROLSMIMORECEIVEPROPERTIESMESSAGE_HEADER_
