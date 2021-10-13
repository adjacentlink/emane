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

#ifndef EMANECONTROLSRXANTENNAADDCONTROLMESSAGE_HEDAER_
#define EMANECONTROLSRXANTENNAADDCONTROLMESSAGE_HEDAER_

#include "emane/types.h"
#include "emane/antenna.h"
#include "emane/controlmessage.h"
#include "emane/controls/controlmessageids.h"

#include <memory>

namespace EMANE
{
  namespace Controls
  {
    /**
     * @class RxAntennaAddControlMessage
     *
     * @brief Rx Antenna Add Control Message is sent to the emulator
     * physical layer to add an rx antenna.
     *
     * @note Instances are immutable
     */
    class RxAntennaAddControlMessage : public ControlMessage
    {
    public:
      /**
       *  Creates an RxAntennaAddControlMessage instance on the heap
       *
       * @param antenna Antenna to add.
       * @param frequencyOfInterestSet Frequecies that will be
       * monitored by the antenna's respective spectrum monitor
       * instance.
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
      RxAntennaAddControlMessage * create(const Antenna & antenna,
                                          const FrequencySet & frequencyOfInterestSet);

      static
      RxAntennaAddControlMessage * create(const Antenna & antenna,
                                          FrequencySet && frequencyOfInterestSet);

      /**
       * Clones the control message on the heap
       *
       * @return cloned message
       *
       * @note Caller assumes ownership of the clone
       */
      RxAntennaAddControlMessage * clone() const override;

      /**
       * Destroys an instance
       */
      ~RxAntennaAddControlMessage();

      /**
       * Gets antenna
       *
       * @return Antenna
       */
      const Antenna & getAntenna() const;

      const FrequencySet & getFrequencyOfInterestSet() const;

      enum {IDENTIFIER = EMANE_CONTROL_MEASSAGE_RX_ANTENNA_ADD};

    private:
      class Implementation;
      std::shared_ptr<Implementation> pImpl_;

      RxAntennaAddControlMessage(const Antenna & antenna,
                                 const FrequencySet & frequencyOfInterestSet);

      RxAntennaAddControlMessage(const Antenna & antenna,
                                 FrequencySet && frequencyOfInterestSet);


      RxAntennaAddControlMessage(const RxAntennaAddControlMessage &);

      RxAntennaAddControlMessage &
      operator=(const RxAntennaAddControlMessage &) = delete;
    };
  }
}

#endif // EMANECONTROLSRXANTENNAADDCONTROLMESSAGE_HEDAER_
