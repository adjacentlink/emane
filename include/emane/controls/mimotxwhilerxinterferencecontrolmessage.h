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

#ifndef EMANECONTROLSMIMOTXWHILERXINTERFERENCECONTROLMESSAGE_HEADER_
#define EMANECONTROLSMIMOTXWHILERXINTERFERENCECONTROLMESSAGE_HEADER_

#include "emane/controlmessage.h"
#include "emane/types.h"
#include "emane/controls/controlmessageids.h"
#include "emane/controls/antennaselfinterference.h"
#include "emane/frequencysegment.h"

#include <memory>
#include <vector>
#include <map>

namespace EMANE
{
  namespace Controls
  {
    /**
     * @class MIMOTxWhileRxInterferenceControlMessage
     *
     * @brief Tx While Rx Interference Control Message is sent to the
     * emulator physical layer to specify a receive power to apply to
     * the EMANE::SpectrumMonitor for those radio models that can
     * simultaenously transmit and receive and experience some amount
     * of self interference.
     *
     * @note Instances are immutable
     */
    class MIMOTxWhileRxInterferenceControlMessage : public ControlMessage
    {
    public:
      /**
       * Creates a MIMOTxWhileRxInterferenceControlMessage on the heap
       *
       * @param frequencyGroups Frequency groups to use
       * @param rxAntennaSelections RxAntennaInterferenceMap
       * containing antenna interference info.
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

      using RxAntennaInterferenceMap = std::map<AntennaIndex,AntennaSelfInterferences>;

      static
      MIMOTxWhileRxInterferenceControlMessage *
      create(const FrequencyGroups & frequencyGroups,
             const RxAntennaInterferenceMap & rxAntennaSelections);

      static
      MIMOTxWhileRxInterferenceControlMessage *
      create(FrequencyGroups && frequencyGroups,
             const RxAntennaInterferenceMap & rxAntennaSelections);

      static
      MIMOTxWhileRxInterferenceControlMessage *
      create(FrequencyGroups && frequencyGroups,
             RxAntennaInterferenceMap && rxAntennaSelections);

      /**
       * Clones the control message on the heap
       *
       * @return cloned message
       *
       * @note Caller assumes ownership of the clone
       */
      MIMOTxWhileRxInterferenceControlMessage * clone() const override;

      /**
       * Destroys an instance
       */
      ~MIMOTxWhileRxInterferenceControlMessage();

      /**
       * Gets the frequency groups
       *
       * @return time
       */
      const FrequencyGroups & getFrequencyGroups() const;


      /**
       * Gets the rx antenna selections
       *
       * @return propagation delay in microseconds
       */
      const RxAntennaInterferenceMap & getRxAntennaInterferenceMap() const;

      enum {IDENTIFIER = EMANE_CONTROL_MEASSGE_MIMO_TX_WHILE_RX_INTERFERENCE};

    private:
      class Implementation;
      std::shared_ptr<Implementation> pImpl_;

      MIMOTxWhileRxInterferenceControlMessage(const FrequencyGroups & frequencyGroups,
                                              const RxAntennaInterferenceMap & rxAntennaSelections);

      MIMOTxWhileRxInterferenceControlMessage(FrequencyGroups && frequencyGroups,
                                              const RxAntennaInterferenceMap & rxAntennaSelections);

      MIMOTxWhileRxInterferenceControlMessage(FrequencyGroups && frequencyGroups,
                                              RxAntennaInterferenceMap && rxAntennaSelections);

      MIMOTxWhileRxInterferenceControlMessage(const MIMOTxWhileRxInterferenceControlMessage &);

      MIMOTxWhileRxInterferenceControlMessage &
      operator=(const MIMOTxWhileRxInterferenceControlMessage &) = delete;
    };
  }
}

#endif // EMANECONTROLSMIMOTXWHILERXINTERFERENCECONTROLMESSAGE_HEADER_
