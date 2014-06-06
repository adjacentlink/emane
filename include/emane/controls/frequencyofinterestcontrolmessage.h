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

#ifndef EMANECONTROLSFREQUENCYOFINTERESTCONTROLMESSAGE_HEDAER_
#define EMANECONTROLSFREQUENCYOFINTERESTCONTROLMESSAGE_HEDAER_

#include "emane/controlmessage.h"
#include "emane/controls/controlmessageids.h"
#include "emane/spectrumserviceprovider.h"

#include <memory>
#include <cstdint>

namespace EMANE
{
  namespace Controls
  {
    /**
     * @class FrequencyOfInterestControlMessage
     *
     * @brief The Frequency of Interest Control Message is sent to the emulator physical
     * layer to specify receive frequencies. These frequencies are monitored by the
     * SpectrumMonitor.
     *
     * @note Instances are immutable
     */
    class FrequencyOfInterestControlMessage : public ControlMessage
    {
    public:
      /**
       * Creates a FrequencyOfInterestControlMessage on the heap
       *
       * @param u64BandwidthHz Bandwidth in Hz
       * @param frequencySet Frequency set
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
      FrequencyOfInterestControlMessage * create(std::uint64_t u64BandwidthHz,
                                                 const FrequencySet & frequencySet);

      /**
       * Clones the control message on the heap
       *
       * @return cloned message
       *
       * @note Caller assumes ownership of the clone
       */
      FrequencyOfInterestControlMessage * clone() const override;

      /**
       * Destroys an instance
       */
      ~FrequencyOfInterestControlMessage();

      /**
       * Gets the frequency segments
       *
       * @return segments
       *
       * @see FrequencySegment
       */
      const FrequencySet & getFrequencySet() const;
      
      /**
       * Gets the bandwidth in Hz
       *
       * @return bandwidth
       */
      std::uint64_t getBandwidthHz() const;

      enum {IDENTIFIER = EMANE_CONTROL_MEASSGE_FREQUENCY_OF_INTEREST};
      
    private:
      class Implementation;
      std::unique_ptr<Implementation> pImpl_;
      
      FrequencyOfInterestControlMessage(std::uint64_t u64BandwidthHz,
                                        const FrequencySet & frequencySet);
      
      FrequencyOfInterestControlMessage(const FrequencyOfInterestControlMessage &);

      FrequencyOfInterestControlMessage &
      operator=(const FrequencyOfInterestControlMessage &) = delete;
    };
  }
}

#endif // EMANECONTROLSFREQUENCYCONTROLMESSAGE_HEDAER_
