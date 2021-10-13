/*
 * Copyright (c) 2019-2020 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANECONTROLSSPECTRUMFILTERADDCONTROLMESSAGE_HEADER_
#define EMANECONTROLSSPECTRUMFILTERADDCONTROLMESSAGE_HEADER_

#include "emane/types.h"
#include "emane/controlmessage.h"
#include "emane/controls/controlmessageids.h"
#include "emane/filtermatchcriterion.h"

#include <memory>

namespace EMANE
{
  namespace Controls
  {
    /**
     * @class SpectrumFilterAddControlMessage
     *
     * @brief The Spectrum Filter Add control message is sent to the
     * emulator physical layer to add a physical layer filter.
     * Physical layer filters record spectrum energy based on a set of
     * specified criteria. Filter spectrum windows can be used to
     * perform actions such as excision and cancellation.
     *
     * @note Instances are immutable
     */
    class SpectrumFilterAddControlMessage : public ControlMessage
    {
    public:
      /**
       * Creates a SpectrumFilterAddControlMessage on the heap
       *
       * @param u16FilterIndex Unique filter index
       * @param antennaIndex Antenna index to attach filter
       * @param u64FrequencyHz Filter center frequency in Hz
       * @param u64BandwidthHz Filter bandwidth Hz
       * @param u64SubBandBinSizeHz Size of a bandwidth bin in Hz
       * @param pFilterMatchCriterion Heap allocated filter match
       * crierion object.
       *
       * @note FilterMatchCriterion object ownership is transferred to
       * the emulator infrastructure along with deallocation
       * responsibility.  It is not valid to use a FilterMatchCriterion
       * instance after it has been added to a control message.
       */
      static
      SpectrumFilterAddControlMessage * create(FilterIndex u16FilterIndex,
                                               AntennaIndex antennaIndex,
                                               std::uint64_t u64FrequencyHz,
                                               std::uint64_t u64BandwidthHz,
                                               std::uint64_t u64SubBandBinSizeHz = 0,
                                               FilterMatchCriterion * pFilterMatchCriterion = nullptr);

      /**
       * Creates a SpectrumFilterAddControlMessage on the heap
       *
       * @param u16FilterIndex Unique filter index
       * @param u64FrequencyHz Filter center frequency in Hz
       * @param u64BandwidthHz Filter bandwidth Hz
       * @param u64SubBandBinSizeHz Size of a bandwidth bin in Hz
       * @param pFilterMatchCriterion Heap allocated filter match
       * crierion object.
       *
       * @note FilterMatchCriterion object ownership is transferred to
       * the emulator infrastructure along with deallocation
       * responsibility.  It is not valid to use a FilterMatchCriterion
       * instance after it has been added to a control message.
       */
      static
      SpectrumFilterAddControlMessage * create(FilterIndex u16FilterIndex,
                                               std::uint64_t u64FrequencyHz,
                                               std::uint64_t u64BandwidthHz,
                                               std::uint64_t u64SubBandBinSizeHz = 0,
                                               FilterMatchCriterion * pFilterMatchCriterion = nullptr);

      /**
       * Clones the control message on the heap
       *
       * @return cloned message
       *
       * @note Caller assumes ownership of the clone
       */
      SpectrumFilterAddControlMessage * clone() const override;


      /**
       * Destroys an instance
       */
      ~SpectrumFilterAddControlMessage();

      /**
       * Gets the filter id
       *
       * @return filter id
       */
      FilterIndex getFilterIndex() const;

      /**
       * Gets the antenna index
       *
       * @return antenna index
       */
      AntennaIndex getAntennaIndex() const;

      /**
       * Gets the bandwidth in Hz
       *
       * @return bandwidth
       */
      std::uint64_t getBandwidthHz() const;

      /**
       * Gets the center frequency in Hz
       *
       * @return frequency
       */
      std::uint64_t getFrequencyHz() const;

      /**
       * Gets a borrowed reference to the filter match criterion
       * object.
       *
       * @return FilterMatchCriterion
       *
       *  @note Clone the FilterMatchCriterion object to store. Cloner
       *  assumes ownership of the clone.
       */
      const FilterMatchCriterion * getFilterMatchCriterion() const;


      /**
       * Gets the bandwidth bin size in Hz
       *
       * @return bandwidth bin size
       */
      std::uint64_t getSubBandBinSizeHz() const;

      enum {IDENTIFIER = EMANE_CONTROL_MEASSGE_SPECTRUM_FILTER_ADD};

    private:
      class Implementation;
      std::unique_ptr<Implementation> pImpl_;

      SpectrumFilterAddControlMessage(FilterIndex u16FilterIndex,
                                      AntennaIndex antennaIndex,
                                      std::uint64_t u64FrequencyHz,
                                      std::uint64_t u64BandwidthHz,
                                      std::uint64_t u64SubBandBinSizeHz,
                                      FilterMatchCriterion * pFilterMatchCriterion);

      SpectrumFilterAddControlMessage(const SpectrumFilterAddControlMessage &);

      SpectrumFilterAddControlMessage &
      operator=(const SpectrumFilterAddControlMessage &) = delete;
    };
  }
}

#endif // EMANECONTROLSSPECTRUMFILTERADDCONTROLMESSAGE_HEADER_
