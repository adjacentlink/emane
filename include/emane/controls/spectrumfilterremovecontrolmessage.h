/*
 * Copyright (c) 2019 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANECONTROLSSPECTRUMFILTERREMOVECONTROLMESSAGE_HEADER_
#define EMANECONTROLSSPECTRUMFILTERREMOVECONTROLMESSAGE_HEADER_

#include "emane/types.h"
#include "emane/controlmessage.h"
#include "emane/controls/controlmessageids.h"

#include <memory>

namespace EMANE
{
  namespace Controls
  {
    /**
     * @class SpectrumFilterRemoveControlMessage
     *
     * @brief The Spectrum Filter Remove control message is sent to the
     * emulator physical layer to remove a spectrum filter.
     *
     * @note Instances are immutable
     */
    class SpectrumFilterRemoveControlMessage : public ControlMessage
    {
    public:
      /**
       * Creates a SpectrumFilterRemoveControlMessage on the heap
       *
       * @param filterIndex Index of filter to be removed.
       * @param antennaIndex Index of antenna containg the filter.
       */
      static
      SpectrumFilterRemoveControlMessage * create(FilterIndex filterIndex,
                                                  AntennaIndex antennaIndex);

      /**
       * Creates a SpectrumFilterRemoveControlMessage on the heap
       *
       * @param filterIndex Index of filter to be removed.
       */
      static
      SpectrumFilterRemoveControlMessage * create(FilterIndex filterIndex);

      /**
       * Clones the control message on the heap
       *
       * @return cloned message
       *
       * @note Caller assumes ownership of the clone
       */
      SpectrumFilterRemoveControlMessage * clone() const override;

      /**
       * Destroys an instance
       */
      ~SpectrumFilterRemoveControlMessage();

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

      enum {IDENTIFIER = EMANE_CONTROL_MEASSGE_SPECTRUM_FILTER_REMOVE};

    private:
      class Implementation;
      std::unique_ptr<Implementation> pImpl_;

      SpectrumFilterRemoveControlMessage(FilterIndex filterIndex,
                                         AntennaIndex antennaIndex);

      SpectrumFilterRemoveControlMessage(const SpectrumFilterRemoveControlMessage &);

      SpectrumFilterRemoveControlMessage &
      operator=(const SpectrumFilterRemoveControlMessage &) = delete;
    };
  }
}

#endif // EMANECONTROLSSPECTRUMFILTERREMOVECONTROLMESSAGE_HEADER_
