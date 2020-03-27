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

#ifndef EMANECONTROLSSPECTRUMFILTERDATACONTROLMESSAGE_HEADER_
#define EMANECONTROLSSPECTRUMFILTERDATACONTROLMESSAGE_HEADER_

#include "emane/types.h"
#include "emane/controlmessage.h"
#include "emane/controls/controlmessageids.h"

#include <memory>

namespace EMANE
{
  namespace Controls
  {
    /**
     * @class SpectrumFilterDataControlMessage
     *
     * @brief The Spectrum Filter Data control message is sent to the
     * emulator physical layer to add filter data to an OTA message.
     *
     * @note Instances are immutable
     */
    class SpectrumFilterDataControlMessage : public ControlMessage
    {
    public:
      /**
       * Creates a SpectrumFilterDataControlMessage on the heap
       *
       * @param filterData filter Data to be added.
       */
      static
      SpectrumFilterDataControlMessage * create(const FilterData & filterData);

      /**
       * Clones the control message on the heap
       *
       * @return cloned message
       *
       * @note Caller assumes ownership of the clone
       */
      SpectrumFilterDataControlMessage * clone() const override;

      /**
       * Destroys an instance
       */
      ~SpectrumFilterDataControlMessage();

      /**
       * Gets the filter data
       *
       * @return filter data
       */
      const FilterData & getFilterData() const;

      enum {IDENTIFIER = EMANE_CONTROL_MEASSGE_SPECTRUM_FILTER_DATA};

    private:
      class Implementation;
      std::unique_ptr<Implementation> pImpl_;

      SpectrumFilterDataControlMessage(const FilterData & filterData);

      SpectrumFilterDataControlMessage(const SpectrumFilterDataControlMessage &);

      SpectrumFilterDataControlMessage &
      operator=(const SpectrumFilterDataControlMessage &) = delete;
    };
  }
}

#endif // EMANECONTROLSSPECTRUMFILTERDATACONTROLMESSAGE_HEADER_
