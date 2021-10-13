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

#ifndef EMANECONTROLSANTENNASELFINTERFERENCE_HEADER_
#define EMANECONTROLSANTENNASELFINTERFERENCE_HEADER_

#include "emane/types.h"

#include <memory>
#include <vector>

namespace EMANE
{
  namespace Controls
  {
    /**
     * @class AntennaSelfInterference
     *
     * @brief AntennaSelfInterference instances are used in conjunction
     * with a MIMOTxWhileRxInterferenceControlMessage instance to report
     * per antenna per frequency group interference power levels.
     *
     * @note Instances are immutable
     */
    class AntennaSelfInterference
    {
    public:
      /**
       * Creates an AntennaSelfInterference instance.
       *
       * @param frequencyGroupIndex FrequencyGroupIndex
       * @param dPowerMilliWatt Power in mW
       */
      AntennaSelfInterference(FrequencyGroupIndex frequencyGroupIndex,
                              double dPowerMilliWatt);

      AntennaSelfInterference(FrequencyGroupIndex frequencyGroupIndex,
                              const std::vector<double> & dPowerMilliWatts);

      AntennaSelfInterference(FrequencyGroupIndex frequencyGroupIndex,
                              std::vector<double> && dPowerMilliWatts);

      /**
       * Creates an AntennaSelfInterference instance by copy.
       */
      AntennaSelfInterference(const AntennaSelfInterference & rhs);

      /**
       * Sets an AntennaSelfInterference by copy
       *
       * @param rhs Instance to copy
       */
      AntennaSelfInterference & operator=(const AntennaSelfInterference & rhs);

      /**
       * Creates an AntennaSelfInterference by moving
       *
       * @param rval Instance to move
       */
      AntennaSelfInterference(AntennaSelfInterference && rval);

      /**
       * Sets an AntennaSelfInterference by moving
       *
       * @param rval Instance to move
       */
      AntennaSelfInterference & operator=(AntennaSelfInterference && rval);

      /**
       * Destorys an AntennaSelfInterference instance.
       */
      ~AntennaSelfInterference();

      /**
       * Gets the frequency group index
       *
       * @return rx antenna index
       *
       */
      AntennaIndex getFrequencyGroupIndex() const;

      /*
       * Gets the power in mW
       *
       * @return vector of powers
       */
      const std::vector<double> & getPowerMilliWatts() const;

    private:
      FrequencyGroupIndex frequencyGroupIndex_;
      std::vector<double> powerMilliWatts_;
    };

    using AntennaSelfInterferences = std::vector<AntennaSelfInterference>;
  }
}

#include "emane/controls/antennaselfinterference.inl"

#endif //EMANECONTROLSANTENNASELFINTERFERENCE_HEADER_
