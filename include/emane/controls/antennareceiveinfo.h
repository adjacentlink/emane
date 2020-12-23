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

#ifndef EMANECONTROLSANTENNARECEIVEINFO_HEADER_
#define EMANECONTROLSANTENNARECEIVEINFO_HEADER_

#include "emane/types.h"
#include "emane/frequencysegment.h"

#include <memory>
#include <vector>

namespace EMANE
{
  namespace Controls
  {
    /**
     * @class AntennaReceiveInfo
     *
     * @brief AntennaReceiveInfo instances are used in conjunction
     * with a MIMOReceivePropertiesControlMessage instance to report
     * per antenna receive information such as frequency segments with
     * power, span, and receiver sensitivity.
     *
     * @note Instances are immutable
     */
    class AntennaReceiveInfo
    {
    public:
      /**
       * Creates an AntennaReceiveInfo instance.
       *
       * @param rxAntennaIndex Receive antenna index
       * @param txAntennaIndex Transmit antenna index
       * @param frequencySegments Frequency segments
       * @param span Length of time in microseconds between the earliest
       * start-of-reception (SoR) frequency segment and latest end-of-reception (EoR) segment.
       * @param dReceiverSensitivitydBm Receiver sensitivity in dBm
       */
      AntennaReceiveInfo(AntennaIndex rxAntennaIndex,
                         AntennaIndex txAntennaIndex,
                         const FrequencySegments & frequencySegments,
                         Microseconds span,
                         double dReceiverSensitivitydBm);

      AntennaReceiveInfo(AntennaIndex rxAntennaIndex,
                         AntennaIndex txAntennaIndex,
                         FrequencySegments && frequencySegments,
                         Microseconds span,
                         double dReceiverSensitivitydBm);

      /**
       * Creates an AntennaReceiveInfo instance by copy.
       */
      AntennaReceiveInfo(const AntennaReceiveInfo & rhs);

      /**
       * Sets an AntennaReceiveInfo by copy
       *
       * @param rhs Instance to copy
       */
      AntennaReceiveInfo & operator=(const AntennaReceiveInfo & rhs);

      /**
       * Creates an AntennaReceiveInfo by moving
       *
       * @param rval Instance to move
       */
      AntennaReceiveInfo(AntennaReceiveInfo && rval);

      /**
       * Sets an AntennaReceiveInfo by moving
       *
       * @param rval Instance to move
       */
      AntennaReceiveInfo & operator=(AntennaReceiveInfo && rval);

      /**
       * Destorys an AntennaReceiveInfo instance.
       */
      ~AntennaReceiveInfo();

      /**
       * Gets the receive antenna index
       *
       * @return rx antenna index
       *
       */
      AntennaIndex getRxAntennaIndex() const;

      /**
       * Gets the transmit antenna index
       *
       * @return tx antenna index
       *
       */
      AntennaIndex getTxAntennaIndex() const;

      /**
       * Gets the frequency segments
       *
       * @return segments
       *
       * @see FrequencySegment
       */
      const FrequencySegments & getFrequencySegments() const;

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

    private:
      AntennaIndex rxAntennaIndex_;
      AntennaIndex txAntennaIndex_;
      FrequencySegments frequencySegments_;
      Microseconds span_;
      double dReceiverSensitivitydBm_;
    };

    using AntennaReceiveInfos = std::vector<AntennaReceiveInfo>;
  }
}

#include "emane/controls/antennareceiveinfo.inl"

#endif //EMANECONTROLSANTENNARECEIVEINFO_HEADER_
