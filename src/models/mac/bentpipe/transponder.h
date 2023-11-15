/*
 * Copyright (c) 2023 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANE_MODELS_BENTPIPE_TRANSPONDER_HEADER_
#define EMANE_MODELS_BENTPIPE_TRANSPONDER_HEADER_

#include "transponderconfiguration.h"
#include "bentpipemessage.h"

#include "emane/platformserviceprovider.h"

#include <tuple>

namespace EMANE
{
  namespace Models
  {
    namespace BentPipe
    {
      class TransponderUser;

      class Transponder
      {
      public:
        TransponderIndex getIndex() const;

        TransponderConfiguration & getConfiguration();

        const TransponderConfiguration & getConfiguration() const;

        virtual void start() = 0;

        virtual void stop() = 0;

        virtual bool isTransmitOpportunity(const TimePoint & now) = 0;

        virtual size_t getMTUBytes() const = 0;

        using TransmissionInfo = std::tuple<BentPipeMessage,
                                            Microseconds>;

        virtual TransmissionInfo prepareTransmission(const TimePoint & now,
                                                     size_t bytes,
                                                     MessageComponents && messages) = 0;
      protected:

        Transponder(NEMId id,
                    PlatformServiceProvider * pPlatformService,
                    TransponderUser * pTransponderUser,
                    const TransponderConfiguration & transponderConfiguration);

        NEMId id_;
        PlatformServiceProvider * pPlatformService_;
        TransponderUser * pTransponderUser_;
        TransponderConfiguration configuration_;
      };
    }
  }
}

#endif // EMANE_MODELS_BENTPIPE_TRANSPONDER_HEADER_
