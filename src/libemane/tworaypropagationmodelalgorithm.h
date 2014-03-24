/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANETWORAYPROPAGATIONMODELALGORITHM_HEADER_
#define EMANETWORAYPROPAGATIONMODELALGORITHM_HEADER_

#include "propagationmodelalgorithm.h"

namespace EMANE
{
  class TwoRayPropagationModelAlgorithm : public PropagationModelAlgorithm
  {
  public:
    TwoRayPropagationModelAlgorithm(NEMId){}
      
    std::pair<std::vector<double>, bool> operator()(NEMId,
                                                    const LocationInfo & locationPairInfo,
                                                    const FrequencySegments & segments) override
    {
      // at least one location is unknown
      if(!locationPairInfo)
        {
          return {{},false};
        }

      double dDistance{locationPairInfo.getDistanceMeters()};

      double dPathloss{};

      if(dDistance)
        {
          dPathloss =
            (40.0 * log10(dDistance)) -
            (20.0 *
             (log10(locationPairInfo.getLocalPOV().getPosition().getAltitudeMeters()) +
              log10(locationPairInfo.getRemotePOV().getPosition().getAltitudeMeters())));
        }
        
      return {std::vector<double>(segments.size(),dPathloss < 0 ? 0 : dPathloss),true};
    }
  };
}

#endif  // EMANETWORAYPROPAGATIONMODELALGORITHM_HEADER_
