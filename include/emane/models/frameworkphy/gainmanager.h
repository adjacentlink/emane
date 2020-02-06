/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANEPHYGAINMANAGER_HEADER
#define EMANEPHYGAINMANAGER_HEADER_

#include "emane/models/frameworkphy/antennapattern.h"
#include "emane/models/frameworkphy/positionneu.h"
#include "emane/models/frameworkphy/locationinfo.h"
#include "emane/types.h"
#include "emane/events/antennaprofile.h"

namespace EMANE
{
  class GainManager
  {
  public:
    GainManager(NEMId nemId);
    
    void update(const Events::AntennaProfiles & antennaProfiles);

    enum class GainStatus {
      SUCCESS = 0,
        ERROR_LOCATIONINFO,
        ERROR_PROFILEINFO,
        ERROR_HORIZON,
        };

    std::pair<double,GainStatus> determineGain(NEMId transmitterId,
                                               const LocationInfo & locationPairInfo,
                                               const std::pair<double, bool> & optionalRxFixedGaindBi,
                                               const std::pair<double, bool> & optionalTxFixedGaindBi) const;

  private:
    using AntennaProfileStore = std::map<NEMId,Events::AntennaProfile>;
    NEMId nemId_;
    AntennaProfileStore antennaProfileStore_;
    AntennaPattern * pLocalPattern_;
    AntennaPattern * pLocalBlockage_;
    PositionNEU localAntennaPlacement_;
    double dLocalAntennaAzimuthDegrees_;
    double dLocalAntennaElevationDegrees_;
    bool bHasLocalAntennaProfile_;
  };
}

#endif // EMANEPHYGAINMANAGER_HEADER_
