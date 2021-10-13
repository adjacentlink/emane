/*
 * Copyright (c) 2013,2020 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANEANTENNAMANAGER_HEADER_
#define EMANEANTENNAMANAGER_HEADER_

#include "antennaprofilemanifest.h"
#include "emane/antenna.h"
#include "emane/events/antennaprofile.h"

namespace EMANE
{
  class AntennaManager
  {
  public:
    AntennaManager();

    struct AntennaInfo
    {
      std::uint64_t u64UpdateSequence_{};
      Antenna antenna_{};
      AntennaPattern * pPattern_{};
      AntennaPattern * pBlockage_{};
      PositionNEU placement_{};

      AntennaInfo();
    };

    void update(const Events::AntennaProfiles & antennaProfiles);

    void update(NEMId nemId, const Antenna & antenna);

    std::pair<const AntennaInfo &, bool> getAntennaInfo(NEMId nemId,
                                                        AntennaIndex antennaIndex) const;
    void remove(NEMId nemId,
                AntennaIndex antennaIndex);

  private:
    using AntennaStore = std::map<AntennaIndex,AntennaInfo>;
    using NEMAntennaStore = std::map<NEMId, AntennaStore>;
    using DefaultEventPointingStore = std::map<NEMId, Antenna::Pointing>;
    NEMAntennaStore store_;
    DefaultEventPointingStore defaultEventPointingStore_;
    std::uint64_t u64UpdateSequence_;
  };
}

#endif // EMANEANTENNAMANAGER_HEADER_
