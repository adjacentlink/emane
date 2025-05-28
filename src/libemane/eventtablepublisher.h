/*
 * Copyright (c) 2014,2017,2025 - Adjacent Link LLC, Bridgewater,
 * New Jersey
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

#ifndef EMANEEVENTTABLEPUBLISHER_HEADER_
#define EMANEEVENTTABLEPUBLISHER_HEADER_

#include "emane/events/antennaprofile.h"
#include "emane/events/fadingselection.h"
#include "emane/events/location.h"
#include "emane/events/pathloss.h"
#include "emane/events/pathlossex.h"
#include "emane/statistictable.h"
#include "emane/statisticregistrar.h"

#include <set>

namespace EMANE
{
  class EventTablePublisher
  {
  public:
    EventTablePublisher(NEMId nemId);

    void registerStatistics(StatisticRegistrar & registrar);

    void update(const Events::Locations & locations);

    void update(const Events::Pathlosses & pathlosses);

    void update(const Events::PathlossExs & pathlossExs);

    void update(const Events::AntennaProfiles & profiles);

    void update(const Events::FadingSelections & selections);

  private:
    NEMId nemId_;
    std::set<NEMId> locationNEMSet_;
    std::set<NEMId> pathlossNEMSet_;
    std::set<NEMId> antennaProfileNEMSet_;
    std::set<NEMId> fadingSelectionNEMSet_;

    using PathlossExKey = std::pair<NEMId,std::uint64_t>;
    std::set<PathlossExKey> pathlossExNEMSet_;

    StatisticTable<NEMId> * pLocationTable_;
    StatisticTable<NEMId> * pPathlossTable_;
    StatisticTable<PathlossExKey> * pPathlossExTable_;
    StatisticTable<NEMId> * pAntennaProfileTable_;
    StatisticTable<NEMId> * pFadingSelectionTable_;
  };
}

#endif //EMANEEVENTTABLEPUBLISHER_HEADER_
