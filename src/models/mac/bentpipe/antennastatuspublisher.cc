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

#include "antennastatuspublisher.h"

// specialized hash for AntennaStatusTable
namespace std
{
  template<>
  struct hash<std::pair<EMANE::AntennaIndex,std::uint64_t>>
  {
    typedef std::pair<EMANE::AntennaIndex,std::uint64_t> argument_type;
    typedef std::size_t result_type;

    result_type operator()(argument_type const& s) const
    {
      result_type const h1{std::hash<std::uint64_t>()(s.first)};
      result_type const h2{std::hash<std::uint64_t>()(s.second)};
      return (h1 << 6) ^ h2;
    }
  };
}

namespace
{
  const EMANE::StatisticTableLabels AntennaStatusLabels =
    {
      "Index",
      "Profile",
      "Bandwidth",
      "Rx Frequency",
      "Fixed Gain",
      "Azimuth",
      "Elevation",
      "Mask",
    };
}

EMANE::Models::BentPipe::AntennaStatusPublisher::AntennaStatusPublisher(){}

EMANE::Models::BentPipe::AntennaStatusPublisher::~AntennaStatusPublisher(){}

void EMANE::Models::BentPipe::AntennaStatusPublisher::registerStatistics(StatisticRegistrar & statisticRegistrar)
{
  pAntennaStatusTable_ =
    statisticRegistrar.registerTable<std::pair<std::uint16_t,std::uint64_t>>("AntennaStatusTable",
                                                                             AntennaStatusLabels,
                                                                             StatisticProperties::NONE,
                                                                             "Antenna status table");
}

void EMANE::Models::BentPipe::AntennaStatusPublisher::addAntenna(const Antenna & antenna,
                                                                 const FrequencySet & frequencySet)
{
  if(!knownAntenna_.count(antenna.getIndex()))
    {
      auto pointing = antenna.getPointing();

      for(const auto & frequencyHz : frequencySet)
        {
          auto key = std::make_pair(antenna.getIndex(),frequencyHz);

          pAntennaStatusTable_->addRow(key,
                                       {Any{antenna.getIndex()},
                                        antenna.isProfileDefined() ?  Any{pointing.first.getProfileId()} : Any{"NA"},
                                        Any{antenna.getBandwidthHz()},
                                        Any{frequencyHz},
                                        antenna.isProfileDefined() ? Any{"NA"} : Any{antenna.getFixedGaindBi().first},
                                        antenna.isProfileDefined() ?  Any{pointing.first.getAzimuthDegrees()} : Any{"NA"},
                                        antenna.isProfileDefined() ?  Any{pointing.first.getElevationDegrees()} : Any{"NA"},
                                        Any{antenna.getSpectralMaskIndex()}
                                       });
        }

      knownAntenna_.emplace(antenna.getIndex(),frequencySet);
    }
}

void EMANE::Models::BentPipe::AntennaStatusPublisher::removeAntenna(const Antenna & antenna)
{
  if(auto iter = knownAntenna_.find(antenna.getIndex());
     iter !=  knownAntenna_.end())
    {
      for(const auto & frequencyHz : iter->second)
        {
          pAntennaStatusTable_->deleteRow(std::make_pair(iter->first,frequencyHz));
        }

      knownAntenna_.erase(iter);
    }
}
