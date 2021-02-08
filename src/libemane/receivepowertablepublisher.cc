/*
 * Copyright (c) 2014,2021 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "receivepowertablepublisher.h"

// specialized hash for ReceivePowerTable
namespace std
{
  template<>
  struct hash<std::tuple<EMANE::NEMId,EMANE::AntennaIndex,EMANE::AntennaIndex,std::uint64_t>>
  {
    typedef std::tuple<EMANE::NEMId,EMANE::AntennaIndex,EMANE::AntennaIndex,std::uint64_t> argument_type;
    typedef std::size_t result_type;

    result_type operator()(argument_type const& s) const
    {
      result_type const h1{std::hash<std::uint64_t>()(std::get<0>(s))};
      result_type const h2{std::hash<std::uint64_t>()(std::get<1>(s))};
      result_type const h3{std::hash<std::uint64_t>()(std::get<2>(s))};
      result_type const h4{std::hash<std::uint64_t>()(std::get<3>(s))};
      return (h1 << 6) ^ (h2 << 4) ^ (h3 << 2) ^ h4;
    }
  };
}

void EMANE::ReceivePowerTablePublisher::registerStatistics(StatisticRegistrar & statisticRegistrar)
{

  pReceivePowerTable_ =
    statisticRegistrar.registerTable<ReceivePowerTableKey>("ReceivePowerTable",
                                                           {"NEM",
                                                            "Rx Antenna",
                                                            "Tx Antenna",
                                                            "Frequency",
                                                            "Rx Power",
                                                            "Tx Gain",
                                                            "Rx Gain",
                                                            "Tx Power",
                                                            "Pathloss",
                                                            "Last Packet Time"},
                                                           StatisticProperties::NONE,
                                                           "Shows the calculated receive power for the last received segment.");
}

void EMANE::ReceivePowerTablePublisher::update(NEMId nemId,
                                               AntennaIndex rxAntennaIndex,
                                               AntennaIndex txAntennaIndex,
                                               std::uint64_t u64Frequency,
                                               double dReceivePowerdBm,
                                               double dTxGaindBi,
                                               double dRxGaindBi,
                                               double dTransmitPowerdBm,
                                               double dPathlossdB,
                                               const TimePoint & rxTime)
{
  auto key = ReceivePowerTableKey{nemId,rxAntennaIndex,txAntennaIndex,u64Frequency};

  if(receivePowerTableSet_.count(key))
    {
      pReceivePowerTable_->setRow(key,
                                  {
                                    Any{nemId},
                                    Any{rxAntennaIndex},
                                    Any{txAntennaIndex},
                                    Any{u64Frequency},
                                    Any{dReceivePowerdBm},
                                    Any{dTxGaindBi},
                                    Any{dRxGaindBi},
                                    Any{dTransmitPowerdBm},
                                    Any{dPathlossdB},
                                    Any{std::chrono::duration_cast<DoubleSeconds>(rxTime.time_since_epoch()).count()}});
    }
  else
    {
      pReceivePowerTable_->addRow(key,
                                  {
                                    Any{nemId},
                                    Any{rxAntennaIndex},
                                    Any{txAntennaIndex},
                                    Any{u64Frequency},
                                    Any{dReceivePowerdBm},
                                    Any{dTxGaindBi},
                                    Any{dRxGaindBi},
                                    Any{dTransmitPowerdBm},
                                    Any{dPathlossdB},
                                    Any{std::chrono::duration_cast<DoubleSeconds>(rxTime.time_since_epoch()).count()}});

      receivePowerTableSet_.insert(key);
    }
}
