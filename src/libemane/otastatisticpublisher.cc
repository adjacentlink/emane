/*
 * Copyright (c) 2016-2017 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "otastatisticpublisher.h"
#include "statisticregistrarproxy.h"

// specialized hash for PacketCountTable
namespace std
{
  template<>
  struct hash<std::pair<std::string,EMANE::NEMId>>
  {
    typedef  std::pair<std::string,EMANE::NEMId> argument_type;
    typedef std::size_t result_type;

    result_type operator()(argument_type const& s) const
    {
      result_type const h1{std::hash<std::string>()(s.first)};
      result_type const h2{std::hash<EMANE::NEMId>()(s.second)};
      return h1 ^ (h2 << 1);
    }
  };
}

namespace
{
  const EMANE::StatisticTableLabels PacketCountLabels =
    {
      "Src",
      "Emulator UUID",
      "Num Pkts Tx",
      "Num Pkts Rx",
      "Pkts Rx Drop Miss Part",
    };

  enum PacketCountColumn
    {
      PACKET_COUNT_COLUMN_NEM = 0,
      PACKET_COUNT_COLUMN_UUID = 1,
      PACKET_COUNT_COLUMN_NUM_PACKETS_TX = 2,
      PACKET_COUNT_COLUMN_NUM_PACKETS_RX = 3,
      PACKET_COUNT_COLUMN_NUM_PACKETS_RX_DROP_MISS_PART = 4,
    };
}

EMANE::OTAStatisticPublisher::OTAStatisticPublisher():
  rowLimit_{0}
{
  auto statisticRegistrar = StatisticRegistrarProxy{*StatisticServiceSingleton::instance(),0};

  pNumOTAChannelDownstreamPackets_ =
    statisticRegistrar.registerNumeric<std::uint64_t>("numOTAChannelDownstreamPackets",
                                                      StatisticProperties::CLEARABLE);
  pNumOTAChannelUpstreamPackets_ =
    statisticRegistrar.registerNumeric<std::uint64_t>("numOTAChannelUpstreamPackets",
                                                      StatisticProperties::CLEARABLE);
  pNumOTAChannelUpstreamPacketsDroppedMissingPart_ =
    statisticRegistrar.registerNumeric<std::uint64_t>("numOTAChannelUpstreamPacketsDroppedMissingPart",
                                                      StatisticProperties::CLEARABLE);

  pPacketCountTable_ =
    statisticRegistrar.registerTable<PacketCountTableKey>("OTAChannelPacketCountTable",
                                                          PacketCountLabels,
                                                          [this](StatisticTablePublisher * pTable)
                                                          {
                                                            std::lock_guard<std::mutex> m(mutexPacketCountTable_);
                                                            packetCountInfo_.clear();
                                                            pTable->clear();
                                                          },
                                                          "OTA packet count table.");
}

void EMANE::OTAStatisticPublisher::update(Type type, const uuid_t & uuid, NEMId nemId)
{
  char buf[37];
  uuid_unparse(uuid,buf);
  auto key = PacketCountTableKey{buf,nemId};

  std::lock_guard<std::mutex> m(mutexPacketCountTable_);

  auto iter = packetCountInfo_.find(key);

  if(iter == packetCountInfo_.end())
    {
      if(packetCountInfo_.size() < rowLimit_)
        {
          iter = packetCountInfo_.insert({key,std::make_tuple(0,0,0)}).first;

          pPacketCountTable_->addRow(key,
                                     {Any{nemId},
                                         Any{buf},
                                           Any{0L},
                                             Any{0L},
                                               Any{0L}});
        }
    }

  if(type == Type::TYPE_UPSTREAM_PACKET_SUCCESS)
    {
      ++*pNumOTAChannelUpstreamPackets_;

      if(iter != packetCountInfo_.end())
        {
          auto & packets = std::get<PACKET_COUNT_COLUMN_NUM_PACKETS_RX-2>(iter->second);

          packets += 1;

          pPacketCountTable_->setCell(key,
                                      PACKET_COUNT_COLUMN_NUM_PACKETS_RX,
                                      Any{packets});
        }

    }
  else if(type == Type::TYPE_DOWNSTREAM_PACKET_SUCCESS)
    {
      ++*pNumOTAChannelDownstreamPackets_;

      if(iter !=  packetCountInfo_.end())
        {
          auto & packets = std::get<PACKET_COUNT_COLUMN_NUM_PACKETS_TX-2>(iter->second);

          packets += 1;

          pPacketCountTable_->setCell(key,
                                      PACKET_COUNT_COLUMN_NUM_PACKETS_TX,
                                      Any{packets});
        }
    }
  else if(type == Type::TYPE_UPSTREAM_PACKET_DROP_MISSING_PARTS)
    {
      ++*pNumOTAChannelUpstreamPacketsDroppedMissingPart_;

      if(iter !=  packetCountInfo_.end())
        {
          auto & packets = std::get<PACKET_COUNT_COLUMN_NUM_PACKETS_RX_DROP_MISS_PART-2>(iter->second);

          packets += 1;

          pPacketCountTable_->setCell(key,
                                      PACKET_COUNT_COLUMN_NUM_PACKETS_RX_DROP_MISS_PART,
                                      Any{packets});
        }
    }
}

void  EMANE::OTAStatisticPublisher::setRowLimit(size_t rows)
{
  rowLimit_ = rows;
}
