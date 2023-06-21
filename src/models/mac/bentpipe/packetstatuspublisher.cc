/*
 * Copyright (c) 2015-2016,2023 - Adjacent Link LLC, Bridgewater,
 *  New Jersey
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

#include "packetstatuspublisher.h"

namespace
{
  const EMANE::StatisticTableLabels PacketAcceptLabels =
    {
      "NEM",
      "Num Bytes Tx",
      "Num Bytes Rx"
    };

  enum PacketAcceptColumn
    {
      ACCEPT_COLUMN_NEM = 0,
      ACCEPT_COLUMN_NUM_BYTES_TX = 1,
      ACCEPT_COLUMN_NUM_BYTES_RX = 2,
    };

  const EMANE::StatisticTableLabels PacketDropLabels =
    {
      "NEM",
      "SINR",
      "Reg Id",
      "Dst MAC",
      "Queue Overflow",
      "Bad Control",
      "Bad Spectrum Query",
      "Big",
      "Long",
      "Miss Fragment",
      "Bad Curve",
      "Lock",
      "Rx Off",
      "Tx off",
    };

  enum PacketDropColumn
    {
      DROP_COLUMN_NEM = 0,
      DROP_COLUMN_SINR = 1,
      DROP_COLUMN_REG_ID = 2,
      DROP_COLUMN_DST_MAC = 3,
      DROP_COLUMN_QUEUE_OVERFLOW = 4,
      DROP_COLUMN_BAD_CONTROL = 5,
      DROP_COLUMN_BAD_SPECTRUM_QUERY = 6,
      DROP_COLUMN_TOO_BIG = 7,
      DROP_COLUMN_TOO_LONG = 8,
      DROP_COLUMN_MISS_FRAGMENT = 9,
      DROP_COLUMN_BAD_CURVE = 10,
      DROP_COLUMN_LOCK = 11,
      DROP_COLUMN_RX_OFF = 12,
      DROP_COLUMN_TX_OFF = 13,
    };

}
EMANE::Models::BentPipe::PacketStatusPublisher::PacketStatusPublisher(){}

EMANE::Models::BentPipe::PacketStatusPublisher::~PacketStatusPublisher(){}

void EMANE::Models::BentPipe::PacketStatusPublisher::registerStatistics(StatisticRegistrar & statisticRegistrar)
{
  for(int queueIndex = 0; queueIndex < QUEUE_COUNT; ++queueIndex)
    {
      broadcastAcceptTables_[queueIndex] =
        statisticRegistrar.registerTable<NEMId>("BroadcastByteAcceptTable" + std::to_string(queueIndex),
                                                PacketAcceptLabels,
                                                [this,queueIndex](StatisticTablePublisher * pTable)
                                                {
                                                  std::lock_guard<std::mutex> m(mutexBroadcastPacketAcceptTable_);
                                                  broadcastAcceptInfos_[queueIndex].clear();
                                                  pTable->clear();
                                                },
                                                "Broadcast bytes accepted");

      unicastAcceptTables_[queueIndex] =
        statisticRegistrar.registerTable<NEMId>("UnicastByteAcceptTable" + std::to_string(queueIndex),
                                                PacketAcceptLabels,
                                                [this,queueIndex](StatisticTablePublisher * pTable)
                                                {
                                                  std::lock_guard<std::mutex> m(mutexUnicastPacketAcceptTable_);
                                                  unicastAcceptInfos_[queueIndex].clear();
                                                  pTable->clear();
                                                },
                                                "Unicast bytes accepted");

      broadcastDropTables_[queueIndex] =
        statisticRegistrar.registerTable<NEMId>("BroadcastByteDropTable" + std::to_string(queueIndex),
                                                PacketDropLabels,
                                                [this,queueIndex](StatisticTablePublisher * pTable)
                                                {
                                                  std::lock_guard<std::mutex> m(mutexBroadcastPacketDropTable_);
                                                  broadcastDropInfos_[queueIndex].clear();
                                                  pTable->clear();
                                                },
                                                "Broadcast bytes dropped");

      unicastDropTables_[queueIndex] =
        statisticRegistrar.registerTable<NEMId>("UnicastByteDropTable" + std::to_string(queueIndex),
                                                PacketDropLabels,
                                                [this,queueIndex](StatisticTablePublisher * pTable)
                                                {
                                                  std::lock_guard<std::mutex> m(mutexUnicastPacketDropTable_);
                                                  unicastDropInfos_[queueIndex].clear();
                                                  pTable->clear();
                                                },
                                                "Unicast bytes dropped");
    }
}

void EMANE::Models::BentPipe::PacketStatusPublisher::inbound(NEMId src,
                                                             const MessageComponent & component,
                                                             InboundAction action)
{
  inbound(src,
          component.getDestination(),
          component.getData().size(),
          action);
}

void  EMANE::Models::BentPipe::PacketStatusPublisher::inbound(NEMId src,
                                                              const MessageComponents & components,
                                                              InboundAction action)
{
  for(const auto & component : components)
    {
      inbound(src,
              component.getDestination(),
              component.getData().size(),
              action);
    }
}



void EMANE::Models::BentPipe::PacketStatusPublisher::inbound(NEMId src,
                                                             NEMId dst,
                                                             size_t size,
                                                             InboundAction action)
{
  TableArray * pAcceptTables{};
  TableArray * pDropTables{};

  AcceptInfoArrary * pAcceptInfos{};
  DropInfoArrary  * pDropInfos{};

  std::mutex * pMutexAcceptTable = {};
  std::mutex * pMutexDropTable = {};

  if(dst == NEM_BROADCAST_MAC_ADDRESS)
    {
      pAcceptTables = &broadcastAcceptTables_;
      pDropTables = &broadcastDropTables_;

      pAcceptInfos = &broadcastAcceptInfos_;
      pDropInfos = &broadcastDropInfos_;

      pMutexAcceptTable = &mutexBroadcastPacketAcceptTable_;
      pMutexDropTable = &mutexBroadcastPacketDropTable_;
    }
  else
    {
      pAcceptTables = &unicastAcceptTables_;
      pDropTables = &unicastDropTables_;

      pAcceptInfos = &unicastAcceptInfos_;
      pDropInfos = &unicastDropInfos_;

      pMutexAcceptTable = &mutexUnicastPacketAcceptTable_;
      pMutexDropTable = &mutexUnicastPacketDropTable_;
    }

  if(action == InboundAction::ACCEPT_GOOD)
    {
      std::lock_guard<std::mutex> m(*pMutexAcceptTable);

      std::uint8_t u8QueueIndex{0};

      auto iter = (*pAcceptInfos)[u8QueueIndex].find(src);

      if(iter == (*pAcceptInfos)[u8QueueIndex].end())
        {
          iter = (*pAcceptInfos)[u8QueueIndex].insert({src,{}}).first;


          (*pAcceptTables)[u8QueueIndex]->addRow(src,
                                                 {Any{src},
                                                  Any{0L},
                                                  Any{0L}});
        }

      auto & bytes = std::get<ACCEPT_COLUMN_NUM_BYTES_RX-1>(iter->second);

      bytes += size;

      (*pAcceptTables)[u8QueueIndex]->setCell(src,
                                              ACCEPT_COLUMN_NUM_BYTES_RX,
                                              Any{bytes});
    }
  else
    {
      std::lock_guard<std::mutex> m(*pMutexDropTable);

      std::uint8_t u8QueueIndex{0};

      auto iter = (*pDropInfos)[u8QueueIndex].find(src);

      if(iter == (*pDropInfos)[u8QueueIndex].end())
        {
          iter = (*pDropInfos)[u8QueueIndex].insert({src,{}}).first;

          (*pDropTables)[u8QueueIndex]->addRow(src,
                                               {Any{src},
                                                Any{0L},
                                                Any{0L},
                                                Any{0L},
                                                Any{0L},
                                                Any{0L},
                                                Any{0L},
                                                Any{0L},
                                                Any{0L},
                                                Any{0L},
                                                Any{0L},
                                                Any{0L},
                                                Any{0L},
                                                Any{0L}});
        }

      switch(action)
        {
        case InboundAction::DROP_BAD_CONTROL:
          {
            auto & bytes = std::get<DROP_COLUMN_BAD_CONTROL-1>(iter->second);

            bytes += size;

            (*pDropTables)[u8QueueIndex]->setCell(src,
                                                  DROP_COLUMN_BAD_CONTROL,
                                                  Any{bytes});
          }
          break;

        case InboundAction::DROP_MISS_FRAGMENT:
          {
            auto & bytes = std::get<DROP_COLUMN_MISS_FRAGMENT-1>(iter->second);

            bytes += size;

            (*pDropTables)[u8QueueIndex]->setCell(src,
                                                  DROP_COLUMN_MISS_FRAGMENT,
                                                  Any{bytes});
          }
          break;

        case InboundAction::DROP_BAD_CURVE:
          {
            auto & bytes = std::get<DROP_COLUMN_BAD_CURVE-1>(iter->second);

            bytes += size;

            (*pDropTables)[u8QueueIndex]->setCell(src,
                                                  DROP_COLUMN_BAD_CURVE,
                                                  Any{bytes});
          }
          break;

        case InboundAction::DROP_LOCK:
          {
            auto & bytes = std::get<DROP_COLUMN_LOCK-1>(iter->second);

            bytes += size;

            (*pDropTables)[u8QueueIndex]->setCell(src,
                                                  DROP_COLUMN_LOCK,
                                                  Any{bytes});
          }
          break;

        case InboundAction::DROP_RX_OFF:
          {
            auto & bytes = std::get<DROP_COLUMN_RX_OFF-1>(iter->second);

            bytes += size;

            (*pDropTables)[u8QueueIndex]->setCell(src,
                                                  DROP_COLUMN_RX_OFF,
                                                  Any{bytes});
          }
          break;

        case InboundAction::DROP_SPECTRUM_SERVICE:
          {
            auto & bytes = std::get<DROP_COLUMN_BAD_SPECTRUM_QUERY-1>(iter->second);

            bytes += size;

            (*pDropTables)[u8QueueIndex]->setCell(src,
                                                  DROP_COLUMN_BAD_SPECTRUM_QUERY,
                                                  Any{bytes});
          }
          break;

        case InboundAction::DROP_SINR:
          {
            auto & bytes = std::get<DROP_COLUMN_SINR-1>(iter->second);

            bytes += size;

            (*pDropTables)[u8QueueIndex]->setCell(src,
                                                  DROP_COLUMN_SINR,
                                                  Any{bytes});
          }
          break;

        case InboundAction::DROP_REGISTRATION_ID:
          {
            auto & bytes = std::get<DROP_COLUMN_REG_ID-1>(iter->second);

            bytes += size;

            (*pDropTables)[u8QueueIndex]->setCell(src,
                                                  DROP_COLUMN_REG_ID,
                                                  Any{bytes});
          }
          break;

        case InboundAction::DROP_DESTINATION_MAC:
          {
            auto & bytes = std::get<DROP_COLUMN_DST_MAC-1>(iter->second);

            bytes += size;

            (*pDropTables)[u8QueueIndex]->setCell(src,
                                                  DROP_COLUMN_DST_MAC,
                                                  Any{bytes});
          }
          break;
        case InboundAction::DROP_TOO_LONG:
          {
            auto & bytes = std::get<DROP_COLUMN_TOO_LONG-1>(iter->second);

            bytes += size;

            (*pDropTables)[u8QueueIndex]->setCell(src,
                                                  DROP_COLUMN_TOO_LONG,
                                                  Any{bytes});
          }
          break;

        default:
          break;
        }
    }
}


void EMANE::Models::BentPipe::PacketStatusPublisher::outbound(NEMId src,
                                                              NEMId dst,
                                                              size_t size,
                                                              OutboundAction action)
{
  TableArray * pAcceptTables{};
  TableArray * pDropTables{};

  AcceptInfoArrary * pAcceptInfos{};
  DropInfoArrary  * pDropInfos{};

  std::mutex * pMutexAcceptTable = {};
  std::mutex * pMutexDropTable = {};

  if(dst == NEM_BROADCAST_MAC_ADDRESS)
    {
      pAcceptTables = &broadcastAcceptTables_;
      pDropTables = &broadcastDropTables_;

      pAcceptInfos = &broadcastAcceptInfos_;
      pDropInfos = &broadcastDropInfos_;

      pMutexAcceptTable = &mutexBroadcastPacketAcceptTable_;
      pMutexDropTable = &mutexBroadcastPacketDropTable_;
    }
  else
    {
      pAcceptTables = &unicastAcceptTables_;
      pDropTables = &unicastDropTables_;

      pAcceptInfos = &unicastAcceptInfos_;
      pDropInfos = &unicastDropInfos_;

      pMutexAcceptTable = &mutexUnicastPacketAcceptTable_;
      pMutexDropTable = &mutexUnicastPacketDropTable_;
    }

  // detetmine the relevant queue based on priority
  std::uint8_t u8QueueIndex{0};

  if(action == OutboundAction::ACCEPT_GOOD)
    {
      std::lock_guard<std::mutex> m(*pMutexAcceptTable);

      auto iter = (*pAcceptInfos)[u8QueueIndex].find(src);

      if(iter == (*pAcceptInfos)[u8QueueIndex].end())
        {
          iter = (*pAcceptInfos)[u8QueueIndex].insert({src,{}}).first;


          (*pAcceptTables)[u8QueueIndex]->addRow(src,
                                                 {Any{src},
                                                  Any{0L},
                                                  Any{0L}});
        }

      auto & bytes = std::get<ACCEPT_COLUMN_NUM_BYTES_TX-1>(iter->second);

      bytes += size;

      (*pAcceptTables)[u8QueueIndex]->setCell(src,
                                              ACCEPT_COLUMN_NUM_BYTES_TX,
                                              Any{bytes});
    }
  else
    {
      std::lock_guard<std::mutex> m(*pMutexDropTable);

      auto iter = (*pDropInfos)[u8QueueIndex].find(src);

      if(iter == (*pDropInfos)[u8QueueIndex].end())
        {
          iter = (*pDropInfos)[u8QueueIndex].insert({src,{}}).first;

          (*pDropTables)[u8QueueIndex]->addRow(src,
                                               {Any{src},
                                                Any{0L},
                                                Any{0L},
                                                Any{0L},
                                                Any{0L},
                                                Any{0L},
                                                Any{0L},
                                                Any{0L},
                                                Any{0L},
                                                Any{0L},
                                                Any{0L},
                                                Any{0L},
                                                Any{0L},
                                                Any{0L},
                                                Any{0L}});
        }

      switch(action)
        {
        case OutboundAction::DROP_TOO_BIG:
          {
            auto & bytes = std::get<DROP_COLUMN_TOO_BIG-1>(iter->second);

            bytes += size;

            (*pDropTables)[u8QueueIndex]->setCell(src,
                                                  DROP_COLUMN_TOO_BIG,
                                                  Any{bytes});
          }
          break;
        case OutboundAction::DROP_OVERFLOW:
          {
            auto & bytes = std::get<DROP_COLUMN_QUEUE_OVERFLOW-1>(iter->second);

            bytes += size;

            (*pDropTables)[u8QueueIndex]->setCell(src,
                                                  DROP_COLUMN_QUEUE_OVERFLOW,
                                                  Any{bytes});
          }
          break;

        case OutboundAction::DROP_TX_OFF:
          {
            auto & bytes = std::get<DROP_COLUMN_TX_OFF-1>(iter->second);

            bytes += size;

            (*pDropTables)[u8QueueIndex]->setCell(src,
                                                  DROP_COLUMN_TX_OFF,
                                                  Any{bytes});
          }
          break;

        default:
          break;
        }
    }
}

void EMANE::Models::BentPipe::PacketStatusPublisher::outbound(NEMId src,
                                                              const MessageComponents & components,
                                                              OutboundAction action)
{
  for(const auto & component : components)
    {
      outbound(src,
               component.getDestination(),
               component.getData().size(),
               action);
    }
}
