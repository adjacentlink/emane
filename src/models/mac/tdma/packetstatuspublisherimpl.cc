/*
 * Copyright (c) 2015 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "packetstatuspublisherimpl.h"
#include "priority.h"

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
      "Flow Control",
      "Too Big",
      "Slot Error",
      "Miss Fragment"
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
      DROP_COLUMN_FLOW_CONTROL = 7,
      DROP_COLUMN_TOO_BIG = 8,
      DROP_COLUMN_SLOT_ERROR = 9,
      DROP_COLUMN_MISS_FRAGMENT = 10
    };

}
EMANE::Models::TDMA::PacketStatusPublisherImpl::PacketStatusPublisherImpl(){}

EMANE::Models::TDMA::PacketStatusPublisherImpl::~PacketStatusPublisherImpl(){}

void EMANE::Models::TDMA::PacketStatusPublisherImpl::registerStatistics(StatisticRegistrar & statisticRegistrar)
{
  for(int queueIndex = 0; queueIndex < QUEUE_COUNT; ++queueIndex)
    {
      broadcastAcceptTables_[queueIndex] =
        statisticRegistrar.registerTable<NEMId>("BroadcastByteAcceptTable" + std::to_string(queueIndex),
                                                PacketAcceptLabels,
                                                [this,queueIndex](StatisticTablePublisher * pTable)
                                                {
                                                  std::lock_guard<std::mutex> m(mutexBroadcastPacketAcceptTable_);
                                                  broadcastAcceptTables_[queueIndex]->clear();
                                                  pTable->clear();
                                                },
                                                "Broadcast bytes accepted");

      unicastAcceptTables_[queueIndex] =
        statisticRegistrar.registerTable<NEMId>("UnicastByteAcceptTable" + std::to_string(queueIndex),
                                                PacketAcceptLabels,
                                                [this,queueIndex](StatisticTablePublisher * pTable)
                                                {
                                                  std::lock_guard<std::mutex> m(mutexUnicastPacketAcceptTable_);
                                                  unicastAcceptTables_[queueIndex]->clear();
                                                  pTable->clear();
                                                },
                                                "Unicast bytes accepted");

      broadcastDropTables_[queueIndex] =
        statisticRegistrar.registerTable<NEMId>("BroadcastByteDropTable" + std::to_string(queueIndex),
                                                PacketDropLabels,
                                                [this,queueIndex](StatisticTablePublisher * pTable)
                                                {
                                                  std::lock_guard<std::mutex> m(mutexBroadcastPacketDropTable_);
                                                  broadcastDropTables_[queueIndex]->clear();
                                                  pTable->clear();
                                                },
                                                "Broadcast bytes dropped");

      unicastDropTables_[queueIndex] =
        statisticRegistrar.registerTable<NEMId>("UnicastByteDropTable" + std::to_string(queueIndex),
                                                PacketDropLabels,
                                                [this,queueIndex](StatisticTablePublisher * pTable)
                                                {
                                                  std::lock_guard<std::mutex> m(mutexUnicastPacketDropTable_);
                                                  unicastDropTables_[queueIndex]->clear();
                                                  pTable->clear();
                                                },
                                                "Unicast bytes dropped");
    }
}

void EMANE::Models::TDMA::PacketStatusPublisherImpl::inbound(NEMId src,
                                                             const MessageComponent & component,
                                                             InboundAction action)
{
  inbound(src,
          component.getDestination(),
          component.getPriority(),
          component.getData().size(),
          action);
}

void  EMANE::Models::TDMA::PacketStatusPublisherImpl::inbound(NEMId src,
                                                              const MessageComponents & components,
                                                              InboundAction action)
{
  for(const auto & component : components)
    {
      inbound(src,
              component.getDestination(),
              component.getPriority(),
              component.getData().size(),
              action);
    }
}



void EMANE::Models::TDMA::PacketStatusPublisherImpl::inbound(NEMId src,
                                                             NEMId dst,
                                                             Priority priority,
                                                             size_t size,
                                                             InboundAction action)
{
  TableArray * pAcceptTables{};
  TableArray * pDropTables{};

  AcceptInfoArrary * pAcceptInfos{};
  DropInfoArrary  * pDropInfos{};

  if(dst == NEM_BROADCAST_MAC_ADDRESS)
    {
      pAcceptTables = &broadcastAcceptTables_;
      pDropTables = &broadcastDropTables_;

      pAcceptInfos = &broadcastAcceptInfos_;
      pDropInfos = &broadcastDropInfos_;
    }
  else
    {
      pAcceptTables = &unicastAcceptTables_;
      pDropTables = &unicastDropTables_;

      pAcceptInfos = &unicastAcceptInfos_;
      pDropInfos = &unicastDropInfos_;
    }

  // detetmine the relevant queue based on priority
  std::uint8_t u8QueueIndex{priorityToQueue(priority)};

  if(action == InboundAction::ACCEPT_GOOD)
    {
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

        case InboundAction::DROP_SLOT_NOT_RX:
        case InboundAction::DROP_SLOT_MISSED_RX:
          {
            auto & bytes = std::get<DROP_COLUMN_SLOT_ERROR-1>(iter->second);

            bytes += size;

            (*pDropTables)[u8QueueIndex]->setCell(src,
                                                  DROP_COLUMN_SLOT_ERROR,
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
        default:
          break;
        }
    }
}


void EMANE::Models::TDMA::PacketStatusPublisherImpl::outbound(NEMId src,
                                                              NEMId dst,
                                                              Priority priority,
                                                              size_t size,
                                                              OutboundAction action)
{
  TableArray * pAcceptTables{};
  TableArray * pDropTables{};

  AcceptInfoArrary * pAcceptInfos{};
  DropInfoArrary  * pDropInfos{};

  if(dst == NEM_BROADCAST_MAC_ADDRESS)
    {
      pAcceptTables = &broadcastAcceptTables_;
      pDropTables = &broadcastDropTables_;

      pAcceptInfos = &broadcastAcceptInfos_;
      pDropInfos = &broadcastDropInfos_;
    }
  else
    {
      pAcceptTables = &unicastAcceptTables_;
      pDropTables = &unicastDropTables_;

      pAcceptInfos = &unicastAcceptInfos_;
      pDropInfos = &unicastDropInfos_;
    }

  // detetmine the relevant queue based on priority
  std::uint8_t u8QueueIndex{priorityToQueue(priority)};


   if(action == OutboundAction::ACCEPT_GOOD)
    {
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
        default:
          break;
        }
     }
}

void EMANE::Models::TDMA::PacketStatusPublisherImpl::outbound(NEMId src,
                                                              const MessageComponents & components,
                                                              OutboundAction action)
{
 for(const auto & component : components)
    {
      outbound(src,
               component.getDestination(),
               component.getPriority(),
               component.getData().size(),
               action);
    }
}
