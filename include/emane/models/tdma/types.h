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

#ifndef EMANETDMARADIOMODELTYPES_HEADER_
#define EMANETDMARADIOMODELTYPES_HEADER_

#include "emane/types.h"

#include <vector>
#include <set>
#include <list>

namespace EMANE
{
  namespace Models
  {
    namespace TDMA
    {
      using SlotIndex = uint64_t;
      using FrameIndex = uint64_t;

      /**
       * @struct PacketMetaInfo
       *
       * @brief Received over-the-air message information.
       *
       * Contains information on a received over-the-air message. This
       * information is send by BaseModel to a scheduler module for
       * internal processing.
       */
      struct PacketMetaInfo
      {
        NEMId source_; /**< Transmitter NEM */
        SlotIndex slot_; /**< Absolute slot index */
        double dRxPowerdBm_; /**< Receive power */
        double dSINR_; /**< Receive SINR */
        std::uint64_t u64DataRatebps_; /**< Receive datarate */
      };

      /**
       * @struct QueueInfo
       *
       * @brief %Queue status information.
       *
       * Contains information on the number of packets and bytes
       * contained in a queue. A partial packet due to fragmentation,
       * that remains in a queue is counted as whole packet. Only the
       * number of byes remaining in the partial packet is counted
       * when reporting the total bytes in the queue.
       */
      struct QueueInfo
      {
        std::uint8_t u8QueueId_; /**< %Queue Id */
        std::uint64_t u64Packets_; /**< Number of packets in queue */
        std::uint64_t u64Bytes_; /**< Number of bytes in queue */
      };

      /**
       * @struct TxSlotInfo
       *
       * @brief Transmit slot information.
       *
       * Contains information about a transmit slot. This information
       * originates from a scheduler module and is used by the
       * BaseModel to process transmit opportunities.
       */
      struct TxSlotInfo
      {
        std::uint64_t u64AbsoluteSlotIndex_; /**< Absolute slot index of slot */
        std::uint32_t u32RelativeIndex_; /**< Relative slot index in the multiframe of slot */
        std::uint32_t u32RelativeSlotIndex_; /**< Relative slot index in the frame of slot */
        std::uint32_t u32RelativeFrameIndex_; /**< Relative frame index in the multiframe of slot */
        TimePoint timePoint_; /**< Start of slot time*/
        std::uint64_t u64FrequencyHz_; /**< Transmit frequency assigned */
        std::uint64_t u64DataRatebps_; /**< Transmit datarate assigned */
        std::uint8_t  u8QueueId_; /**< %Queue id to dequeue traffic */
        double dPowerdBm_; /**< Transmit power assigned */
        NEMId destination_; /**< Destination target to dequeue traffic */
      };


      /**
       * @struct RxSlotInfo
       *
       * @brief Receive slot information.
       *
       * Contains information about the current Rx slot. This
       * information is used by the BaseModel to determine if an
       * over-the-air message was received on the correct slot.
       */
      struct RxSlotInfo
      {
        SlotIndex u64AbsoluteSlotIndex_; /**< Absolute slot index of slot */
        std::uint32_t u32RelativeIndex_; /**< Relative slot index in the multiframe of slot */
        std::uint32_t u32RelativeSlotIndex_; /**< Relative slot index in the frame of slot */
        std::uint32_t u32RelativeFrameIndex_; /**< Relative frame index in the multiframe of slot */
        TimePoint timePoint_; /**< Start of slot time*/
        std::uint64_t u64FrequencyHz_; /**< Receive frequency assigned */
      };

      /**
       * @struct SlotInfo
       *
       * @brief Current slot information.
       *
       * Contains information about the current slot. This information
       * is used by the BaseModel to identify where it is in the
       * current slot structure.
       */
      struct SlotInfo
      {
        enum class Type
        {
          TX, /**< Transmit slot */
          RX, /**< Recieve slot */
          IDLE /**< Idle slot */
        };
        SlotIndex u64AbsoluteSlotIndex_; /**< Absolute slot index of slot */
        std::uint32_t u32RelativeIndex_; /**< Relative slot index in the multiframe of slot */
        std::uint32_t u32RelativeSlotIndex_; /**< Relative slot index in the frame of slot */
        std::uint32_t u32RelativeFrameIndex_; /**< Relative frame index in the multiframe of slot */
        TimePoint timePoint_; /**< Start of slot time*/
        Type type_; /**< Type of slot */
      };

      using QueueInfos = std::vector<QueueInfo>;

      using TxSlotInfos = std::list<TxSlotInfo>;

      using Frequencies = std::set<std::uint64_t>;
    }
  }
}

#endif // EMANETDMARADIOMODELTYPES_HEADER_
