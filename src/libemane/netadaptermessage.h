/*
 * Copyright (c) 2016-2017 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2008-2009 - DRS CenGen, LLC, Columbia, Maryland
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
 * * Neither the name of DRS CenGen, LLC nor the names of its
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

#ifndef EMANENETADAPTERMESSAGE_HEADER_
#define EMANENETADAPTERMESSAGE_HEADER_

#include <cstdint>

namespace EMANE
{
  const std::uint16_t NETADAPTER_DATA_MSG = 1;
  const std::uint16_t NETADAPTER_CTRL_MSG = 2;

  /**
   * @class NetAdapterHeader
   *
   * @brief NetAdapter message header
   */
  struct NetAdapterHeader
  {
    std::uint16_t u16Id_;     /**< Event id */
    std::uint32_t u32Length_; /**< Total message length in bytes */
    std::uint8_t  data_[0];   /**< Pointer to message payload */
  } __attribute__((packed));

  /**
   * Convert a NetAdapterHeader to host byte order
   *
   * @param pMsg Message header reference
   *
   * @return Message header reference.  Same as @a pMsg.
   */
  inline
  NetAdapterHeader * NetAdapterHeaderToHost(NetAdapterHeader * pMsg)
  {
    pMsg->u16Id_     =  ntohs(pMsg->u16Id_);
    pMsg->u32Length_ =  ntohl(pMsg->u32Length_);
    return pMsg;
  }

  /**
   * Convert a NetAdapterHeader to network byte order
   *
   * @param pMsg Message header reference
   *
   * @return Message header reference.  Same as @a pMsg.
   */
  inline
  NetAdapterHeader * NetAdapterHeaderToNet(NetAdapterHeader * pMsg)
  {
    pMsg->u16Id_     =  htons(pMsg->u16Id_);
    pMsg->u32Length_ =  htonl(pMsg->u32Length_);
    return pMsg;
  }

  /**
   *
   * @brief data shared between network adapter and nem.
   *
   */
  struct NetAdapterDataMessage
  {
    std::uint16_t u16Src_;
    std::uint16_t u16Dst_;
    std::uint32_t u32DataLen_;
    std::uint32_t u32CtrlLen_;
    std::uint8_t  u8Priority_;
    std::uint8_t  data_[0];
  } __attribute__((packed));

  /**
   *
   * @brief information shared between network adapter and nem.
   *
   */
  struct NetAdapterControlMessage
  {
    std::uint32_t u32CtrlLen_;
    std::uint8_t data_[0];
  } __attribute__((packed));


  inline
  NetAdapterControlMessage * NetAdapterControlMessageToHost(NetAdapterControlMessage * ctrl)
  {
    ctrl->u32CtrlLen_ = ntohl(ctrl->u32CtrlLen_);
    return ctrl;
  }

  inline
  NetAdapterControlMessage *  NetAdapterControlMessageToNet(NetAdapterControlMessage * ctrl)
  {
    ctrl->u32CtrlLen_ = htonl(ctrl->u32CtrlLen_);
    return ctrl;
  }

  /**
   *
   * @brief converts netadapter data message from network to host byte order.
   *
   */
  inline
  NetAdapterDataMessage * NetAdapterDataMessageToHost(NetAdapterDataMessage * pkt)
  {
    pkt->u16Src_      = ntohs(pkt->u16Src_);
    pkt->u16Dst_      = ntohs(pkt->u16Dst_);
    pkt->u32DataLen_  = ntohl(pkt->u32DataLen_);
    pkt->u32CtrlLen_  = ntohl(pkt->u32CtrlLen_);

    return pkt;
  }

  /**
   *
   * @brief converts netadapter data message from host to network byte order.
   *
   */
  inline
  NetAdapterDataMessage * NetAdapterDataMessageToNet(NetAdapterDataMessage * pkt)
  {
    pkt->u16Src_      = htons(pkt->u16Src_);
    pkt->u16Dst_      = htons(pkt->u16Dst_);
    pkt->u32DataLen_  = htonl(pkt->u32DataLen_);
    pkt->u32CtrlLen_  = htonl(pkt->u32CtrlLen_);

    return pkt;
  }


  /**
   *
   * @brief definition of the broadcast address used between network adapter and nem.
   *
   */
  const std::uint16_t NETADAPTER_BROADCAST_ADDRESS = 0xFFFF;
}

#endif //EMANENETADAPTERMESSAGE_HEADER_
