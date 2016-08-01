/*
 * Copyright (c) 2013,2016 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "ieee80211abgmacheadermessage.h"
#include "ieee80211abgmacheader.pb.h"
#include "msgtypes.h"
#include "utils.h"

namespace {

  inline EMANEMessage::IEEE80211ABGMACHeader_MessageType convertMessageType(std::uint8_t type)
  {
    switch(type)
      {
      case EMANE::Models::IEEE80211ABG::MSG_TYPE_NONE:
        return EMANEMessage::IEEE80211ABGMACHeader_MessageType::IEEE80211ABGMACHeader_MessageType_NONE;

      case EMANE::Models::IEEE80211ABG::MSG_TYPE_BROADCAST_DATA:
        return EMANEMessage::IEEE80211ABGMACHeader_MessageType::IEEE80211ABGMACHeader_MessageType_BROADCAST_DATA;

      case EMANE::Models::IEEE80211ABG::MSG_TYPE_UNICAST_DATA:
        return EMANEMessage::IEEE80211ABGMACHeader_MessageType::IEEE80211ABGMACHeader_MessageType_UNICAST_DATA;

      case EMANE::Models::IEEE80211ABG::MSG_TYPE_UNICAST_RTS_CTS_DATA:
        return EMANEMessage::IEEE80211ABGMACHeader_MessageType::IEEE80211ABGMACHeader_MessageType_UNICAST_RTS_CTS_DATA;

      case EMANE::Models::IEEE80211ABG::MSG_TYPE_UNICAST_CTS_CTRL:
        return EMANEMessage::IEEE80211ABGMACHeader_MessageType::IEEE80211ABGMACHeader_MessageType_UNICAST_CTS_CTRL;

      default:
        return EMANEMessage::IEEE80211ABGMACHeader_MessageType::IEEE80211ABGMACHeader_MessageType_NONE;
      }
  }

  inline std::uint8_t convertMessageType(EMANEMessage::IEEE80211ABGMACHeader_MessageType type)
  {
    switch(type)
      {
      case EMANEMessage::IEEE80211ABGMACHeader_MessageType::IEEE80211ABGMACHeader_MessageType_NONE:
        return EMANE::Models::IEEE80211ABG::MSG_TYPE_NONE;

      case EMANEMessage::IEEE80211ABGMACHeader_MessageType::IEEE80211ABGMACHeader_MessageType_BROADCAST_DATA:
        return EMANE::Models::IEEE80211ABG::MSG_TYPE_BROADCAST_DATA;

      case EMANEMessage::IEEE80211ABGMACHeader_MessageType::IEEE80211ABGMACHeader_MessageType_UNICAST_DATA:
        return EMANE::Models::IEEE80211ABG::MSG_TYPE_UNICAST_DATA;

      case EMANEMessage::IEEE80211ABGMACHeader_MessageType::IEEE80211ABGMACHeader_MessageType_UNICAST_RTS_CTS_DATA:
        return EMANE::Models::IEEE80211ABG::MSG_TYPE_UNICAST_RTS_CTS_DATA;

      case EMANEMessage::IEEE80211ABGMACHeader_MessageType::IEEE80211ABGMACHeader_MessageType_UNICAST_CTS_CTRL:
        return EMANE::Models::IEEE80211ABG::MSG_TYPE_UNICAST_CTS_CTRL;

      default:
        return EMANE::Models::IEEE80211ABG::MSG_TYPE_NONE;
      }
  }
}

class EMANE::Models::IEEE80211ABG::MACHeaderMessage::Implementation
{
public:
  Implementation(std::uint8_t  u8MessageType,
                 std::uint8_t  u8NumRetries,
                 std::uint16_t u16DataRateIndex,
                 std::uint16_t u16SequenceNumber,
                 std::uint16_t u16SrcNEM,
                 std::uint16_t u16DstNEM,
                 const Microseconds &durationMicroseconds) :
    u8MessageType_{u8MessageType},
    u8NumRetries_{u8NumRetries},
    u16DataRateIndex_{u16DataRateIndex},
    u16SequenceNumber_{u16SequenceNumber},
    u16SrcNEM_{u16SrcNEM},
    u16DstNEM_{u16DstNEM},
    durationMicroseconds_{durationMicroseconds}
  { }

  std::uint8_t getMessageType() const
  {
    return u8MessageType_;
  }

  std::uint8_t getNumRetries() const
  {
    return u8NumRetries_;
  }

  std::uint16_t getDataRateIndex() const
  {
    return u16DataRateIndex_;
  }

  std::uint16_t getSequenceNumber() const
  {
    return u16SequenceNumber_;
  }

  std::uint16_t getSrcNEM() const
  {
    return u16SrcNEM_;
  }

  std::uint16_t getDstNEM() const
  {
    return u16DstNEM_;
  }

  EMANE::Microseconds getDurationMicroseconds() const
  {
    return durationMicroseconds_;
  }

private:
  const std::uint8_t  u8MessageType_;
  const std::uint8_t  u8NumRetries_;
  const std::uint16_t u16DataRateIndex_;
  const std::uint16_t u16SequenceNumber_;
  const std::uint16_t u16SrcNEM_;
  const std::uint16_t u16DstNEM_;
  EMANE::Microseconds durationMicroseconds_;
};


EMANE::Models::IEEE80211ABG::MACHeaderMessage::MACHeaderMessage(std::uint8_t  u8MessageType,
                                                                std::uint8_t  u8NumRetries,
                                                                std::uint16_t u16DataRateIndex,
                                                                std::uint16_t u16SequenceNumber,
                                                                std::uint16_t u16SrcNEM,
                                                                std::uint16_t u16DstNEM,
                                                                const Microseconds &durationMicroseconds) :

  pImpl_{new Implementation{u8MessageType,
      u8NumRetries,
      u16DataRateIndex,
      u16SequenceNumber,
      u16SrcNEM,
      u16DstNEM,
      durationMicroseconds}}

{ }


EMANE::Models::IEEE80211ABG::MACHeaderMessage::MACHeaderMessage(const void * p, size_t len)
{
  EMANEMessage::IEEE80211ABGMACHeader message;

  if(!message.ParseFromArray(p, len))
    {
      throw SerializationException("unable to deserialize MACHeaderMessage");
    }

  pImpl_.reset(new Implementation{convertMessageType(message.messagetype()),
        static_cast<std::uint8_t>(message.numretries()),
        static_cast<std::uint16_t>(message.datarateindex()),
        static_cast<std::uint16_t>(message.sequencenumber()),
        static_cast<std::uint16_t>(message.srcnem()),
        static_cast<std::uint16_t>(message.dstnem()),
        Microseconds{message.durationmicroseconds()}});
}


EMANE::Models::IEEE80211ABG::MACHeaderMessage::~MACHeaderMessage()
{ }


std::uint8_t EMANE::Models::IEEE80211ABG::MACHeaderMessage::getMessageType() const
{
  return pImpl_->getMessageType();
}


std::uint8_t EMANE::Models::IEEE80211ABG::MACHeaderMessage::getNumRetries() const
{
  return pImpl_->getNumRetries();

}

std::uint16_t EMANE::Models::IEEE80211ABG::MACHeaderMessage::getDataRateIndex() const
{
  return pImpl_->getDataRateIndex();
}

std::uint16_t EMANE::Models::IEEE80211ABG::MACHeaderMessage::getSequenceNumber() const
{
  return pImpl_->getSequenceNumber();
}

std::uint16_t EMANE::Models::IEEE80211ABG::MACHeaderMessage::getSrcNEM() const
{
  return pImpl_->getSrcNEM();
}

std::uint16_t EMANE::Models::IEEE80211ABG::MACHeaderMessage::getDstNEM() const
{
  return pImpl_->getDstNEM();
}

EMANE::Microseconds EMANE::Models::IEEE80211ABG::MACHeaderMessage::getDurationMicroseconds() const
{
  return pImpl_->getDurationMicroseconds();
}



EMANE::Serialization EMANE::Models::IEEE80211ABG::MACHeaderMessage::serialize() const
{
  Serialization serialization;

  EMANEMessage::IEEE80211ABGMACHeader message;

  message.set_messagetype(convertMessageType(pImpl_->getMessageType()));

  message.set_numretries(pImpl_->getNumRetries());

  message.set_datarateindex(pImpl_->getDataRateIndex());

  message.set_sequencenumber(pImpl_->getSequenceNumber());

  message.set_srcnem(pImpl_->getSrcNEM());

  message.set_dstnem(pImpl_->getDstNEM());

  message.set_durationmicroseconds(pImpl_->getDurationMicroseconds().count());

  if(!message.SerializeToString(&serialization))
    {
      throw SerializationException("unable to serialize MACHeaderMessage");
    }

  return serialization;
}
