/*
 * Copyright (c) 2013,2016 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2008 - DRS CenGen, LLC, Columbia, Maryland
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

#include "emane/upstreampacket.h"
#include "emane/net.h"

#include <memory>

class EMANE::UpstreamPacket::Implementation
{
public:
  Implementation():
    head_{},
    pShared_{std::make_shared<Shared>()}{}

  Implementation(const PacketInfo & info, const void * buf, size_t len):
    head_{},
    pShared_{std::make_shared<Shared>()}
  {
    const unsigned char * c = static_cast<const unsigned char *>(buf);
    pShared_->info_ = info;
    pShared_->packetSegment_.reserve(len);
    pShared_->packetSegment_.insert(pShared_->packetSegment_.end(),&c[0],&c[len]);
  }

  Implementation(const PacketInfo & info,const Utils::VectorIO & vectorIO):
    head_{},
    pShared_{std::make_shared<Shared>()}
  {
    pShared_->info_ = info;

    for(const auto & iov : vectorIO)
      {
        pShared_->packetSegment_.insert(pShared_->packetSegment_.end(),
                                        &static_cast<const std::uint8_t *>(iov.iov_base)[0],
                                        &static_cast<const std::uint8_t *>(iov.iov_base)[iov.iov_len]);
      }
  }

  size_t strip(size_t size)
  {
    if(head_ + size < pShared_->packetSegment_.size())
      {
        head_ += size;
      }
    else
      {
        size = pShared_->packetSegment_.size() - head_;
        head_ += size;
      }

    return size;
  }


  std::uint16_t stripLengthPrefixFraming()
  {
    std::uint16_t u16LengthPrefixFraming{};

    if(head_ + sizeof(uint16_t) < pShared_->packetSegment_.size())
      {
        u16LengthPrefixFraming =
          NTOHS(*reinterpret_cast<const std::uint16_t *>(&pShared_->packetSegment_[head_]));

        head_ += sizeof(uint16_t);
      }

    return u16LengthPrefixFraming;
  }


  const void * get() const
  {
    return(head_ <  pShared_->packetSegment_.size()) ? &pShared_->packetSegment_[head_] : 0;
  }


  size_t length() const
  {
    return pShared_->packetSegment_.size() - head_;
  }


  const PacketInfo &  getPacketInfo() const
  {
    return pShared_->info_;
  }

private:
  typedef std::vector<unsigned char> PacketSegment;

  class Shared
  {
  public:
    PacketSegment packetSegment_;
    PacketInfo info_{0,0,0,{}};
  };

  PacketSegment::size_type head_;
  std::shared_ptr<Shared> pShared_;
};


EMANE::UpstreamPacket::UpstreamPacket(const  EMANE::PacketInfo & info,
                                      const void * buf,
                                      size_t size):
  pImpl_{new Implementation{info,buf,size}}{}

EMANE::UpstreamPacket::UpstreamPacket(const  EMANE::PacketInfo & info,
                                      const Utils::VectorIO & vectorIO):
  pImpl_{new Implementation{info,vectorIO}}{}

EMANE::UpstreamPacket::UpstreamPacket(const UpstreamPacket & pkt):
  pImpl_{new Implementation{*pkt.pImpl_}}{}

EMANE::UpstreamPacket::UpstreamPacket(UpstreamPacket && pkt):
  pImpl_{std::move(pkt.pImpl_)}
 {
   pkt.pImpl_.reset(new Implementation{});
 }

EMANE::UpstreamPacket::~UpstreamPacket(){}

EMANE::UpstreamPacket & EMANE::UpstreamPacket::operator=(const UpstreamPacket & pkt)
{
  pImpl_.reset(new Implementation{*pkt.pImpl_});
  return *this;
}

EMANE::UpstreamPacket & EMANE::UpstreamPacket::operator=(UpstreamPacket && pkt)
{
  pImpl_ = std::move(pkt.pImpl_);
  pkt.pImpl_.reset(new Implementation{});
  return *this;
}


size_t EMANE::UpstreamPacket::strip(size_t size)
{
  return pImpl_->strip(size);
}


std::uint16_t EMANE::UpstreamPacket::stripLengthPrefixFraming()
{
  return pImpl_->stripLengthPrefixFraming();
}


const void * EMANE::UpstreamPacket::get() const
{
  return pImpl_->get();
}


size_t EMANE::UpstreamPacket::length() const
{
  return pImpl_->length();
}


const EMANE::PacketInfo &  EMANE::UpstreamPacket::getPacketInfo() const
{
  return pImpl_->getPacketInfo();
}
