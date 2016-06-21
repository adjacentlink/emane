/*
 * Copyright (c) 2013-2014,2016 - Adjacent Link LLC, Bridgewater,
 * New Jersey
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

#include "emane/downstreampacket.h"
#include "emane/utils/vectorio.h"
#include "emane/net.h"
#include "emane/event.h"

#include <vector>
#include <list>

class EMANE::DownstreamPacket::Implementation
{
public:
  Implementation():
    totalLengthBytes_{},
    info_{0,0,0,{}}{}

  Implementation(const PacketInfo & info, const void * buf, size_t size):
    totalLengthBytes_{},
    info_{info}
  {
    prepend(buf,size);
  }

  void prepend(const void * buf, size_t size)
  {
    const unsigned char * c = static_cast<const unsigned char *>(buf);

    segmentList_.push_front(PacketSegment(&c[0],&c[size]));

    totalLengthBytes_ += size;
  }

  void prependLengthPrefixFraming(std::uint16_t u16Length)
  {
    std::uint16_t u16LengthNet{HTONS(u16Length)};

    auto c = reinterpret_cast<const std::uint8_t *>(&u16LengthNet);

    segmentList_.push_front(PacketSegment{&c[0],&c[sizeof(u16Length)]});

    totalLengthBytes_ += sizeof(u16Length);
  }


  const void * get()
  {
    if(!segmentList_.empty() && segmentList_.begin()->size())
      {
        if(segmentList_.size() > 1)
          {
            PacketSegment combinedSeg;

            combinedSeg.reserve(totalLengthBytes_);

            SegmentList::iterator iter = segmentList_.begin();

            for(;iter !=  segmentList_.end(); ++iter)
              {
                combinedSeg.insert(combinedSeg.end(),iter->begin(),iter->end());
              }

            segmentList_.erase(++segmentList_.begin(),segmentList_.end());

            segmentList_.begin()->swap(combinedSeg);
          }

        return &(*segmentList_.begin())[0];
      }

    return 0;
  }

  size_t length() const
  {
    return totalLengthBytes_;
  }

  const PacketInfo & getPacketInfo() const
  {
    return info_;
  }

  Utils::VectorIO getVectorIO() const
  {
    Utils::VectorIO vectorIO{};

    vectorIO.reserve(segmentList_.size());

    for(const auto & segment : segmentList_)
      {
        vectorIO.push_back({const_cast<std::uint8_t *>(&segment[0]),segment.size()});
      }

    return vectorIO;
  }

  void attachEvent(NEMId nemId, const Event & event)
  {
    attachedEvents_.push_back(std::make_tuple(nemId,
                                              event.getEventId(),
                                              event.serialize()));
  }

  const EventSerializations & getEventSerializations()
  {
    return attachedEvents_;
  }

private:
  typedef std::vector<std::uint8_t> PacketSegment;
  typedef std::list<PacketSegment> SegmentList;
  typedef std::list<std::tuple<NEMId,EventId,std::string>> AttachedEvents;
  SegmentList segmentList_;
  PacketSegment::size_type totalLengthBytes_;
  PacketInfo info_;
  AttachedEvents attachedEvents_;
};

EMANE::DownstreamPacket::DownstreamPacket(const  EMANE::PacketInfo & info,
                                          const void * buf,
                                          size_t size):
  pImpl_{new Implementation{info,buf,size}}{}


EMANE::DownstreamPacket::DownstreamPacket(const DownstreamPacket & pkt):
  pImpl_{pkt.pImpl_}{}

EMANE::DownstreamPacket::DownstreamPacket(DownstreamPacket && pkt):
  pImpl_{std::move(pkt.pImpl_)}{}

EMANE::DownstreamPacket::~DownstreamPacket(){}

EMANE::DownstreamPacket & EMANE::DownstreamPacket::operator=(const DownstreamPacket & pkt)
{
  pImpl_ = pkt.pImpl_;
  return *this;
}

EMANE::DownstreamPacket & EMANE::DownstreamPacket::operator=(DownstreamPacket && pkt)
{
  pImpl_ = std::move(pkt.pImpl_);
  return *this;
}

void EMANE::DownstreamPacket::prepend(const void * buf, size_t size)
{
  pImpl_->prepend(buf,size);
}

void EMANE::DownstreamPacket::prependLengthPrefixFraming(std::uint16_t u16Length)
{
  pImpl_->prependLengthPrefixFraming(u16Length);
}


EMANE::Utils::VectorIO EMANE::DownstreamPacket::getVectorIO() const
{
  return pImpl_->getVectorIO();
}

size_t EMANE::DownstreamPacket::length() const
{
  return pImpl_->length();
}

const  EMANE::PacketInfo & EMANE::DownstreamPacket::getPacketInfo() const
{
  return pImpl_->getPacketInfo();
}

void EMANE::DownstreamPacket::attachEvent(NEMId nemId,const Event & event)
{
  pImpl_->attachEvent(nemId,event);
}

const EMANE::DownstreamPacket::EventSerializations &
EMANE::DownstreamPacket::getEventSerializations() const
{
  return pImpl_->getEventSerializations();
}
