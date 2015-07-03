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

#include "queue.h"

EMANE::Models::TDMA::Queue::Queue():
  u16QueueDepth_{},
  bFragment_{},
  u64Counter_{},
  currentBytes_{},
  bIsControl_{}{}


void EMANE::Models::TDMA::Queue::initialize(std::uint16_t u16QueueDepth,
                                            bool bFragment,
                                            bool bAggregate,
                                            bool bIsControl)
{
  u16QueueDepth_ = u16QueueDepth;
  bFragment_ = bFragment;
  bIsControl_ = bIsControl;
  bAggregate_ = bAggregate;
}

std::pair<std::unique_ptr<EMANE::DownstreamPacket>,bool>
  EMANE::Models::TDMA::Queue::enqueue(DownstreamPacket && pkt)
{
  std::unique_ptr<DownstreamPacket> pDroppedPacket{};
  bool bDroppedPacket{};

  DownstreamPacket * pPkt{new DownstreamPacket{std::move(pkt)}};

  NEMId dest{pPkt->getPacketInfo().getDestination()};

  if(queue_.size() == u16QueueDepth_)
    {
      auto const & entry = queue_.begin();

      // ownership transfer
      pDroppedPacket.reset(entry->second.first);

      // ownership transfer
      std::unique_ptr<MetaInfo> pDroppedMetaInfo{entry->second.second};

      bDroppedPacket = true;

      auto dst = pDroppedPacket->getPacketInfo().getDestination();

      destQueue_[dst].erase(entry->first);

      queue_.erase(entry->first);

      // update current bytes in queue by subtracting off amount
      // remaining in dropped packet
      currentBytes_ -= pDroppedPacket->length() - pDroppedMetaInfo->offset_;
    }

  MetaInfo * pMetaInfo = new MetaInfo;

  queue_.insert(std::make_pair(u64Counter_,std::make_pair(pPkt,pMetaInfo)));

  auto iter = destQueue_.find(dest);

  if(iter == destQueue_.end())
    {
      iter = destQueue_.insert({dest,PacketQueue{}}).first;
    }

  currentBytes_ += pPkt->length();

  iter->second.insert(std::make_pair(u64Counter_,std::make_pair(pPkt,pMetaInfo)));

  ++u64Counter_;

  return {std::move(pDroppedPacket),bDroppedPacket};
}

std::tuple<EMANE::Models::TDMA::MessageComponents,
           size_t,
           std::list<std::unique_ptr<EMANE::DownstreamPacket>>>
  EMANE::Models::TDMA::Queue::dequeue(size_t requestedBytes, NEMId destination,bool bDrop)
{
  MessageComponents components{};
  size_t totalBytes{};
  std::list<std::unique_ptr<DownstreamPacket>> dropped;

  while(totalBytes <= requestedBytes)
    {
      if(destination)
        {
          auto iter = destQueue_.find(destination);

          // check for a destination queue that is not empty
          if(iter != destQueue_.end() && !iter->second.empty())
            {
              auto const & entry = iter->second.begin();

              auto & pPacket = std::get<0>(entry->second);
              auto & pMetaInfo = std::get<1>(entry->second);

              if(pPacket->length() - pMetaInfo->offset_ <= requestedBytes - totalBytes)
                {
                  if(pMetaInfo->offset_)
                    {
                      auto ret = fragmentPacket(pPacket,
                                                pMetaInfo,
                                                entry->first,
                                                requestedBytes - totalBytes);

                      totalBytes += ret.second;

                      components.push_back(std::move(ret.first));
                    }
                  else
                    {
                      components.push_back({bIsControl_ ?
                            MessageComponent::Type::CONTROL :
                            MessageComponent::Type::DATA,
                            pPacket->getPacketInfo().getDestination(),
                            pPacket->getPacketInfo().getPriority(),
                            pPacket->getVectorIO(),
                            pMetaInfo->index_,
                            pMetaInfo->offset_,
                            entry->first,
                            false});


                      totalBytes += pPacket->length() - pMetaInfo->offset_;
                    }

                  delete pPacket;
                  delete pMetaInfo;

                  // remove from packet queue using queue sequence
                  queue_.erase(entry->first);

                  // remove from destination queue using iterator
                  iter->second.erase(entry);

                  // if aggregation is disabled don't look further
                  if(!bAggregate_)
                    {
                      break;
                    }
                }
              else
                {
                  if(bFragment_)
                    {
                      auto ret = fragmentPacket(pPacket,
                                                pMetaInfo,
                                                entry->first,
                                                requestedBytes - totalBytes);

                      totalBytes += ret.second;

                      components.push_back(std::move(ret.first));

                      break;
                    }
                  else
                    {
                      if(bDrop && components.empty())
                        {
                          // drop packet - too large and fragmentation
                          // is disabled
                          currentBytes_ -= pPacket->length();

                          // transfer ownership to std::unique_ptr
                          dropped.push_back(std::unique_ptr<DownstreamPacket>{pPacket});

                          delete pMetaInfo;

                          // remove from packet queue using queue sequence
                          queue_.erase(entry->first);

                          // remove from destination queue using iterator
                          iter->second.erase(entry);
                        }
                      else
                        {
                          break;
                        }
                    }
                }
            }
          else
            {
              break;
            }
        }
      else if(!queue_.empty())
        {
          auto const & entry = queue_.begin();

          auto & pPacket = std::get<0>(entry->second);
          auto & pMetaInfo = std::get<1>(entry->second);

          NEMId dst{pPacket->getPacketInfo().getDestination()};

          if(pPacket->length() - pMetaInfo->offset_ <= requestedBytes - totalBytes)
            {
              if(pMetaInfo->offset_)
                {
                  auto ret = fragmentPacket(pPacket,
                                            pMetaInfo,
                                            entry->first,
                                            requestedBytes - totalBytes);

                  totalBytes += ret.second;

                  components.push_back(std::move(ret.first));
                }
              else
                {
                  components.push_back({bIsControl_ ?
                        MessageComponent::Type::CONTROL :
                        MessageComponent::Type::DATA,
                        dst,
                        pPacket->getPacketInfo().getPriority(),
                        pPacket->getVectorIO(),
                        pMetaInfo->index_,
                        pMetaInfo->offset_,
                        entry->first,
                        false});

                  totalBytes += pPacket->length() - pMetaInfo->offset_;
                }

              delete pPacket;
              delete pMetaInfo;

              // remove from destination queue using queue sequence
              destQueue_[dst].erase(entry->first);

              // remove for packet queue using iterator
              queue_.erase(entry);

              // if aggregation is disabled don't look further
              if(!bAggregate_)
                {
                  break;
                }
            }
          else
            {
              if(bFragment_)
                {
                  auto ret = fragmentPacket(pPacket,
                                            pMetaInfo,
                                            entry->first,
                                            requestedBytes - totalBytes);

                  totalBytes += ret.second;

                  components.push_back(std::move(ret.first));

                  break;
                }
              else
                {
                  if(bDrop && components.empty())
                    {
                      // drop packet - too large and fragmentation
                      // is disabled
                      currentBytes_ -= pPacket->length();

                      // transfer ownership to std::unique_ptr
                      dropped.push_back(std::unique_ptr<DownstreamPacket>{pPacket});

                      delete pMetaInfo;

                      // remove from destination queue using queue sequence
                      destQueue_[dst].erase(entry->first);

                      // remove for packet queue using iterator
                      queue_.erase(entry);
                    }
                  else
                    {
                      break;
                    }
                }
            }
        }
      else
        {
          break;
        }
    }

  // reduce bytes in queue by the total being returned, dropped bytes
  // already accounted for
  currentBytes_ -= totalBytes;

  return std::make_tuple(components,totalBytes,std::move(dropped));
}

std::pair<EMANE::Models::TDMA::MessageComponent,size_t>
EMANE::Models::TDMA::Queue::fragmentPacket(DownstreamPacket * pPacket,
                                           MetaInfo * pMetaInfo,
                                           std::uint64_t u64FragmentSequence,
                                           size_t bytes)
{
  size_t totalBytesVisited{};
  size_t totalBytesCopied{};
  Utils::VectorIO vectorIOs{};

  // packet data is stored in an iovec, need to determine
  // which iovec to start with and proceed to advance when necessary
  for(const auto & entry : pPacket->getVectorIO())
    {
      if(totalBytesCopied < bytes)
        {

          if(totalBytesVisited + entry.iov_len < pMetaInfo->offset_)
            {
              totalBytesVisited += entry.iov_len;
            }
          else
            {
              char * pBuf{reinterpret_cast<char *>(entry.iov_base)};

              auto offset = pMetaInfo->offset_ - totalBytesVisited;

              auto remainder = entry.iov_len - offset;

              // if necessary adjust totalBytesVisited after
              // calculating where in the current entry the fragment
              // begins
              if(totalBytesVisited < pMetaInfo->offset_)
                {
                  totalBytesVisited = pMetaInfo->offset_;
                }

              // clamp the reaminder if there is more remaining than
              // what was request
              if(remainder > bytes)
                {
                  remainder = bytes;
                }

              vectorIOs.push_back(Utils::make_iovec(pBuf+offset,remainder));

              pMetaInfo->offset_ += remainder;
              totalBytesVisited += remainder;
              totalBytesCopied += remainder;
            }
        }
      else
        {
          break;
        }
    }

  MessageComponent component{bIsControl_ ?
      MessageComponent::Type::CONTROL :
      MessageComponent::Type::DATA,
      pPacket->getPacketInfo().getDestination(),
      pPacket->getPacketInfo().getPriority(),
      vectorIOs,
      pMetaInfo->index_,
      pMetaInfo->offset_ - totalBytesCopied,
      u64FragmentSequence,
      totalBytesVisited != pPacket->length()};

  ++pMetaInfo->index_;

  return {component,totalBytesCopied};
}

// packets, bytes
std::tuple<size_t,size_t> EMANE::Models::TDMA::Queue::getStatus() const
{
  return std::make_tuple(queue_.size(),currentBytes_);
}
