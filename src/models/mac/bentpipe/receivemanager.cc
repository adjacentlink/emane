/*
 * Copyright (c) 2015,2017-2018,2023 - Adjacent Link LLC, Bridgewater,
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

#include "receivemanager.h"
#include "bentpipemessage.pb.h"
#include "emane/utils/spectrumwindowutils.h"

EMANE::Models::BentPipe::ReceiveManager::ReceiveManager(NEMId id,
                                                        TransponderIndex transponderIndex,
                                                        TransponderPacketTransport * pTransponderPacketTransport,
                                                        PlatformServiceProvider * pPlatformService,
                                                        RadioServiceProvider * pRadioService,
                                                        PacketStatusPublisher * pPacketStatusPublisher,
                                                        NeighborStatusPublisher * pNeighborStatusPublisher,
                                                        bool bProcess,
                                                        PCRManager * pPCRManager,
                                                        AntennaIndex rxAntennaIndex,
                                                        const std::chrono::seconds & fragmentCheckThreshold,
                                                        const std::chrono::seconds & fragmentTimeoutThreshold):

                                                        id_{id},
                                                        transponderIndex_{transponderIndex},
                                                        pTransponderPacketTransport_{pTransponderPacketTransport},
                                                        pPlatformService_{pPlatformService},
                                                        pRadioService_{pRadioService},
                                                        pPacketStatusPublisher_{pPacketStatusPublisher},
                                                        pNeighborStatusPublisher_{pNeighborStatusPublisher},
                                                        bProcess_{bProcess},
                                                        pPCRManager_{pPCRManager},
                                                        rxAntennaIndex_{rxAntennaIndex},
                                                        lastEndOfReception_{},
                                                        distribution_{0.0, 1.0},
                                                        fragmentCheckThreshold_{fragmentCheckThreshold},
                                                        fragmentTimeoutThreshold_{fragmentTimeoutThreshold},
                                                        nextEoRCheckTime_{TimePoint::max()}

{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "MACI %03hu BentPipe::ReceiveManager::%s created for transponder %hu set for %s",
                          id_,
                          __func__,
                          transponderIndex,
                          bProcess ? "process" : "ubend");
}

void
EMANE::Models::BentPipe::ReceiveManager::enqueue(BentPipeMessage && otaMessage,
                                                 const PacketInfo & pktInfo,
                                                 size_t length,
                                                 const TimePoint & startOfReception,
                                                 const FrequencySegments & frequencySegments,
                                                 const Microseconds & span,
                                                 const TimePoint & beginTime,
                                                 std::uint64_t u64PacketSequence)
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "MACI %03hu BentPipe::ReceiveManager::%s ",
                          id_,
                          __func__);

  // if the startOfReception of this over-the-air frame is <= to the
  // last end of reception of the receive manager than it is possible
  // to handle this message, so add it to the pending queue
  if(startOfReception >= lastEndOfReception_)
    {
      pendingQueue_.emplace(startOfReception,
                            std::make_tuple(std::move(otaMessage),
                                            pktInfo,
                                            length,
                                            startOfReception,
                                            frequencySegments,
                                            span,
                                            beginTime,
                                            u64PacketSequence));
    }
  else
    {
      pPacketStatusPublisher_->inbound(pktInfo.getSource(),
                                       otaMessage.getMessages(),
                                       PacketStatusPublisher::InboundAction::DROP_LOCK);
    }

  process();
}

void
EMANE::Models::BentPipe::ReceiveManager::process()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "MACI %03hu BentPipe::ReceiveManager::%s",
                          id_,
                          __func__);

  auto now = Clock::now();

  // check to make sure we work off anything in the pending queue that
  // should be worked off
  while(!pendingQueue_.empty())
    {
      // first item in the queue is the frame we are currently locked
      // on, earliest SoR
      TimePoint pendingStartOfReception{pendingQueue_.begin()->first};
      TimePoint pendingEndOfReception{pendingStartOfReception + std::get<5>(pendingQueue_.begin()->second)};

      // are we able to process this SoR, does it start after the end of the
      // last frame we processed
      if(pendingStartOfReception >= lastEndOfReception_)
        {
          if(now >= pendingEndOfReception)
            {
              // time to processing this pending frame
              FrameInfo pendingFrameInfo = std::move(pendingQueue_.begin()->second);

              // remove from queue after moving contents
              pendingQueue_.erase(pendingQueue_.begin());

              BentPipeMessage & bentPipeMessage{std::get<0>(pendingFrameInfo)};
              PacketInfo & pktInfo{std::get<1>(pendingFrameInfo)};
              size_t length{std::get<2>(pendingFrameInfo)};
              FrequencySegments & frequencySegments = std::get<4>(pendingFrameInfo);
              Microseconds & span = std::get<5>(pendingFrameInfo);

              auto & frequencySegment = *frequencySegments.begin();

              double dSINR{};
              double dNoiseFloordB{};

              try
                {
                  auto window =
                    pRadioService_->spectrumService().requestAntenna(rxAntennaIndex_,
                                                                     frequencySegment.getFrequencyHz(),
                                                                     span,
                                                                     pendingStartOfReception);

                  bool bSignalInNoise{};

                  std::tie(dNoiseFloordB,bSignalInNoise) =
                    Utils::maxBinNoiseFloor(window,frequencySegment.getRxPowerdBm());

                  dSINR = frequencySegment.getRxPowerdBm() - dNoiseFloordB;

                  LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                         DEBUG_LEVEL,
                                         "MACI %03hu BentPipe::ReceiveManager upstream EOR processing:"
                                         " src %hu, dst %hu, max noise %lf, signal in noise %s, SINR %lf",
                                         id_,
                                         pktInfo.getSource(),
                                         pktInfo.getDestination(),
                                         dNoiseFloordB,
                                         bSignalInNoise ? "yes" : "no",
                                         dSINR);
                }
              catch(SpectrumServiceException & exp)
                {
                  pPacketStatusPublisher_->inbound(pktInfo.getSource(),
                                                   bentPipeMessage.getMessages(),
                                                   PacketStatusPublisher::InboundAction::DROP_SPECTRUM_SERVICE);


                  LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                         ERROR_LEVEL,
                                         "MACI %03hu BentPipe::ReceiveManager upstream EOR processing: src %hu,"
                                         " dst %hu, now %ju sor %ju, span %ju queue %ju spectrum service request error: %s",
                                         id_,
                                         pktInfo.getSource(),
                                         pktInfo.getDestination(),
                                         std::chrono::duration_cast<Microseconds>(now.time_since_epoch()).count(),
                                         std::chrono::duration_cast<Microseconds>(pendingStartOfReception.time_since_epoch()).count(),
                                         span.count(),
                                         pendingQueue_.size(),
                                         exp.what());

                  continue;
                }


              // check sinr
              if(auto fPOR = pPCRManager_->getPOR(bentPipeMessage.getPCRCurveIndex(),dSINR,length))
                {
                  LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                         DEBUG_LEVEL,
                                         "MACI %03hu BentPipe::ReceiveManager upstream EOR processing: src %hu,"
                                         " dst %hu, curve index: %hu sinr: %lf length: %lu, por: %f",
                                         id_,
                                         pktInfo.getSource(),
                                         pktInfo.getDestination(),
                                         bentPipeMessage.getPCRCurveIndex(),
                                         dSINR,
                                         length,
                                         *fPOR);

                  // get random value [0.0, 1.0]
                  float fRandom{distribution_()};

                  if(fPOR < fRandom)
                    {
                      pPacketStatusPublisher_->inbound(pktInfo.getSource(),
                                                       bentPipeMessage.getMessages(),
                                                       PacketStatusPublisher::InboundAction::DROP_SINR);

                      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                             DEBUG_LEVEL,
                                             "MACI %03hu BentPipe::ReceiveManager upstream EOR processing: src %hu, dst %hu, "
                                             "rxpwr %3.2f dBm, drop",
                                             id_,
                                             pktInfo.getSource(),
                                             pktInfo.getDestination(),
                                             frequencySegment.getRxPowerdBm());

                      continue;
                    }
                }
              else
                {
                  pPacketStatusPublisher_->inbound(pktInfo.getSource(),
                                                   bentPipeMessage.getMessages(),
                                                   PacketStatusPublisher::InboundAction::DROP_BAD_CURVE);

                  LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                         ERROR_LEVEL,
                                         "MACI %03hu BentPipe::ReceiveManager upstream EOR processing error: src %hu, dst %hu, "
                                         " using unknown pcr curve index: %hu, drop",
                                         id_,
                                         pktInfo.getSource(),
                                         pktInfo.getDestination(),
                                         bentPipeMessage.getPCRCurveIndex());

                  continue;
                }

              // update neighbor metrics
              pNeighborStatusPublisher_->update(pktInfo.getSource(),
                                                transponderIndex_,
                                                dSINR,
                                                dNoiseFloordB,
                                                pendingStartOfReception);

              for(const auto & message : bentPipeMessage.getMessages())
                {
                  NEMId dst{message.getDestination()};

                  if((bProcess_ && ((dst == id_) || (dst == NEM_BROADCAST_MAC_ADDRESS))) ||
                     (!bProcess_ && (dst != id_)))
                    {
                      const auto & data = message.getData();

                      if(message.isFragment())
                        {
                          LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                                 DEBUG_LEVEL,
                                                 "MACI %03hu BentPipe::ReceiveManager upstream EOR processing:"
                                                 " src %hu, dst %hu, findex: %zu foffset: %zu fbytes: %zu"
                                                 " fmore: %s",
                                                 id_,
                                                 pktInfo.getSource(),
                                                 pktInfo.getDestination(),
                                                 message.getFragmentIndex(),
                                                 message.getFragmentOffset(),
                                                 data.size(),
                                                 message.isMoreFragments() ? "yes" : "no");

                          auto key = std::make_tuple(pktInfo.getSource(),
                                                     message.getFragmentSequence());

                          auto iter = fragmentStore_.find(key);

                          if(iter != fragmentStore_.end())
                            {
                              auto & indexSet = std::get<0>(iter->second);
                              auto & parts = std::get<1>(iter->second);
                              auto & lastFragmentTime = std::get<2>(iter->second);
                              auto & totalNumFragments = std::get<4>(iter->second);

                              if(indexSet.insert(message.getFragmentIndex()).second)
                                {
                                  parts.insert(std::make_pair(message.getFragmentOffset(),message.getData()));

                                  lastFragmentTime = now;

                                  // this is a new fragment. If the
                                  // more fragments bit is not set
                                  // then this is the last fragment
                                  // piece so set the
                                  // totalNumFragments appropriately.
                                  if(!message.isMoreFragments())
                                    {
                                      totalNumFragments = message.getFragmentIndex() + 1;
                                    }

                                  // check to see if all fragments have been received
                                  if(totalNumFragments && indexSet.size() == totalNumFragments)
                                    {
                                      Utils::VectorIO vectorIO{};

                                      for(const auto & part : parts)
                                        {
                                          vectorIO.push_back(Utils::make_iovec(const_cast<std::uint8_t *>(part.second.data()),
                                                                               part.second.size()));
                                        }

                                      if(bProcess_)
                                        {
                                          UpstreamPacket pkt{{pktInfo.getSource(),
                                                                dst,
                                                                pktInfo.getPriority(),
                                                                pktInfo.getCreationTime(),
                                                                pktInfo.getUUID()},vectorIO};

                                          pPacketStatusPublisher_->inbound(pktInfo.getSource(),
                                                                           dst,
                                                                           pkt.length(),
                                                                           PacketStatusPublisher::InboundAction::ACCEPT_GOOD);

                                          LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                                                 DEBUG_LEVEL,
                                                                 "MACI %03hu BentPipe::ReceiveManager upstream EOR processing:"
                                                                 " src %hu, dst %hu, forward upstream",
                                                                 id_,
                                                                 pktInfo.getSource(),
                                                                 pktInfo.getDestination());

                                          pTransponderPacketTransport_->processPacket(pkt);
                                        }
                                      else
                                        {
                                          LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                                                 DEBUG_LEVEL,
                                                                 "MACI %03hu BentPipe::ReceiveManager upstream EOR processing:"
                                                                 " src %hu, dst %hu, bend downstream",
                                                                 id_,
                                                                 pktInfo.getSource(),
                                                                 pktInfo.getDestination());

                                          DownstreamPacket pkt{{id_,
                                                                  pktInfo.getDestination(),
                                                                  pktInfo.getPriority(),
                                                                  pktInfo.getCreationTime(),
                                                                  pktInfo.getUUID()},
                                                               nullptr,0};

                                          for(auto iter = vectorIO.rbegin();
                                              iter != vectorIO.rend();
                                              ++iter)
                                            {
                                              pkt.prepend(iter->iov_base,iter->iov_len);
                                            }

                                          // ubend
                                          pTransponderPacketTransport_->ubendPacket(pkt,
                                                                                    transponderIndex_);

                                        }

                                      fragmentStore_.erase(iter);
                                    }
                                }
                            }
                          else
                            {
                              // this is the first fragment for this
                              // message. Just need to set the total number of
                              // fragments if this happens to also be the last
                              // fragment in the set.
                              fragmentStore_.insert(std::make_pair(key,
                                                                   std::make_tuple(std::set<size_t>{message.getFragmentIndex()},
                                                                                   FragmentParts{{message.getFragmentOffset(),
                                                                                                    message.getData()}},
                                                                                   now,
                                                                                   dst,
                                                                                   message.isMoreFragments() ? 0 : message.getFragmentIndex() + 1)));
                            }
                        }
                      else
                        {
                          if(bProcess_)
                            {
                              LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                                     DEBUG_LEVEL,
                                                     "MACI %03hu BentPipe::ReceiveManager upstream EOR processing:"
                                                     " src %hu, dst %hu, forward upstream",
                                                     id_,
                                                     pktInfo.getSource(),
                                                     pktInfo.getDestination());


                              auto data = message.getData();

                              UpstreamPacket pkt{{pktInfo.getSource(),
                                                    dst,
                                                    pktInfo.getPriority(),
                                                    pktInfo.getCreationTime(),
                                                    pktInfo.getUUID()},
                                                 &data[0],
                                                 data.size()};


                              pPacketStatusPublisher_->inbound(pktInfo.getSource(),
                                                               message,
                                                               PacketStatusPublisher::InboundAction::ACCEPT_GOOD);

                              pTransponderPacketTransport_->processPacket(pkt);
                            }
                          else
                            {
                              LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                                     DEBUG_LEVEL,
                                                     "MACI %03hu BentPipe::ReceiveManager upstream EOR processing:"
                                                     " src %hu, dst %hu, bend downstream",
                                                     id_,
                                                     pktInfo.getSource(),
                                                     pktInfo.getDestination());

                              // ubend
                              DownstreamPacket pkt{{id_,
                                                      pktInfo.getDestination(),
                                                      pktInfo.getPriority(),
                                                      pktInfo.getCreationTime(),
                                                      pktInfo.getUUID()},
                                                   &data[0],
                                                   data.size()};

                              pTransponderPacketTransport_->ubendPacket(pkt,
                                                                        transponderIndex_);
                            }

                          // update the last EoR
                          lastEndOfReception_ = pendingEndOfReception;
                        }
                    }
                  else
                    {
                      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                             DEBUG_LEVEL,
                                             "MACI %03hu BentPipe::ReceiveManager::%s mode:%s dst:%hu, dropping",
                                             id_,
                                             __func__,
                                             bProcess_ ? "process" : "ubend",
                                             pktInfo.getDestination());

                      pPacketStatusPublisher_->inbound(pktInfo.getSource(),
                                                       message,
                                                       PacketStatusPublisher::InboundAction::DROP_DESTINATION_MAC);
                    }
                }
            }
          else
            {
              break;
            }
        }
      else
        {
          FrameInfo pendingFrameInfo = std::move(pendingQueue_.begin()->second);
          BentPipeMessage & bentPipeMessage{std::get<0>(pendingFrameInfo)};
          PacketInfo & pktInfo{std::get<1>(pendingFrameInfo)};

          pPacketStatusPublisher_->inbound(pktInfo.getSource(),
                                           bentPipeMessage.getMessages(),
                                           PacketStatusPublisher::InboundAction::DROP_LOCK);

          // drop this frame, it has an SoR that occurs during our
          // previous locked frame transmission
          pendingQueue_.erase(pendingQueue_.begin());
        }
    }

  if(!pendingQueue_.empty())
    {
      TimePoint pendingStartOfReception{pendingQueue_.begin()->first};
      TimePoint pendingEndOfReception{pendingStartOfReception + std::get<5>(pendingQueue_.begin()->second)};

      // if(pendingEndOfReception < nextEoRCheckTime_)
      //   {
      pPlatformService_->timerService().
        schedule(std::bind(&ReceiveManager::process,this),
                 pendingEndOfReception);

      nextEoRCheckTime_ = pendingEndOfReception;
      //}
    }


  // check to see if there are fragment assemblies to abandon
  if(lastFragmentCheckTime_ + fragmentCheckThreshold_ <= now)
    {
      for(auto iter = fragmentStore_.begin(); iter != fragmentStore_.end();)
        {
          auto & parts = std::get<1>(iter->second);
          auto & lastFragmentTime  = std::get<2>(iter->second);
          auto & dst  = std::get<3>(iter->second);

          if(lastFragmentTime + fragmentTimeoutThreshold_ <= now)
            {
              size_t totalBytes{};

              for(const auto & part : parts)
                {
                  totalBytes += part.second.size();
                }

              pPacketStatusPublisher_->inbound(std::get<0>(iter->first),
                                               dst,
                                               totalBytes,
                                               PacketStatusPublisher::InboundAction::DROP_MISS_FRAGMENT);

              fragmentStore_.erase(iter++);
            }
          else
            {
              ++iter;
            }
        }

      lastFragmentCheckTime_ = now;
    }
}
