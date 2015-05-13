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

#include "receivemanager.h"
#include "emane/utils/spectrumwindowutils.h"

EMANE::Models::TDMA::ReceiveManager::ReceiveManager(NEMId id,
                                                    DownstreamTransport * pDownstreamTransport,
                                                    LogServiceProvider * pLogService,
                                                    RadioServiceProvider * pRadioService,
                                                    Scheduler * pScheduler):
  id_{id},
  pDownstreamTransport_{pDownstreamTransport},
  pLogService_{pLogService},
  pRadioService_{pRadioService},
  pScheduler_{pScheduler},
  pendingInfo_{{{0,0,0,{}},nullptr,0},{},{},{},{},{}},
  u64PendingAbsoluteSlotIndex_{},
  distribution_{0.0, 1.0},
  bPromiscuousMode_{}{}

void EMANE::Models::TDMA::ReceiveManager::setPromiscuousMode(bool bEnable)
{
  bPromiscuousMode_ = bEnable;
}

void EMANE::Models::TDMA::ReceiveManager::loadCurves(const std::string & sPCRFileName)
{
  porManager_.load(sPCRFileName);
}
  
bool
EMANE::Models::TDMA::ReceiveManager::enqueue(UpstreamPacket && pkt,
                                             std::uint64_t u64AbsoluteSlotIndex,
                                             const BaseModelHeader & baseModelHeader,
                                             const TimePoint & startOfReception,
                                             const FrequencySegments & frequencySegments,
                                             const Microseconds & span,
                                             const TimePoint & beginTime)
{
  bool bReturn{};

  if(!u64PendingAbsoluteSlotIndex_) 
    {
      u64PendingAbsoluteSlotIndex_ = u64AbsoluteSlotIndex;
          
      pendingInfo_ = std::make_tuple(std::move(pkt),
                                     baseModelHeader,
                                     startOfReception,
                                     frequencySegments,
                                     span,
                                     beginTime);
      bReturn = true;
    }
  else if(u64PendingAbsoluteSlotIndex_ < u64AbsoluteSlotIndex)
    {
      process(u64AbsoluteSlotIndex);
      
      u64PendingAbsoluteSlotIndex_ = u64AbsoluteSlotIndex;
      
      pendingInfo_ = std::make_tuple(std::move(pkt),
                                     baseModelHeader,
                                     startOfReception,
                                     frequencySegments,
                                     span,
                                     beginTime);
      bReturn = true;
    }
  else if(u64PendingAbsoluteSlotIndex_ > u64AbsoluteSlotIndex)
    {
      LOGGER_VERBOSE_LOGGING(*pLogService_,
                             ERROR_LEVEL,
                             "MACI %03hu TDMA::ReceiveManager enqueue: pending slot: %zu greater than enqueue: %zu",
                             id_,
                             u64PendingAbsoluteSlotIndex_,
                             u64AbsoluteSlotIndex);

      u64PendingAbsoluteSlotIndex_ = u64AbsoluteSlotIndex;
          
      pendingInfo_ = std::make_tuple(std::move(pkt),
                                     baseModelHeader,
                                     startOfReception,
                                     frequencySegments,
                                     span,
                                     beginTime);
      bReturn = true;
      
    }
  else
    {
      if(std::get<2>(pendingInfo_)  < startOfReception)
        {
          pendingInfo_ = std::make_tuple(std::move(pkt),
                                         baseModelHeader,
                                         startOfReception,
                                         frequencySegments,
                                         span,
                                         beginTime);
        }
    }
  
  return bReturn;
}

void
EMANE::Models::TDMA::ReceiveManager::process(std::uint64_t u64AbsoluteSlotIndex)
{
  if(u64PendingAbsoluteSlotIndex_ + 1 == u64AbsoluteSlotIndex)
    {
      u64PendingAbsoluteSlotIndex_ = 0;

      double dSINR{};
      double dNoiseFloordB{};

      UpstreamPacket & pkt{std::get<0>(pendingInfo_)};
      BaseModelHeader & baseModelHeader{std::get<1>(pendingInfo_)};
      TimePoint & startOfReception{std::get<2>(pendingInfo_)};
      FrequencySegments & frequencySegments{std::get<3>(pendingInfo_)};
      Microseconds & span{std::get<4>(pendingInfo_)};

      auto & pktInfo = pkt.getPacketInfo();
      auto & frequencySegment = *frequencySegments.begin();
      
      try
        {
          auto window = pRadioService_->spectrumService().request(frequencySegment.getFrequencyHz(),
                                                                  span,
                                                                  startOfReception);

          
          bool bSignalInNoise{};

          std::tie(dNoiseFloordB,bSignalInNoise) =
            Utils::maxBinNoiseFloor(window,frequencySegment.getRxPowerdBm());
                  
          dSINR = frequencySegment.getRxPowerdBm() - dNoiseFloordB;
          
          LOGGER_VERBOSE_LOGGING(*pLogService_,
                                 DEBUG_LEVEL,
                                 "MACI %03hu TDMA::ReceiveManager upstream EOR processing: src %hu, dst %hu, max noise %f, signal in noise %s, SINR %f",
                                 id_,
                                 pktInfo.getSource(),
                                 pktInfo.getDestination(),
                                 dNoiseFloordB,
                                 bSignalInNoise ? "yes" : "no",
                                 dSINR);
        }
      catch(SpectrumServiceException & exp)
        {
          LOGGER_VERBOSE_LOGGING(*pLogService_,
                                 ERROR_LEVEL,
                                 "MACI %03hu TDMA::ReceiveManager upstream EOR processing: src %hu, dst %hu, sor %ju, span %ju spectrum service request error: %s",
                                 id_,
                                 pktInfo.getSource(),
                                 pktInfo.getDestination(),
                                 std::chrono::duration_cast<Microseconds>(startOfReception.time_since_epoch()).count(),
                                 span.count(),
                                 exp.what());
          
          return;
        }


      // check sinr
      float fPOR = porManager_.getPOR(baseModelHeader.getDataRate(),dSINR, pkt.length());

      // get random value [0.0, 1.0]
      float fRandom{distribution_()};

      if(fPOR < fRandom)
        {
          LOGGER_VERBOSE_LOGGING(*pLogService_,
                                 DEBUG_LEVEL,
                                 "MACI %03hu TDMA::ReceiveManager upstream EOR processing: src %hu, dst %hu, "
                                 "rxpwr %3.2f dBm, drop",
                                 id_,
                                 pktInfo.getSource(),
                                 pktInfo.getDestination(),
                                 frequencySegment.getRxPowerdBm());

          return;
        }

     
      if(bPromiscuousMode_ ||
         (pktInfo.getDestination() == id_) ||
         (pktInfo.getDestination() == NEM_BROADCAST_MAC_ADDRESS))
        {
          LOGGER_VERBOSE_LOGGING(*pLogService_,
                                 DEBUG_LEVEL,
                                 "MACI %03hu TDMA::ReceiveManager upstream EOR processing: src %hu, dst %hu, forward upstream",
                                 id_,
                                 pktInfo.getSource(),
                                 pktInfo.getDestination());
          
          
          pDownstreamTransport_->sendUpstreamPacket(pkt);
        }
      
    }
}
