/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "emane/utils/commonlayerstatistics.h"
#include "emane/statisticnumeric.h"
#include "emane/utils/runningaverage.h"
#include "emane/statistictable.h"

#include <vector>
#include <map>

class EMANE::Utils::CommonLayerStatistics::Implementation
{
public:
  Implementation(const StatisticTableLabels & unicastDropTableLabels,
                 const StatisticTableLabels & broadcastDropTableLabels,
                 const std::string & sInstance) :
    sInstance_{sInstance},
    bHaveDropTables_{},
    unicastDropTableLabels_{"NEM"},
    broadcastDropTableLabels_{"NEM"},
    unicastAcceptTableLabels_{"NEM", "Num Pkts Tx", "Num Bytes Tx", "Num Pkts Rx", "Num Bytes Rx"},
    broadcastAcceptTableLabels_{"NEM", "Num Pkts Tx", "Num Bytes Tx", "Num Pkts Rx", "Num Bytes Rx"},
    pNumUpstreamPacketsTx_{},
    pNumUpstreamBytesTx_{},
    pNumUpstreamPacketsUnicastTx_{},
    pNumUpstreamBytesUnicastTx_{},
    pNumUpstreamPacketsBroadcastTx_{},
    pNumUpstreamBytesBroadcastTx_{},
    pNumUpstreamPacketsRx_{},
    pNumUpstreamBytesRx_{},
    pNumUpstreamPacketsDrop_{},
    pNumUpstreamPacketsUnicastRx_{},
    pNumUpstreamBytesUnicastRx_{},
    pNumUpstreamPacketsUnicastDrop_{},
    pNumUpstreamPacketsBroadcastRx_{},
    pNumUpstreamBytesBroadcastRx_{},
    pNumUpstreamPacketsBroadcastDrop_{},
    pNumUpstreamProcessingDelay_{},
    pNumDownstreamPacketsTx_{},
    pNumDownstreamBytesTx_{},
    pNumDownstreamPacketsUnicastTx_{},
    pNumDownstreamBytesUnicastTx_{},
    pNumDownstreamPacketsBroadcastTx_{},
    pNumDownstreamBytesBroadcastTx_{},
    pNumDownstreamPacketsRx_{},
    pNumDownstreamBytesRx_{},
    pNumDownstreamPacketsDrop_{},
    pNumDownstreamPacketsUnicastRx_{},
    pNumDownstreamBytesUnicastRx_{},
    pNumDownstreamPacketsUnicastDrop_{},
    pNumDownstreamPacketsBroadcastRx_{},
    pNumDownstreamBytesBroadcastRx_{},
    pNumDownstreamPacketsBroadcastDrop_{},
    pNumDownstreamProcessingDelay_{},
    pNumDownstreamPacketsGenerated_{},
    pNumDownstreamBytesGenerated_{},
    pNumDownstreamPacketsUnicastGenerated_{},
    pNumDownstreamBytesUnicastGenerated_{},
    pNumDownstreamPacketsBroadcastGenerated_{},
    pNumDownstreamBytesBroadcastGenerated_{},
    pStatisticUnicastDropTable_{},
    pStatisticBroadcastDropTable_{},
    pStatisticUnicastAcceptTable_{},
    pStatisticBroadcastAcceptTable_{}
  {
    if(unicastDropTableLabels.empty())
      {
        bHaveDropTables_ = false;
      }
    else
      {
        bHaveDropTables_ = true;

        unicastDropTableLabels_.insert(unicastDropTableLabels_.end(),
                                       unicastDropTableLabels.begin(),
                                       unicastDropTableLabels.end());
    
        if(broadcastDropTableLabels.empty())
          {
            broadcastDropTableLabels_ = unicastDropTableLabels_;
          }
        else
          {
            broadcastDropTableLabels_.insert(broadcastDropTableLabels_.end(),
                                             broadcastDropTableLabels.begin(),
                                             broadcastDropTableLabels.end());
          }
       }
  }

  ~Implementation(){ }

  void registerStatistics(StatisticRegistrar & statisticRegistrar)
  {
    // upstream tx all
    pNumUpstreamPacketsTx_ =
      statisticRegistrar.registerNumeric<Counter>("numUpstreamPacketsTx" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Number of upstream packets transmitted");

    pNumUpstreamBytesTx_ =
      statisticRegistrar.registerNumeric<Counter>("numUpstreamBytesTx" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Number of upstream bytes transmitted");

    avgUpstreamPacketSizeTx_.registerStatistic(
      statisticRegistrar.registerNumeric<Average>("avgUpstreamPacketSizeTx" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Average upstream packet size"));

    // upstream processing delay
    pNumUpstreamProcessingDelay_ =
      statisticRegistrar.registerNumeric<Counter>("numUpstreamProcessingDelay" + sInstance_,
                                                  StatisticProperties::CLEARABLE);

    avgUpstreamProcessingDelay_.registerStatistic(
      statisticRegistrar.registerNumeric<Average>("avgUpstreamProcessingDelay" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Average upstream processing delay"));


    // upstream tx ucast
    pNumUpstreamPacketsUnicastTx_ =
      statisticRegistrar.registerNumeric<Counter>("numUpstreamPacketsUnicastTx" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Number of upstream unicast packets transmitted");

    pNumUpstreamBytesUnicastTx_ =
      statisticRegistrar.registerNumeric<Counter>("numUpstreamBytesUnicastTx" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Number of upstream unicast bytes transmitted");

    avgUpstreamPacketSizeUnicastTx_.registerStatistic(
      statisticRegistrar.registerNumeric<Average>("avgUpstreamPacketSizeUnicastTx" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Average upstream unicast packet size transmitted"));


    // upstream tx bcast
    pNumUpstreamPacketsBroadcastTx_ =
      statisticRegistrar.registerNumeric<Counter>("numUpstreamPacketsBroadcastTx" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Number of upstream broadcast packets transmitted");

    pNumUpstreamBytesBroadcastTx_ =
      statisticRegistrar.registerNumeric<Counter>("numUpstreamBytesBroadcastTx" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Number of updtream broadcast bytes transmitted");

    avgUpstreamPacketSizeBroadcastTx_.registerStatistic(
      statisticRegistrar.registerNumeric<Average>("avgUpstreamPacketSizeBroadcastTx" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Average upstream broadcast packet size transmitted"));


    // upstream rx all
    pNumUpstreamPacketsRx_ =
      statisticRegistrar.registerNumeric<Counter>("numUpstreamPacketsRx" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Number of upstream packets received");

    pNumUpstreamBytesRx_ =
      statisticRegistrar.registerNumeric<Counter>("numUpstreamBytesRx" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Number of upstream bytes received");

    pNumUpstreamPacketsDrop_ =
      statisticRegistrar.registerNumeric<Counter>("numUpstreamPacketsDrop" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Number upstream packets droppped");

    avgUpstreamPacketSizeRx_.registerStatistic(
      statisticRegistrar.registerNumeric<Average>("avgUpstreamPacketSizeRx" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Average upstream packet size received"));

    // upstream rx ucast
    pNumUpstreamPacketsUnicastRx_ =
      statisticRegistrar.registerNumeric<Counter>("numUpstreamPacketsUnicastRx" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Number upstream unicast packets received");

    pNumUpstreamBytesUnicastRx_ =
      statisticRegistrar.registerNumeric<Counter>("numUpstreamBytesUnicastRx" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Number upstream unicast bytes received");

    pNumUpstreamPacketsUnicastDrop_ =
      statisticRegistrar.registerNumeric<Counter>("numUpstreamPacketsUnicastDrop" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Number of upstream unicast packets dropped");

    avgUpstreamPacketSizeUnicastRx_.registerStatistic(
      statisticRegistrar.registerNumeric<Average>("avgUpstreamPacketSizeUnicastRx" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Average upstream unicast packet size received"));


    // upstream rx bcast
    pNumUpstreamPacketsBroadcastRx_ =
      statisticRegistrar.registerNumeric<Counter>("numUpstreamPacketsBroadcastRx" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Number of upstream broadcast packets received");

    pNumUpstreamBytesBroadcastRx_ =
      statisticRegistrar.registerNumeric<Counter>("numUpstreamBytesBroadcastRx" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Number of upstream broadcast bytes received");

    pNumUpstreamPacketsBroadcastDrop_ =
      statisticRegistrar.registerNumeric<Counter>("numUpstreamPacketsBroadcastDrop" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Number of upstream broadcast packets dropped");

    avgUpstreamPacketSizeBroadcastRx_.registerStatistic(
      statisticRegistrar.registerNumeric<Average>("avgUpstreamPacketSizeBroadcastRx" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Average upstream broadcast packet size received"));


    // downstream tx all
    pNumDownstreamPacketsTx_ =
      statisticRegistrar.registerNumeric<Counter>("numDownstreamPacketsTx" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Number of downstream packets transmitted");

    pNumDownstreamBytesTx_ =
      statisticRegistrar.registerNumeric<Counter>("numDownstreamBytesTx" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Number of downstream bytes transmitted");

    avgDownstreamPacketSizeTx_.registerStatistic(
      statisticRegistrar.registerNumeric<Average>("avgDownstreamPacketSizeTx" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Average downstream packet size transmitted"));

    // downstream tx ucast
    pNumDownstreamPacketsUnicastTx_ =
      statisticRegistrar.registerNumeric<Counter>("numDownstreamPacketsUnicastTx" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Number of downstream unicast packets transmitted");

    pNumDownstreamBytesUnicastTx_ =
      statisticRegistrar.registerNumeric<Counter>("numDownstreamBytesUnicastTx" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Number of downstream unicast bytes transmitted");

    avgDownstreamPacketSizeUnicastTx_.registerStatistic(
      statisticRegistrar.registerNumeric<Average>("avgDownstreamPacketSizeUnicastTx" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Average downstream unicast packet size transmitted"));

    // downstream tx bcast
    pNumDownstreamPacketsBroadcastTx_ =
      statisticRegistrar.registerNumeric<Counter>("numDownstreamPacketsBroadcastTx" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Number of downstream broadcast packets transmitted");

    pNumDownstreamBytesBroadcastTx_ =
      statisticRegistrar.registerNumeric<Counter>("numDownstreamBytesBroadcastTx" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Number of downstream broadcast bytes transmitted");

    avgDownstreamPacketSizeBroadcastTx_.registerStatistic(
      statisticRegistrar.registerNumeric<Average>("avgDownstreamPacketSizeBroadcastTx" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Average downstream broadcast packet size transmitted"));


    // downstream rx all
    pNumDownstreamPacketsRx_ =
      statisticRegistrar.registerNumeric<Counter>("numDownstreamPacketsRx" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Number of downstream packets received");

    pNumDownstreamBytesRx_ =
      statisticRegistrar.registerNumeric<Counter>("numDownstreamBytesRx" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Number of downstream bytes received");

    pNumDownstreamPacketsDrop_ =
      statisticRegistrar.registerNumeric<Counter>("numDownstreamPacketsDrop" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "number of downstream packets dropped");

    avgDownstreamPacketSizeRx_.registerStatistic(
      statisticRegistrar.registerNumeric<Average>("avgDownstreamPacketSizeRx" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Average downstream packet size received"));

    // downstream rx ucast
    pNumDownstreamPacketsUnicastRx_ =
      statisticRegistrar.registerNumeric<Counter>("numDownstreamPacketsUnicastRx" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Number of downstream unicast packets received");

    pNumDownstreamBytesUnicastRx_ =
      statisticRegistrar.registerNumeric<Counter>("numDownstreamBytesUnicastRx" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Number of downstream unicast bytes received");

    pNumDownstreamPacketsUnicastDrop_ =
      statisticRegistrar.registerNumeric<Counter>("numDownstreamPacketsUnicastDrop" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Number of downstream unicast packets dropped");

    avgDownstreamPacketSizeUnicastRx_.registerStatistic(
      statisticRegistrar.registerNumeric<Average>("avgDownstreamPacketSizeUnicastRx" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Average downstream unicast packet size received"));

    // downstream rx bcast
    pNumDownstreamPacketsBroadcastRx_ =
      statisticRegistrar.registerNumeric<Counter>("numDownstreamPacketsBroadcastRx" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Number of downstream broadcast packets received");

    pNumDownstreamBytesBroadcastRx_ =
      statisticRegistrar.registerNumeric<Counter>("numDownstreamBytesBroadcastRx" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Number of downstream broadcast bytes received");

    pNumDownstreamPacketsBroadcastDrop_ =
      statisticRegistrar.registerNumeric<Counter>("numDownstreamPacketsBroadcastDrop" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Number of downstream broadcast packets dropped");

    avgDownstreamPacketSizeBroadcastRx_.registerStatistic(
      statisticRegistrar.registerNumeric<Average>("avgDownstreamPacketSizeBroadcastRx" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Average downstream broadcast packet size received"));

    // downstream processing delay
    pNumDownstreamProcessingDelay_ =
      statisticRegistrar.registerNumeric<Counter>("numDownstreamProcessingDelay" + sInstance_,
                                                  StatisticProperties::CLEARABLE);

    avgDownstreamProcessingDelay_.registerStatistic(
      statisticRegistrar.registerNumeric<Average>("avgDownstreamProcessingDelay" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Average downstream processing delay"));



    // downstream generated all
    pNumDownstreamPacketsGenerated_ =
      statisticRegistrar.registerNumeric<Counter>("numDownstreamPacketsGenerated" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Number of layer generated downstream packets");

    pNumDownstreamBytesGenerated_ =
      statisticRegistrar.registerNumeric<Counter>("numDownstreamBytesGenerated" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Number of layer generated downstream bytes");

    avgDownstreamPacketSizeGenerated_.registerStatistic(
      statisticRegistrar.registerNumeric<Average>("avgDownstreamPacketSizeGenerated" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Average layer generated downstream packet size"));

    // downstream generated ucast
    pNumDownstreamPacketsUnicastGenerated_ =
      statisticRegistrar.registerNumeric<Counter>("numDownstreamPacketsUnicastGenerated" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Number of layer generated downstream unicast packets");

    pNumDownstreamBytesUnicastGenerated_ =
      statisticRegistrar.registerNumeric<Counter>("numDownstreamBytesUnicastGenerated" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Number of layer generated downstream unicast bytes");

    avgDownstreamPacketSizeUnicastGenerated_.registerStatistic(
      statisticRegistrar.registerNumeric<Average>("avgDownstreamPacketSizeUnicastGenerated" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Average layer generated downstream unicast packet size"));

    // downstream generated bcast
    pNumDownstreamPacketsBroadcastGenerated_ =
      statisticRegistrar.registerNumeric<Counter>("numDownstreamPacketsBroadcastGenerated" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Number of layer generated downstream broadcast packets");

    pNumDownstreamBytesBroadcastGenerated_ =
      statisticRegistrar.registerNumeric<Counter>("numDownstreamBytesBroadcastGenerated" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Number of layer generated downstream broadcast bytes");

    avgDownstreamPacketSizeBroadcastGenerated_.registerStatistic(
      statisticRegistrar.registerNumeric<Average>("avgDownstreamPacketSizeBroadcastGenerated" + sInstance_,
                                                  StatisticProperties::CLEARABLE,
                                                  "Average layer generated downstream broadcast packet size"));

    if(bHaveDropTables_)
      {
        pStatisticUnicastDropTable_ =
          statisticRegistrar.registerTable<NEMId>("UnicastPacketDropTable" + sInstance_, 
                                                  unicastDropTableLabels_,
                                                  [this](StatisticTablePublisher * pTable)
                                                  {
                                                    std::lock_guard<std::mutex> m(nemUnicastDropTableMutex_);
                                                    nemUnicastDropCodeMap_.clear();
                                                    pTable->clear();
                                                  },
                                                  "Unicast packets dropped by reason code");

        pStatisticBroadcastDropTable_ =
          statisticRegistrar.registerTable<NEMId>("BroadcastPacketDropTable" + sInstance_, 
                                                  broadcastDropTableLabels_,
                                                  [this](StatisticTablePublisher * pTable)
                                                  {
                                                    std::lock_guard<std::mutex> m(nemBroadcastDropTableMutex_);
                                                    nemBroadcastDropCodeMap_.clear();
                                                    pTable->clear();
                                                  },
                                                  "Broadcast packets dropped by reason code");
      }

     pStatisticUnicastAcceptTable_ =
       statisticRegistrar.registerTable<NEMId>("UnicastPacketAcceptTable" + sInstance_, 
                                               unicastAcceptTableLabels_,
                                               [this](StatisticTablePublisher * pTable)
                                               {
                                                 std::lock_guard<std::mutex> m(nemUnicastAcceptTableMutex_);
                                                 nemUnicastAcceptMap_.clear();
                                                 pTable->clear();
                                               },
                                               "Unicast packets accepted");
     
     pStatisticBroadcastAcceptTable_ =
       statisticRegistrar.registerTable<NEMId>("BroadcastPacketAcceptTable" + sInstance_, 
                                               broadcastAcceptTableLabels_,
                                               [this](StatisticTablePublisher * pTable)
                                               {
                                                 std::lock_guard<std::mutex> m(nemBroadcastAcceptTableMutex_);
                                                 nemBroadcastAcceptMap_.clear();
                                                 pTable->clear();
                                               },
                                               "Broadcast packets accepted");
  }


  void processInbound(const UpstreamPacket & pkt)
  {
    // update all rx stats
    ++*pNumUpstreamPacketsRx_;

    *pNumUpstreamBytesRx_ += pkt.length();

    avgUpstreamPacketSizeRx_.update(pkt.length());

    if(isBroadcast(pkt))
      {
        ++*pNumUpstreamPacketsBroadcastRx_;

        *pNumUpstreamBytesBroadcastRx_ += pkt.length();

        avgUpstreamPacketSizeBroadcastRx_.update(pkt.length());
      }
    else
      {
        ++*pNumUpstreamPacketsUnicastRx_;

        *pNumUpstreamBytesUnicastRx_ += pkt.length();

        avgUpstreamPacketSizeUnicastRx_.update(pkt.length());
      }
  }


  void processOutbound(const UpstreamPacket & pkt, const Microseconds & delay, size_t dropCode)
  {
    if(dropCode)
      {
        // ensure tables are built for all src
        updateDropTable(pkt.getPacketInfo(), dropCode, isBroadcast(pkt),true);
      }
    else
      {
        updateAcceptTable(pkt.getPacketInfo(), pkt.length(), isBroadcast(pkt), true, true);
      }

    if(dropCode != 0)
      {
        ++*pNumUpstreamPacketsDrop_;

        if(isBroadcast(pkt))
          {
            ++*pNumUpstreamPacketsBroadcastDrop_;
          }
        else
          {
            ++*pNumUpstreamPacketsUnicastDrop_;
          }
      }
    // update all tx stats
    else
      {
        ++*pNumUpstreamPacketsTx_;

        *pNumUpstreamBytesTx_ += pkt.length();

        avgUpstreamPacketSizeTx_.update(pkt.length());

        *pNumUpstreamProcessingDelay_ += delay.count();

        avgUpstreamProcessingDelay_.update(delay.count());

        if(isBroadcast(pkt))
          {
            ++*pNumUpstreamPacketsBroadcastTx_;

            *pNumUpstreamBytesBroadcastTx_ += pkt.length();

            avgUpstreamPacketSizeBroadcastTx_.update(pkt.length());
          }
        else
          {
            ++*pNumUpstreamPacketsUnicastTx_;

            *pNumUpstreamBytesUnicastTx_ += pkt.length();

            avgUpstreamPacketSizeUnicastTx_.update(pkt.length());
          }
      }
  }


  void processInbound(const DownstreamPacket & pkt)
  {
    // update all rx stats
    ++*pNumDownstreamPacketsRx_;

    *pNumDownstreamBytesRx_ += pkt.length();

    avgDownstreamPacketSizeRx_.update(pkt.length());

    if(isBroadcast(pkt))
      {
        ++*pNumDownstreamPacketsBroadcastRx_;

        *pNumDownstreamBytesBroadcastRx_ += pkt.length();

        avgDownstreamPacketSizeBroadcastRx_.update(pkt.length());
      }
    else
      {
        ++*pNumDownstreamPacketsUnicastRx_;

        *pNumDownstreamBytesUnicastRx_ += pkt.length();

        avgDownstreamPacketSizeUnicastRx_.update(pkt.length());
      }
  }


  void processOutbound(const DownstreamPacket & pkt, const Microseconds & delay, size_t dropCode, bool bSelfGenerated)
  {
    if(dropCode)
      {
        updateDropTable(pkt.getPacketInfo(), dropCode, isBroadcast(pkt),false);
      }
    else
      {
        updateAcceptTable(pkt.getPacketInfo(), pkt.length(), isBroadcast(pkt), false, true);
      }

    if(dropCode != 0)
      {
        ++*pNumDownstreamPacketsDrop_;

        if(isBroadcast(pkt))
          {
            ++*pNumDownstreamPacketsBroadcastDrop_;
          }
        else
          {
            ++*pNumDownstreamPacketsUnicastDrop_;
          }
      }
    // update all tx stats
    else
      {
        ++*pNumDownstreamPacketsTx_;

        *pNumDownstreamBytesTx_ += pkt.length();

        avgDownstreamPacketSizeTx_.update(pkt.length());

        *pNumDownstreamProcessingDelay_ += delay.count();

        avgDownstreamProcessingDelay_.update(delay.count());

        if(bSelfGenerated)
          {
            ++*pNumDownstreamPacketsGenerated_;

            *pNumDownstreamBytesGenerated_ += pkt.length();

            avgDownstreamPacketSizeGenerated_.update(pkt.length());

            if(isBroadcast(pkt))
              {
                ++*pNumDownstreamPacketsBroadcastGenerated_;

                pNumDownstreamBytesBroadcastGenerated_ += pkt.length();

                avgDownstreamPacketSizeBroadcastGenerated_.update(pkt.length());
              }
            else
              {
                ++*pNumDownstreamPacketsUnicastGenerated_;

                pNumDownstreamBytesUnicastGenerated_ += pkt.length();

                avgDownstreamPacketSizeUnicastGenerated_.update(pkt.length());
              }
          }
        else
          {
            if(isBroadcast(pkt))
              {
                ++*pNumDownstreamPacketsBroadcastTx_;

                *pNumDownstreamBytesBroadcastTx_ += pkt.length();

                avgDownstreamPacketSizeBroadcastTx_.update(pkt.length());
              }
            else
              {
                ++*pNumDownstreamPacketsUnicastTx_;

                *pNumDownstreamBytesUnicastTx_ += pkt.length();

                avgDownstreamPacketSizeUnicastTx_.update(pkt.length());
              }
          }
      }
  }


  void updateDropTable(const PacketInfo & packetInfo, size_t dropCode, bool bIsBroadcast, bool bIsUpstream)
  {
    if(bHaveDropTables_)
      {
        NEMDropCodeMap * pThisDropCodeMap{};

        StatisticTable<NEMId> * pThisTable{};

        size_t numColumns{};

        std::mutex * pThisMutex{};

        NEMId nemId{};

        if(bIsUpstream)
          {
            nemId = packetInfo.getSource();
          }
        else
          {
            nemId = packetInfo.getDestination();
          }

        if(bIsBroadcast)
          {
            pThisDropCodeMap = &nemBroadcastDropCodeMap_;

            pThisTable = pStatisticBroadcastDropTable_;

            pThisMutex = &nemBroadcastDropTableMutex_;

            numColumns = broadcastDropTableLabels_.size();
          }
        else
          {
            pThisDropCodeMap = &nemUnicastDropCodeMap_;

            pThisTable = pStatisticUnicastDropTable_;

            pThisMutex = &nemUnicastDropTableMutex_;

            numColumns = unicastDropTableLabels_.size();
          }

        // mutex syncronizes table map access
        std::lock_guard<std::mutex> m(*pThisMutex);
        
        auto iter = pThisDropCodeMap->find(nemId);

        if(iter == pThisDropCodeMap->end())
          {
            // the first column is the nemid
            std::vector<Any> v{Any{nemId}};

            // the rest are counters
            v.insert(v.end(), numColumns - 1, Any{Counter{}});

            pThisTable->addRow(nemId, v);

            iter = 
              pThisDropCodeMap->insert(std::make_pair(nemId,
                                                      Counters(numColumns - 1, Counter{}))).first;
          }

        if(dropCode > iter->second.size())
          {
            // throw exception XXX TODO
          }
        else if(dropCode != 0)
          {
            // bump internal table
            ++(iter->second[dropCode - 1]);

            // set statistic table
            pThisTable->setCell(nemId, dropCode, Any{iter->second[dropCode - 1]});
          }
      }
  }

 void updateAcceptTable(const PacketInfo & packetInfo,
                        size_t sizeInBytes,
                        bool bIsBroadcast,
                        bool bIsUpstream,
                        bool bAccepted)
  {
    NEMAcceptMap * pThisAcceptMap{};

    StatisticTable<NEMId> * pThisTable{};

    size_t numColumns{};

    std::mutex * pThisMutex{};

    NEMId nemId{};

    if(bIsBroadcast)
      {
        pThisAcceptMap = &nemBroadcastAcceptMap_;

        pThisTable = pStatisticBroadcastAcceptTable_;

        pThisMutex = &nemBroadcastAcceptTableMutex_;

        numColumns = broadcastAcceptTableLabels_.size();

        nemId = packetInfo.getSource();
      }
    else
      {
        pThisAcceptMap = &nemUnicastAcceptMap_;

        pThisTable = pStatisticUnicastAcceptTable_;

        pThisMutex = &nemUnicastAcceptTableMutex_;

        numColumns = unicastAcceptTableLabels_.size();

        nemId = packetInfo.getSource();
      }
    
    // mutex syncronizes table map access
    std::lock_guard<std::mutex> m(*pThisMutex);

    auto iter = pThisAcceptMap->find(nemId);
    
     if(iter == pThisAcceptMap->end())
       {
         // the first column is the nemid
         std::vector<Any> v{Any{nemId}};

         // the rest are counters
         v.insert(v.end(), numColumns - 1, Any{Counter{}});

         pThisTable->addRow(nemId, v);

         iter = 
           pThisAcceptMap->insert(std::make_pair(nemId,
                                                 BasicCounts{})).first;
       }

    if(bAccepted)
      {
        if(bIsUpstream)
          {
            // bump internal rx stats
            ++(iter->second.numRxPackets_);
            iter->second.numRxBytes_ += sizeInBytes;

            // set statistic table
            pThisTable->setCell(nemId, NUM_PKTS_RX, Any{iter->second.numRxPackets_});
            pThisTable->setCell(nemId, NUM_BYTES_RX, Any{iter->second.numRxBytes_});
          }
        else
          {
            // bump internal tx stats
            ++(iter->second.numTxPackets_);
            iter->second.numTxBytes_ += sizeInBytes;

            // set statistic table
            pThisTable->setCell(nemId, NUM_PKTS_TX, Any{iter->second.numTxPackets_});
            pThisTable->setCell(nemId, NUM_BYTES_TX , Any{iter->second.numTxBytes_});
         }
     }
  }


private:

  // keep this is line with the AcceptTableLables
  enum  AcceptTableLabelIndex {NUM_PKTS_TX = 1, NUM_BYTES_TX = 2, NUM_PKTS_RX = 3, NUM_BYTES_RX = 4};

  using Counter = std::uint64_t;

  struct BasicCounts {
     Counter numRxPackets_;
     Counter numRxBytes_;
     Counter numTxPackets_;
     Counter numTxBytes_;

     BasicCounts() :
       numRxPackets_{},
       numRxBytes_{},
       numTxPackets_{},
       numTxBytes_{}
     { }
  };

  using Average = float;

  using Counters = std::vector<Counter>;

  using NEMDropCodeMap = std::map<NEMId, Counters>;

  using NEMAcceptMap = std::map<NEMId, BasicCounts>;

  NEMDropCodeMap nemUnicastDropCodeMap_;

  NEMDropCodeMap nemBroadcastDropCodeMap_;

  NEMAcceptMap nemUnicastAcceptMap_;

  NEMAcceptMap nemBroadcastAcceptMap_;

  std::mutex nemUnicastDropTableMutex_;

  std::mutex nemBroadcastDropTableMutex_;

  std::mutex nemUnicastAcceptTableMutex_;

  std::mutex nemBroadcastAcceptTableMutex_;
  
  const std::string sInstance_;

  bool bHaveDropTables_;

  StatisticTableLabels unicastDropTableLabels_;

  StatisticTableLabels broadcastDropTableLabels_;

  StatisticTableLabels unicastAcceptTableLabels_;

  StatisticTableLabels broadcastAcceptTableLabels_;

   
  // upstream tx all
  StatisticNumeric<Counter> * pNumUpstreamPacketsTx_;
  StatisticNumeric<Counter> * pNumUpstreamBytesTx_;
  RunningAverage<Average>     avgUpstreamPacketSizeTx_;

  // upstream tx ucast
  StatisticNumeric<Counter> * pNumUpstreamPacketsUnicastTx_;
  StatisticNumeric<Counter> * pNumUpstreamBytesUnicastTx_;
  RunningAverage<Average>     avgUpstreamPacketSizeUnicastTx_;

  // upstream tx bcast
  StatisticNumeric<Counter> * pNumUpstreamPacketsBroadcastTx_;
  StatisticNumeric<Counter> * pNumUpstreamBytesBroadcastTx_;
  RunningAverage<Average>     avgUpstreamPacketSizeBroadcastTx_;

  // upstream rx all
  StatisticNumeric<Counter> * pNumUpstreamPacketsRx_;
  StatisticNumeric<Counter> * pNumUpstreamBytesRx_;
  StatisticNumeric<Counter> * pNumUpstreamPacketsDrop_;
  RunningAverage<Average>     avgUpstreamPacketSizeRx_;

  // upstream rx ucast
  StatisticNumeric<Counter> * pNumUpstreamPacketsUnicastRx_;
  StatisticNumeric<Counter> * pNumUpstreamBytesUnicastRx_;
  StatisticNumeric<Counter> * pNumUpstreamPacketsUnicastDrop_;
  RunningAverage<Average>     avgUpstreamPacketSizeUnicastRx_;

  // upstream rx bcast
  StatisticNumeric<Counter> * pNumUpstreamPacketsBroadcastRx_;
  StatisticNumeric<Counter> * pNumUpstreamBytesBroadcastRx_;
  StatisticNumeric<Counter> * pNumUpstreamPacketsBroadcastDrop_;
  RunningAverage<Average>     avgUpstreamPacketSizeBroadcastRx_;

  // upstream processing delay
  StatisticNumeric<Counter> * pNumUpstreamProcessingDelay_;
  RunningAverage<Average>     avgUpstreamProcessingDelay_;

  // downstream tx all
  StatisticNumeric<Counter> * pNumDownstreamPacketsTx_;
  StatisticNumeric<Counter> * pNumDownstreamBytesTx_;
  RunningAverage<Average>     avgDownstreamPacketSizeTx_;

  // downstream tx ucast
  StatisticNumeric<Counter> * pNumDownstreamPacketsUnicastTx_;
  StatisticNumeric<Counter> * pNumDownstreamBytesUnicastTx_;
  RunningAverage<Average>     avgDownstreamPacketSizeUnicastTx_;

  // downstream tx bcast
  StatisticNumeric<Counter> * pNumDownstreamPacketsBroadcastTx_;
  StatisticNumeric<Counter> * pNumDownstreamBytesBroadcastTx_;
  RunningAverage<Average>     avgDownstreamPacketSizeBroadcastTx_;

  // downstream rx all
  StatisticNumeric<Counter> * pNumDownstreamPacketsRx_;
  StatisticNumeric<Counter> * pNumDownstreamBytesRx_;
  StatisticNumeric<Counter> * pNumDownstreamPacketsDrop_;
  RunningAverage<Average>     avgDownstreamPacketSizeRx_;

  // downstream rx ucast
  StatisticNumeric<Counter> * pNumDownstreamPacketsUnicastRx_;
  StatisticNumeric<Counter> * pNumDownstreamBytesUnicastRx_;
  StatisticNumeric<Counter> * pNumDownstreamPacketsUnicastDrop_;
  RunningAverage<Average>     avgDownstreamPacketSizeUnicastRx_;

  // downstream rx bcast
  StatisticNumeric<Counter> * pNumDownstreamPacketsBroadcastRx_;
  StatisticNumeric<Counter> * pNumDownstreamBytesBroadcastRx_;
  StatisticNumeric<Counter> * pNumDownstreamPacketsBroadcastDrop_;
  RunningAverage<Average>     avgDownstreamPacketSizeBroadcastRx_;

  // downstream processing delay
  StatisticNumeric<Counter> * pNumDownstreamProcessingDelay_;
  RunningAverage<Average>     avgDownstreamProcessingDelay_;

  // downstream generated all
  StatisticNumeric<Counter> * pNumDownstreamPacketsGenerated_;
  StatisticNumeric<Counter> * pNumDownstreamBytesGenerated_;
  RunningAverage<Average>     avgDownstreamPacketSizeGenerated_;

  // downstream generated ucast
  StatisticNumeric<Counter> * pNumDownstreamPacketsUnicastGenerated_;
  StatisticNumeric<Counter> * pNumDownstreamBytesUnicastGenerated_;
  RunningAverage<Average>     avgDownstreamPacketSizeUnicastGenerated_;

  // downstream generated bcast
  StatisticNumeric<Counter> * pNumDownstreamPacketsBroadcastGenerated_;
  StatisticNumeric<Counter> * pNumDownstreamBytesBroadcastGenerated_;
  RunningAverage<Average>     avgDownstreamPacketSizeBroadcastGenerated_;

  StatisticTable<NEMId> * pStatisticUnicastDropTable_;
  StatisticTable<NEMId> * pStatisticBroadcastDropTable_;

  StatisticTable<NEMId> * pStatisticUnicastAcceptTable_;
  StatisticTable<NEMId> * pStatisticBroadcastAcceptTable_;


  inline bool isBroadcast(const DownstreamPacket & pkt)
  {
    return(pkt.getPacketInfo().getDestination() == NEM_BROADCAST_MAC_ADDRESS);
  }

  inline bool isBroadcast(const UpstreamPacket & pkt)
  {
    return(pkt.getPacketInfo().getDestination() == NEM_BROADCAST_MAC_ADDRESS);
  }

};



EMANE::Utils::CommonLayerStatistics::CommonLayerStatistics(const StatisticTableLabels & unicastDropTableLabels,
                                                           const StatisticTableLabels & broadcastDropTableLabels,
                                                           const std::string & sInstance):
pImpl_{new Implementation{unicastDropTableLabels, broadcastDropTableLabels, sInstance}}
{ }


EMANE::Utils::CommonLayerStatistics::~CommonLayerStatistics()
{ }


void EMANE::Utils::CommonLayerStatistics::registerStatistics(StatisticRegistrar & statisticRegistrar)
{
  pImpl_->registerStatistics(statisticRegistrar);
}


void EMANE::Utils::CommonLayerStatistics::processInbound(const UpstreamPacket & pkt)
{
  pImpl_->processInbound(pkt);
}


void EMANE::Utils::CommonLayerStatistics::processOutbound(const UpstreamPacket & pkt, 
                                                          Microseconds delay,
                                                          size_t dropCode)
{
  pImpl_->processOutbound(pkt, delay, dropCode);
}


void EMANE::Utils::CommonLayerStatistics::processInbound(const DownstreamPacket & pkt)
{
  pImpl_->processInbound(pkt);
}


void EMANE::Utils::CommonLayerStatistics::processOutbound(const DownstreamPacket & pkt, 
                                                          Microseconds delay,
                                                          size_t dropCode, 
                                                          bool bSelfGenerated)
{
  pImpl_->processOutbound(pkt, delay, dropCode, bSelfGenerated);
}
