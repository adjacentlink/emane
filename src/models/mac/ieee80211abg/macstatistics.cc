/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
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


#include "macstatistics.h"

/**
 *
 * @brief constructor
 *
 */
EMANE::Models::IEEE80211ABG::MACStatistics::MACStatistics(EMANE::NEMId id):
  id_{id},
  pNumDownstreamUnicastDataDiscardDueToRetries_{},
  pNumDownstreamUnicastRtsCtsDataDiscardDueToRetries_{},
  pNumUpstreamUnicastDataDiscardDueToSinr_{},
  pNumUpstreamBroadcastDataDiscardDueToSinr_{},
  pNumUpstreamUnicastDataDiscardDueToClobberRxDuringTx_{},
  pNumUpstreamBroadcastDataDiscardDueToClobberRxDuringTx_{},
  pNumUpstreamUnicastDataDiscardDueToClobberRxHiddenBusy_{},
  pNumUpstreamBroadcastDataDiscardDueToClobberRxHiddenBusy_{},
  pNumDownstreamUnicastDataDiscardDueToTxop_{},
  pNumDownstreamBroadcastDataDiscardDueToTxop_{},
  pNumUpstreamUnicastRtsCtsDataRxFromPhy_{},
  pNumUpstreamUnicastCtsRxFromPhy_{},
  pNumUpstreamUnicastDataNoiseHiddenRx_{},
  pNumUpstreamBroadcastDataNoiseHiddenRx_{},
  pNumUpstreamUnicastDataNoiseRxCommon_{},
  pNumUpstreamBroadcastDataNoiseRxCommon_{},
  pNumOneHopNbrHighWaterMark_{},
  pNumTwoHopNbrHighWaterMark_{},
  pNumRxOneHopNbrListEvents_{},
  pNumRxOneHopNbrListInvalidEvents_{},
  pNumTxOneHopNbrListEvents_{}
{}


/**
*
* @brief destructor
*
*/
EMANE::Models::IEEE80211ABG::MACStatistics::~MACStatistics()
{}


void EMANE::Models::IEEE80211ABG::MACStatistics::registerStatistics(StatisticRegistrar & statisticRegistrar)
{
  
  pNumDownstreamUnicastDataDiscardDueToRetries_ =
    statisticRegistrar.registerNumeric<std::uint32_t>("numDownstreamUnicastDataDiscardDueToRetries", 
                                                      StatisticProperties::CLEARABLE);
                                                      
  pNumDownstreamUnicastRtsCtsDataDiscardDueToRetries_ =
    statisticRegistrar.registerNumeric<std::uint32_t>("numDownstreamUnicastRtsCtsDataDiscardDueToRetries", 
                                                      StatisticProperties::CLEARABLE);

  pNumUpstreamUnicastDataDiscardDueToSinr_ = 
    statisticRegistrar.registerNumeric<std::uint32_t>("numUpstreamUnicastDataDiscardDueToSinr", 
                                                      StatisticProperties::CLEARABLE);
  pNumUpstreamBroadcastDataDiscardDueToSinr_ = 
    statisticRegistrar.registerNumeric<std::uint32_t>("numUpstreamBroadcastDataDiscardDueToSinr", 
                                                      StatisticProperties::CLEARABLE);

  pNumUpstreamUnicastDataDiscardDueToClobberRxDuringTx_ = 
    statisticRegistrar.registerNumeric<std::uint32_t>("numUpstreamUnicastDataDiscardDueToClobberRxDuringTx", 
                                                      StatisticProperties::CLEARABLE);

  pNumUpstreamBroadcastDataDiscardDueToClobberRxDuringTx_ =
    statisticRegistrar.registerNumeric<std::uint32_t>("numUpstreamBroadcastDataDiscardDueToClobberRxDuringTx", 
                                                      StatisticProperties::CLEARABLE);

  pNumUpstreamUnicastDataDiscardDueToClobberRxHiddenBusy_ = 
    statisticRegistrar.registerNumeric<std::uint32_t>("numUpstreamUnicastDataDiscardDueToClobberRxHiddenBusy", 
                                                      StatisticProperties::CLEARABLE);

  pNumUpstreamBroadcastDataDiscardDueToClobberRxHiddenBusy_ = 
    statisticRegistrar.registerNumeric<std::uint32_t>("numUpstreamBroadcastDataDiscardDueToClobberRxHiddenBusy", 
                                                      StatisticProperties::CLEARABLE);
  pNumDownstreamUnicastDataDiscardDueToTxop_ = 
    statisticRegistrar.registerNumeric<std::uint32_t>("numDownstreamUnicastDataDiscardDueToTxop",
                                                      StatisticProperties::CLEARABLE);
  pNumDownstreamBroadcastDataDiscardDueToTxop_ =
    statisticRegistrar.registerNumeric<std::uint32_t>("numDownstreamBroadcastDataDiscardDueToTxop", 
                                                      StatisticProperties::CLEARABLE);

  pNumUpstreamUnicastDataNoiseHiddenRx_ =
    statisticRegistrar.registerNumeric<std::uint32_t>("numUpstreamUnicastDataNoiseHiddenRx", 
                                                      StatisticProperties::CLEARABLE);

  pNumUpstreamBroadcastDataNoiseHiddenRx_ =
    statisticRegistrar.registerNumeric<std::uint32_t>("numUpstreamBroadcastDataNoiseHiddenRx", 
                                                      StatisticProperties::CLEARABLE);

  pNumUpstreamUnicastDataNoiseRxCommon_ = 
    statisticRegistrar.registerNumeric<std::uint32_t>("numUpstreamUnicastDataNoiseRxCommon", 
                                                      StatisticProperties::CLEARABLE);

  pNumUpstreamBroadcastDataNoiseRxCommon_ =
    statisticRegistrar.registerNumeric<std::uint32_t>("numUpstreamBroadcastDataNoiseRxCommon", 
                                                      StatisticProperties::CLEARABLE);

  pNumUpstreamUnicastRtsCtsDataRxFromPhy_ =
    statisticRegistrar.registerNumeric<std::uint32_t>("numUpstreamUnicastRtsCtsDataRxFromPhy", 
                                                      StatisticProperties::CLEARABLE);

  pNumUpstreamUnicastCtsRxFromPhy_ = 
    statisticRegistrar.registerNumeric<std::uint32_t>("numUpstreamUnicastRtsCtsRxFromPhy",  
                                                      StatisticProperties::CLEARABLE);

  pNumOneHopNbrHighWaterMark_ = 
    statisticRegistrar.registerNumeric<std::uint32_t>("numOneHopNbrHighWaterMark", 
                                                      StatisticProperties::CLEARABLE);

  pNumTwoHopNbrHighWaterMark_ = 
    statisticRegistrar.registerNumeric<std::uint32_t>("numTwoHopNbrHighWaterMark",
                                                      StatisticProperties::CLEARABLE);

  pNumRxOneHopNbrListEvents_ =
    statisticRegistrar.registerNumeric<std::uint32_t>("numRxOneHopNbrListEvents",
                                                      StatisticProperties::CLEARABLE);
  pNumRxOneHopNbrListInvalidEvents_ = 
    statisticRegistrar.registerNumeric<std::uint32_t>("numRxOneHopNbrListInvalidEvents",
                                                      StatisticProperties::CLEARABLE);
  pNumTxOneHopNbrListEvents_ =
    statisticRegistrar.registerNumeric<std::uint32_t>("numTxOneHopNbrListEvents",
                                                      StatisticProperties::CLEARABLE);
}



/**
*
* @brief increment unicast rts/ctsdata recv from phy
*
*/
void
EMANE::Models::IEEE80211ABG::MACStatistics::incrementUpstreamUnicastRtsCtsDataRxFromPhy()
{
  // bump counter
  ++*pNumUpstreamUnicastRtsCtsDataRxFromPhy_;
}


/**
*
* @brief increment unicast cts recv from phy
*
*/
void
EMANE::Models::IEEE80211ABG::MACStatistics::incrementUpstreamUnicastCtsRxFromPhy()
{
  // bump counter
  ++*pNumUpstreamUnicastCtsRxFromPhy_;
}



/**
*
* @brief increment unicast data discard due to exhausted retries
*
*/
void
EMANE::Models::IEEE80211ABG::MACStatistics::incrementDownstreamUnicastDataDiscardDueToRetries()
{
  // bump counter
  ++*pNumDownstreamUnicastDataDiscardDueToRetries_;
}


/**
*
* @brief increment braodcast data discard due to exhausted retries
*
*/
void
EMANE::Models::IEEE80211ABG::MACStatistics::incrementDownstreamUnicastRtsCtsDataDiscardDueToRetries()
{
  // bump counter
  ++*pNumDownstreamUnicastRtsCtsDataDiscardDueToRetries_;
}



/**
*
* @brief increment unicast data discard due to sinr
*
*/
void
EMANE::Models::IEEE80211ABG::MACStatistics::incrementUpstreamUnicastDataDiscardDueToSinr()
{
  // bump counter
  ++*pNumUpstreamUnicastDataDiscardDueToSinr_;
}


/**
*
* @brief increment braodcast data discard due to sinr
*
*/
void
EMANE::Models::IEEE80211ABG::MACStatistics::incrementUpstreamBroadcastDataDiscardDueToSinr()
{
  // bump counter
  ++*pNumUpstreamBroadcastDataDiscardDueToSinr_;
}


/**
*
* @brief increment unicast data discard due to collision rx during tx
*
*/
void
EMANE::Models::IEEE80211ABG::MACStatistics::incrementUpstreamUnicastDataDiscardDueToClobberRxDuringTx()
{
  // bump counter
  ++*pNumUpstreamUnicastDataDiscardDueToClobberRxDuringTx_;
}


/**
*
* @brief increment braodcast data discard due to collision rx during tx
*
*/
void
EMANE::Models::IEEE80211ABG::MACStatistics::incrementUpstreamBroadcastDataDiscardDueToClobberRxDuringTx()
{
  // bump counter
  ++*pNumUpstreamBroadcastDataDiscardDueToClobberRxDuringTx_;
}


/**
*
* @brief increment unicast data discard due to collision rx busy hidden
*
*/
void
EMANE::Models::IEEE80211ABG::MACStatistics::incrementUpstreamUnicastDataDiscardDueToClobberRxHiddenBusy()
{
  // bump counter
  ++*pNumUpstreamUnicastDataDiscardDueToClobberRxHiddenBusy_;
}


/**
*
* @brief increment braodcast data discard due to collision rx busy hidden
*
*/
void
EMANE::Models::IEEE80211ABG::MACStatistics::incrementUpstreamBroadcastDataDiscardDueToClobberRxHiddenBusy()
{
  // bump counter
  ++*pNumUpstreamBroadcastDataDiscardDueToClobberRxHiddenBusy_;
}



/**
*
* @brief increment unicast data discard due to txop expired
*
*/
void
EMANE::Models::IEEE80211ABG::MACStatistics::incrementDownstreamUnicastDataDiscardDueToTxop()
{
  // bump counter
  ++*pNumDownstreamUnicastDataDiscardDueToTxop_;
}



/**
*
* @brief increment unicast data discard due to txop expired
*
*/
void
EMANE::Models::IEEE80211ABG::MACStatistics::incrementDownstreamBroadcastDataDiscardDueToTxop()
{
  // bump counter
  ++*pNumDownstreamBroadcastDataDiscardDueToTxop_;
}


/**
*
* @brief increment unicastcast data collision due to hidden rx
*
*/
void 
EMANE::Models::IEEE80211ABG::MACStatistics::incrementUpstreamUnicastNoiseHiddenRx()
{
  // bump counter
  ++*pNumUpstreamUnicastDataNoiseHiddenRx_;
}



/**
*
* @brief increment braodcastcast data collision due to hidden rx
*
*/
void 
EMANE::Models::IEEE80211ABG::MACStatistics::incrementUpstreamBroadcastNoiseHiddenRx()
{
  // bump counter
  ++*pNumUpstreamBroadcastDataNoiseHiddenRx_;
}



/**
*
* @brief increment unicastcast data collision due to rx common
*
*/
void 
EMANE::Models::IEEE80211ABG::MACStatistics::incrementUpstreamBroadcastNoiseRxCommon()
{
  // bump counter
  ++*pNumUpstreamBroadcastDataNoiseRxCommon_;
}


/**
*
* @brief increment braodcastcast data collision due to rx common
*
*/
void 
EMANE::Models::IEEE80211ABG::MACStatistics::incrementUpstreamUnicastNoiseRxCommon()
{
  // bump counter
  ++*pNumUpstreamUnicastDataNoiseRxCommon_;
}



/**
*
* @brief set the one hop nbr high water mark
*
*/
void 
EMANE::Models::IEEE80211ABG::MACStatistics::updateOneHopNbrHighWaterMark(size_t num)
{
   if(*pNumOneHopNbrHighWaterMark_ < num)
    {
       *pNumOneHopNbrHighWaterMark_ = num;
    }
}




/**
*
* @brief set the two hop nbr high water mark
*
*/
void 
EMANE::Models::IEEE80211ABG::MACStatistics::updateTwoHopNbrHighWaterMark(size_t num)
{
   if(*pNumTwoHopNbrHighWaterMark_ < num)
    {
      *pNumTwoHopNbrHighWaterMark_ = num;
    }
}


/**
*
* @brief increment number rx one hop nbr list events
*
*/
void 
EMANE::Models::IEEE80211ABG::MACStatistics::incrementRxOneHopNbrListEventCount()
{
  // bump counter
  ++*pNumRxOneHopNbrListEvents_;
}


/**
*
* @brief increment number rx one hop nbr list invalid events
*
*/
void 
EMANE::Models::IEEE80211ABG::MACStatistics::incrementRxOneHopNbrListInvalidEventCount()
{
  // bump counter
  ++*pNumRxOneHopNbrListInvalidEvents_;
}



/**
*
* @brief increment number tx one hop nbr list events
*
*/
void 
EMANE::Models::IEEE80211ABG::MACStatistics::incrementTxOneHopNbrListEventCount()
{
  // bump counter
  ++*pNumTxOneHopNbrListEvents_;
}

