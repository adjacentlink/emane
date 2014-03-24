/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "emane/types.h"

#include "idletxstate.h"

#include "broadcastpretxstate.h"
#include "unicastpretxstate.h"
#include "unicastrtsctspretxstate.h"

#include "modetimingparameters.h"
#include "maclayer.h"
#include "macstatistics.h"

EMANE::Models::IEEE80211ABG::IdleTxState::IdleTxState()
{ }

EMANE::Models::IEEE80211ABG::IdleTxState::~IdleTxState()
{ }


bool
EMANE::Models::IEEE80211ABG::IdleTxState::process(MACLayer * pMgr, DownstreamQueueEntry & entry)
{
  TimePoint beginTime{Clock::now()};

  // broadcast 
  if(entry.pkt_.getPacketInfo().getDestination() == EMANE::NEM_BROADCAST_MAC_ADDRESS)
    {
      // packet txop timed out
      if((entry.txOpMicroseconds_.count() != 0) && (entry.txOpMicroseconds_ + entry.acquireTime_) < beginTime)
        {
          // set discard due to txop timeout
          pMgr->getStatistics().incrementDownstreamBroadcastDataDiscardDueToTxop();

          // get next packet
          return false;
        }
      else
        {
          // set pre/post tx delay time
          pMgr->setDelayTime(entry);

          // change state to broadcast pre tx state
          changeState(pMgr, BroadcastPreTxStateSingleton::instance());

          // continue to process
          return true;
        }
    }
  // unicast 
  else
    {
      // packet txop timed out
      if((entry.txOpMicroseconds_.count() != 0) && (entry.txOpMicroseconds_ + entry.acquireTime_) < beginTime)
        {
          // set discard due to txop timeout
          pMgr->getStatistics().incrementDownstreamUnicastDataDiscardDueToTxop();

          // get next packet
          return false;
        }
      else
        {
          // set pre/post tx delay time
          pMgr->setDelayTime(entry);

          // check rts cts enable
          if(entry.bRtsCtsEnable_ == true)
           {
             // change state to unicast rts cts pre tx state
             changeState(pMgr, UnicastRtsCtsPreTxStateSingleton::instance());
           }
          else
           {
             // change state to unicast pre tx state
             changeState(pMgr, UnicastPreTxStateSingleton::instance());
           }

          // continue to process
          return true;
        }
    }
}



std::pair<EMANE::TimePoint,bool>
EMANE::Models::IEEE80211ABG::IdleTxState::getWaitTime(DownstreamQueueEntry &)
{
  // no wait time
  return {TimePoint{},false};
}


const char *
EMANE::Models::IEEE80211ABG::IdleTxState::statename()
{
  return "IdleTxState";
}
