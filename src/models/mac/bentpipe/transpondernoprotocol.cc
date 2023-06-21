/*
 * Copyright (c) 2023 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "transpondernoprotocol.h"
#include "transponderuser.h"

EMANE::Models::BentPipe::TransponderNoProtocol::TransponderNoProtocol(NEMId id,
                                                                      PlatformServiceProvider * pPlatformService,
                                                                      TransponderUser * pTransponderUser,
                                                                      const TransponderConfiguration & transponderConfiguration):
  Transponder{id,pPlatformService,pTransponderUser,transponderConfiguration},
  eot_{},
  u64TxOpportunityIndex_{}{}

void EMANE::Models::BentPipe::TransponderNoProtocol::start()
{

}

void EMANE::Models::BentPipe::TransponderNoProtocol::stop()
{

}

bool EMANE::Models::BentPipe::TransponderNoProtocol::isTransmitOpportunity(const TimePoint & now)
{
  return now >= eot_;
}

size_t EMANE::Models::BentPipe::TransponderNoProtocol::getMTUBytes() const
{
  return configuration_.getTransmitMTUBytes();
}

EMANE::Models::BentPipe::Transponder::TransmissionInfo
EMANE::Models::BentPipe::TransponderNoProtocol::prepareTransmission(const TimePoint & now,
                                                                    size_t lengthBytes,
                                                                    MessageComponents && components)
{
  Microseconds durationMicroseconds{std::chrono::duration_cast<Microseconds>(DoubleSeconds{(lengthBytes * 8.0) / configuration_.getTransmitDataRatebps()})};

  ++u64TxOpportunityIndex_;

  eot_ = now + durationMicroseconds;

  // set a timer for the next tx opporunity
  pPlatformService_->timerService().
    schedule(std::bind(&TransponderNoProtocol::processTxOpportunity,
                       this,
                       u64TxOpportunityIndex_),
             eot_);

  return {BentPipeMessage{now,
                          configuration_.getPCRCurveIndex(),
                          std::move(components)},
          durationMicroseconds};
}

void EMANE::Models::BentPipe::TransponderNoProtocol::processTxOpportunity(std::uint64_t u64TxOpportunityIndex)
{
  if(u64TxOpportunityIndex == u64TxOpportunityIndex_)
    {
      // signal to radio model this transponder has a tx opportunity
      pTransponderUser_->notifyTxOpportunity(this);
    }
}
