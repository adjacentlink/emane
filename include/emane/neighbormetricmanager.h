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
 * * Neither the name of Adjacent Link, LLC nor the names of its
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


#ifndef EMANENEIGHBORMETRICMANAGER_HEADER_
#define EMANENEIGHBORMETRICMANAGER_HEADER_

#include "emane/types.h"
#include "emane/controls/r2rineighbormetriccontrolmessage.h"
#include "emane/statisticregistrar.h"


namespace EMANE
{
 /**
  *
  * @class NeighborMetricManager
  *
  * @brief Manages neighbor metrics and sends neighbor
  * metric control message upstream
  *
  */
  class NeighborMetricManager
  { 
    public:
     /**
      * Creates a NeighborMetricManager instance
      *
      * @param nemId NEM id
      */
      NeighborMetricManager(NEMId nemId);
    
     /**
      * Destroys an instance
      *
      */
      ~NeighborMetricManager();

     /**
      * updates neighbor send metric
      *
      * @param dst            dst NEM (nbr)
      * @param u64DataRatebps data rate in bps
      * @param txTime         pkt tx time
      *
      */
      void updateNeighborTxMetric(NEMId dst,
                                  std::uint64_t
                                  u64DataRatebps,
                                  const TimePoint & txTime);


     /**
      * Updates neighbor recv metric
      *
      * @param src          src NEM (nbr)
      * @param u16SeqNum    pkt sequence number
      * @param rxTime       pkt rx time
      *
      */
      void updateNeighborRxMetric(NEMId src,
                                  std::uint16_t u16SeqNum,
                                  const TimePoint & rxTime);


     /**
      * Updates neighbor receive metric
      *
      * @param src                  src NEM (nbr)
      * @param u16SeqNum            pkt sequence number
      * @param fSINR                sinr in dbm
      * @param fNoiseFloor          noise floor in dBm
      * @param rxTime               pkt rx time
      * @param durationMicroseconds pkt duration
      * @param u64DataRatebps       data rate in bps
      *
      */
      void updateNeighborRxMetric(const NEMId src, 
                                  std::uint16_t u16SeqNum,
                                  float fSINR,
                                  float fNoiseFloor,
                                  const TimePoint & rxTime,
                                  const Microseconds & durationMicroseconds,
                                  std::uint64_t u64DataRatebps);

    /**
      * Sets neighbor delete time (age)
      *
      * @param ageMicroseconds the neighbor delete time (relative)
      *
      */
      void setNeighborDeleteTimeMicroseconds(const Microseconds & ageMicroseconds);

     /**
      * Gets neighbor metrics
      *
      * @return neighbore metrics
      */
     Controls::R2RINeighborMetrics getNeighborMetrics();

     void registerStatistics(StatisticRegistrar & statisticRegistrar);

   private:
     class Implementation;

     std::unique_ptr<Implementation> pImpl_;

     NeighborMetricManager(const NeighborMetricManager &) = delete;

     NeighborMetricManager & operator=(const NeighborMetricManager &) = delete;
  };
}

#endif // EMANENEIGHBORMETRICMANAGER_HEADER_
