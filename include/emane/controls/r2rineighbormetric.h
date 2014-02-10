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

#ifndef EMANECONTROLSR2RINEIGHBORMETRIC_HEADER_
#define EMANECONTROLSR2RINEIGHBORMETRIC_HEADER_

#include "emane/types.h"

#include <list>

namespace EMANE
{
  namespace Controls
  {
    /**
     * @class R2RINeighborMetric
     *
     * @brief R2RI neighbor metrics are used in conjunction with the
     * R2RINeighborMetricControlMessage to inform an NEM's transport layer
     * of MAC neighbor state.
     *
     * @note Instances are immutable
     */
    class R2RINeighborMetric
    {
    public:
      /**
       * Creates an R2RINeighborMetric
       *
       * @param id Neighbor NEM id
       * @param u32NumRxFrames Number of frames received from the neighbor
       * @param u32NumTxFrames Number of frames sent to the neighbor
       * @param u32NumMissedFrames Number of frames missed from the neighbor
       * @param bandwidthConsumption Bandwidth consumption in microseconds
       * @param fSINRAvgdBm Average SINR from neighbor in dBm
       * @param fSINRStddev Standard deviation SINR from neighbor
       * @param fNoiseFloorAvgdBm Average noise floor from neighbor in dBm
       * @param fNoiseFloorStddev Standard deviation noise floor from neighbor
       * @param u64RxAvgDataRatebps Average Rx datarate from neighbor bps
       * @param u64TxAvgDataRatebps Average Tx datarate to neighbor bps
       *
       * @note All data is over the report interval
       *
       * @see R2RISelfMetricControlMessage for report interval
       */
      R2RINeighborMetric(NEMId id,
                         std::uint32_t u32NumRxFrames,
                         std::uint32_t u32NumTxFrames,
                         std::uint32_t u32NumMissedFrames,
                         const Microseconds & bandwidthConsumption,
                         float fSINRAvgdBm,
                         float fSINRStddev,
                         float fNoiseFloorAvgdBm,
                         float fNoiseFloorStddev,
                         std::uint64_t u64RxAvgDataRatebps,
                         std::uint64_t u64TxAvgDataRatebps);
      
      /** 
       * Destroys an instance
       */
      ~R2RINeighborMetric();
      
      /**
       * Get neighbor NEM Id
       *
       * @return NEM Id
       */
      NEMId getId() const;
      
      /**
       * Gets the number of frames received during the report interval
       * 
       * @return number of frames
       */
      std::uint32_t getNumRxFrames() const;

      /**
       * Gets the number of frames transmitted during the report interval
       * 
       * @return number of frames
       */
      std::uint32_t getNumTxFrames() const;
      
      /**
       * Gets the number of frames missed during the report interval
       * 
       * @return number of frames
       */
      std::uint32_t getNumMissedFrames() const;
      
      /**
       * Gets the bandwidth consumption over the report interval
       * 
       * @return bandwidth consumption
       */
      const Microseconds & getBandwidthConsumption() const;
      
      /**
       * Gets the average SINR over the report interval
       *
       * @return average SINR
       */
      float getSINRAvgdBm() const;

      /**
       * Gets the SINR standard deviation over the report interval
       * 
       * @return SINR standard deviation
       */
      float getSINRStddev() const;
      
      /**
       * Gets the average noise floor over the report interval
       *
       * @return average noise floor
       */
      float getNoiseFloorAvgdBm() const;
      
      /**
       * Gets the noise floor standard deviation over the report interval
       *
       * @return noise floor standard deviation
       */
      float getNoiseFloorStddev() const;
      
      /**
       * Gets the average receive datarate in bps over the report interval
       *
       * @return average data rate
       */
      std::uint64_t getRxAvgDataRatebps() const;

      /**
       * Gets the average transmit datarate in bps over the report interval
       *
       * @return average data rate
       */
      std::uint64_t getTxAvgDataRatebps() const;
      
    private:
      const NEMId id_;
      const std::uint32_t u32NumRxFrames_;
      const std::uint32_t u32NumTxFrames_;
      const std::uint32_t u32NumMissedFrames_;
      const Microseconds bandwidthConsumption_;
      const float fSINRAvgdBm_;
      const float fSINRStddev_;
      const float fNoiseFloorAvgdBm_;
      const float fNoiseFloorStddev_;
      const std::uint64_t u64RxAvgDataRatebps_;
      const std::uint64_t u64TxAvgDataRatebps_;
    };

    typedef std::list<R2RINeighborMetric> R2RINeighborMetrics;
  }
}


#include "r2rineighbormetric.inl"

#endif // EMANECONTROLSR2RINEIGHBORMETRIC_HEADER_
