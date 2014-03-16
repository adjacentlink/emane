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

#include "emane/types.h"
#include "emane/neighbormetricmanager.h"
#include "emane/statistictable.h"

#include <cmath>
#include <map>
#include <mutex>


namespace {
  const EMANE::StatisticTableLabels sNeighborMetricLables {"NEM",
                                                           "Rx Pkts", 
                                                           "Tx Pkts", 
                                                           "Missed Pkts", 
                                                           "BW Util", 
                                                           "Last Rx",
                                                           "Last Tx",
                                                           "SINR Avg", 
                                                           "SINR Stdv", 
                                                           "NF Avg", 
                                                           "NF Stdv", 
                                                           "Rx Rate Avg", 
                                                           "Tx Rate Avg"};

  const EMANE::StatisticTableLabels sNeighborStatusLables {"NEM",
                                                           "Rx Pkts", 
                                                           "Tx Pkts", 
                                                           "Missed Pkts", 
                                                           "BW Util Ratio", 
                                                           "SINR Avg", 
                                                           "NF Avg", 
                                                           "Rx Age"};
}


class EMANE::NeighborMetricManager::Implementation
  {
    public:
      Implementation(EMANE::NEMId nemId) :
       nemId_{nemId},
       lastNeighborStatusUpdateTime_{}
      { }

     ~Implementation()
      { }


    void setNeighborDeleteTimeMicroseconds(const Microseconds & ageMicroseconds)
      {
        neighborDeleteAgeMicroseconds_ = ageMicroseconds;
      }


    void handleTxActivity(NEMId dst, std::uint64_t u64DataRatebps, const TimePoint & txTime)
      {
        // exclude tx broadcast from the r2ri reports
        if(dst != EMANE::NEM_BROADCAST_MAC_ADDRESS)
          {
            handleR2RITxActivity(dst, u64DataRatebps, txTime);
          }

        handleNeighborTxActivity(dst, u64DataRatebps, txTime);
      }


    void handleRxActivity(NEMId src, 
                          std::uint64_t u64SeqNum, 
                          const uuid_t & uuid,
                          const EMANE::TimePoint & rxTime)
      {
        handleR2RIRxActivity(src, u64SeqNum, uuid, rxTime);

        updateNeighborRxActivity(src, u64SeqNum, uuid, rxTime);
      }



    void handleRxActivity(NEMId src, 
                          std::uint64_t u64SeqNum,
                          const uuid_t & uuid,
                          float fSINR,
                          float fNoiseFloor,
                          const EMANE::TimePoint & rxTime,
                          const EMANE::Microseconds & durationMicroseconds,
                          std::uint64_t u64DataRatebps)
      {
        handleR2RIRxActivity(src, 
                             u64SeqNum, 
                             uuid,
                             fSINR, 
                             fNoiseFloor,
                             rxTime,
                             durationMicroseconds,
                             u64DataRatebps);

        updateNeighborRxActivity(src, 
                                 u64SeqNum, 
                                 uuid,
                                 fSINR, 
                                 fNoiseFloor,
                                 rxTime,
                                 durationMicroseconds,
                                 u64DataRatebps);
      }



    void handleNeighborStatusUpdate()
      {
        EMANE::TimePoint currentTime{Clock::now()};

        StatisticTable<NEMId> * pTable = pStatisticNeighborStatusTable_;

        for(auto iter : neighborDataTable_)
          {
            NeighborData * pNeighborMetric = iter.second.first;

            float fBWUtilzationRatio = (lastNeighborStatusUpdateTime_ == EMANE::TimePoint{}) ? 0.0f :
                             std::chrono::duration_cast<EMANE::DoubleSeconds>(pNeighborMetric->rxUtilizationMicroseconds_).count() /
                             std::chrono::duration_cast<EMANE::DoubleSeconds>(currentTime - lastNeighborStatusUpdateTime_).count();

            float fRxAge = (! pNeighborMetric->bHaveEverHadRxActivity_) ? 0.0f :
                              std::chrono::duration_cast<DoubleSeconds>(
                                currentTime - pNeighborMetric->lastRxTime_).count();

            float fSINRAvg = getAvg(pNeighborMetric->fSINRSum_, pNeighborMetric->u64NumRxFrames_);

            float fNoiseFloorAvg = getAvg(pNeighborMetric->fNoiseFloorSum_, pNeighborMetric->u64NumRxFrames_);

            pTable->setCell(iter.first, 
                             NBR_STATUS_TX_FRAMES, 
                             Any{pNeighborMetric->u64NumTxFrames_});

            pTable->setCell(iter.first, 
                             NBR_STATUS_RX_FRAMES, 
                             Any{pNeighborMetric->u64NumRxFrames_});

            pTable->setCell(iter.first,
                             NBR_STATUS_MISSED_FRAMES, 
                             Any{pNeighborMetric->u64NumRxMissedFrames_});

            pTable->setCell(iter.first,
                             NBR_STATUS_BW_CONSUMPTION, 
                             Any{fBWUtilzationRatio});

            pTable->setCell(iter.first,
                             NBR_STATUS_SINR_AVG, 
                             Any{fSINRAvg});

            pTable->setCell(iter.first,
                             NBR_STATUS_NOISE_FLOOR_AVG, 
                             Any{fNoiseFloorAvg});

            pTable->setCell(iter.first,
                             NBR_STATUS_RX_AGE, 
                             Any{fRxAge});

            clearData_i(pNeighborMetric);
          }

        lastNeighborStatusUpdateTime_ = currentTime;
      }



     EMANE::Controls::R2RINeighborMetrics getNeighborMetrics()
      {
        EMANE::TimePoint currentTime{Clock::now()};

        EMANE::Controls::R2RINeighborMetrics neighborMetrics{};

        for(auto iter = r2riMetricTable_.begin(); iter != r2riMetricTable_.end(); /* bump/erase below */)
          {
            // get the age
            const Microseconds ageMicroseconds{
              std::chrono::duration_cast<Microseconds>(currentTime - iter->second->lastRxTime_)};  

            // only the entries for the r2ri reporting are checked for age     
            if(ageMicroseconds > neighborDeleteAgeMicroseconds_)
              {
                delete (iter->second);
                
                // erase and bump
                r2riMetricTable_.erase(iter++);
              }
            else
              {
                float fSINRAvg{}, fSINRStd{}, fNoiseFloorAvg{}, fNoiseFloorStdv{};

                // get sinr avg and standard deviation 
                getAvgAndStd(iter->second->fSINRSum_, 
                             iter->second->fSINRSum2_, 
                             iter->second->u64NumRxFrames_, 
                             fSINRAvg,
                             fSINRStd);

                // get noise floor avg and standard deviation 
                getAvgAndStd(iter->second->fNoiseFloorSum_, 
                             iter->second->fNoiseFloorSum2_, 
                             iter->second->u64NumRxFrames_, 
                             fNoiseFloorAvg,
                             fNoiseFloorStdv);

                EMANE::Controls::R2RINeighborMetric 
                  neighborMetric{iter->first,                              // nbr id
                                 iter->second->u64NumRxFrames_,            // num rx frames (samples)
                                 iter->second->u64NumTxFrames_,            // num tx frames (samples)
                                 iter->second->u64NumRxMissedFrames_,      // num rx missed frames
                                 iter->second->rxUtilizationMicroseconds_, // bandwidth consumption
                                 fSINRAvg,                                 // sinr avg 
                                 fSINRStd,                                 // sinr std deviation
                                 fNoiseFloorAvg,                           // noisefloor avg
                                 fNoiseFloorStdv,                          // noise floor std deviation
                                 iter->second->u64RxDataRateAvg_,          // avg rx datarate
                                 iter->second->u64TxDataRateAvg_};         // avg tx datarate

                neighborMetrics.push_back(neighborMetric);

                clearData_i(iter->second);

                // bump 
                ++iter;
              }
           }

         return neighborMetrics;
       }

     void registerStatistics(StatisticRegistrar & statisticRegistrar)
      {
        pStatisticNeighborMetricTable_ =
          statisticRegistrar.registerTable<NEMId>("NeighborMetricTable", 
                                                  sNeighborMetricLables,
                                                  StatisticProperties::NONE,
                                                  "Neighbor Metric Table");

        pStatisticNeighborStatusTable_ =
          statisticRegistrar.registerTable<NEMId>("NeighborStatusTable", 
                                                  sNeighborStatusLables,
                                                  StatisticProperties::NONE,
                                                  "Neighbor Status Table");
      }


    private:
      // keep this is line with the NeighborMetricLabels
      enum NeighborMetricLables { NBR_METRIC_RX_FRAMES        = 1,   // num total rx frames
                                  NBR_METRIC_TX_FRAMES        = 2,   // num total tx frames
                                  NBR_METRIC_MISSED_FRAMES    = 3,   // num total missed frames
                                  NBR_METRIC_BW_CONSUMPTION   = 4,   // total bw consumption
                                  NBR_METRIC_LAST_RX_TIME     = 5,   // last rx time
                                  NBR_METRIC_LAST_TX_TIME     = 6,   // last tx time
                                  NBR_METRIC_SINR_AVG         = 7,   // avg sinr rx
                                  NBR_METRIC_SINR_STDV        = 8,   // sinr std dev rx
                                  NBR_METRIC_NOISE_FLOOR_AVG  = 9,   // noise floor avg rx
                                  NBR_METRIC_NOISE_FLOOR_STDV = 10,  // noise floor std dev rx
                                  NBR_METRIC_RX_DATARATE_AVG  = 11,  // avg rx data rate
                                  NBR_METRIC_TX_DATARATE_AVG  = 12}; // avg tx data rate 

      // keep this is line with the NeighborStatusLabels
      enum NeighborStatusLables { NBR_STATUS_RX_FRAMES        = 1,   // num frames rx this interval
                                  NBR_STATUS_TX_FRAMES        = 2,   // num frames tx this interval
                                  NBR_STATUS_MISSED_FRAMES    = 3,   // num frames missed this interval
                                  NBR_STATUS_BW_CONSUMPTION   = 4,   // bandwidth consumption ratio 
                                  NBR_STATUS_SINR_AVG         = 5,   // avg sinr this interval
                                  NBR_STATUS_NOISE_FLOOR_AVG  = 6,   // avg noise floor this interval
                                  NBR_STATUS_RX_AGE           = 7};  // time elapsed since last heard from 


      // neighbor data used for r2ri and neigbor metric/status
      struct NeighborData {
        const NEMId      nemId_;

        std::uint64_t    u64LastRxSeqNum_;
 
        bool             bHaveEverHadRxActivity_;

        EMANE::TimePoint lastRxTime_;
        EMANE::TimePoint lastTxTime_;

        std::uint64_t    u64NumRxFrames_;
        std::uint64_t    u64NumRxMissedFrames_;
        std::uint64_t    u64NumTxFrames_;

        EMANE::Microseconds  rxUtilizationMicroseconds_;

        float          fSINRSum_;
        float          fSINRSum2_;
        float          fNoiseFloorSum_;
        float          fNoiseFloorSum2_;

        std::uint64_t  u64RxDataRateMin_;
        std::uint64_t  u64RxDataRateMax_;
        std::uint64_t  u64RxDataRateAvg_;

        std::uint64_t  u64TxDataRateMin_;
        std::uint64_t  u64TxDataRateMax_;
        std::uint64_t  u64TxDataRateAvg_;

        uuid_t uuid_;

        NeighborData(NEMId nemId) :
          nemId_{nemId},
          u64LastRxSeqNum_{},
          bHaveEverHadRxActivity_{},
          lastRxTime_{},
          lastTxTime_{},
          u64NumRxFrames_{},
          u64NumRxMissedFrames_{},
          u64NumTxFrames_{},
          rxUtilizationMicroseconds_{},
          fSINRSum_{},
          fSINRSum2_{},
          fNoiseFloorSum_{},
          fNoiseFloorSum2_{},
          u64RxDataRateMin_{},
          u64RxDataRateMax_{},
          u64RxDataRateAvg_{},
          u64TxDataRateMin_{},
          u64TxDataRateMax_{},
          u64TxDataRateAvg_{}
        {
          uuid_clear(uuid_);
        }
      };


    using NeighborDataMap = std::map<EMANE::NEMId, NeighborData *>;

    // <short term, long term>
    using NeighborDataPair = std::pair<NeighborData *, NeighborData *>;

    using NeighborDataPairMap = std::map<EMANE::NEMId, NeighborDataPair>;
  
    using Counter = std::uint64_t;

    EMANE::NEMId nemId_;

    // neighbor metric data used for r2ri reports, reset after each report
    NeighborDataMap r2riMetricTable_;

    // neighbor data used for short and long term statistics
    NeighborDataPairMap neighborDataTable_;

    Microseconds neighborDeleteAgeMicroseconds_;

    // short term table
    StatisticTable<NEMId> * pStatisticNeighborStatusTable_;

    // long term table
    StatisticTable<NEMId> * pStatisticNeighborMetricTable_;


    EMANE::TimePoint lastNeighborStatusUpdateTime_;


    NeighborData * lookupR2RIMetric(NEMId nemId)
      {
        auto iter = r2riMetricTable_.find(nemId);

        if(iter == r2riMetricTable_.end())
         {
            // add the report data for this nem
            iter = r2riMetricTable_.insert(std::make_pair(nemId, new NeighborData{nemId})).first;
         }

        return iter->second;
      }


    NeighborDataPair lookupNeighborData(NEMId nemId)
      {
        auto iter = neighborDataTable_.find(nemId);

        if(iter == neighborDataTable_.end())
          {
            // add the data for this nem
            iter = neighborDataTable_.insert(std::make_pair(nemId, 
                     NeighborDataPair{new NeighborData{nemId}, new NeighborData{nemId}})).first;

            // setup the NeighborMetrics Table
            // first column is nemid
            std::vector<Any> v1{Any{nemId}};

            // next 4 are counters
            v1.insert(v1.end(), 4, Any{Counter{}});
 
            // the rest are floats
            v1.insert(v1.end(), sNeighborMetricLables.size() - 5, Any{float{}});

            // add the row for this nem 
            pStatisticNeighborMetricTable_->addRow(nemId, v1);


            // setup the NeighborStatus Table
            // first column is nemid
            std::vector<Any> v2{Any{nemId}};

            // next 3 are counters
            v2.insert(v2.end(), 3, Any{Counter{}});
 
            // the rest are floats
            v2.insert(v2.end(), sNeighborStatusLables.size() - 4, Any{float{}});

            // add the row for this nem 
            pStatisticNeighborStatusTable_->addRow(nemId, v2);
          }

         return iter->second;
      }


    void handleR2RITxActivity(NEMId dst, std::uint64_t u64DataRatebps, const TimePoint & txTime)
      {
        updateTxActivityData_i(lookupR2RIMetric(dst), u64DataRatebps, txTime);
      }


    void handleNeighborTxActivity(NEMId dst, std::uint64_t u64DataRatebps, const TimePoint & txTime)
      {
        auto neighborDataPair = lookupNeighborData(dst);

        // update the data
        updateTxActivityData_i(neighborDataPair.first, u64DataRatebps, txTime);

        updateTxActivityData_i(neighborDataPair.second, u64DataRatebps, txTime);

        // update the long term stats
        updateTxActivityNeighborMetricStatistics_i(neighborDataPair.second);
      }


    void handleR2RIRxActivity(NEMId src, 
                              std::uint64_t u64SeqNum,
                              const uuid_t & uuid,
                              const EMANE::TimePoint & rxTime)
      {
        updateRxActivityData_i(lookupR2RIMetric(src), u64SeqNum, uuid, rxTime);
      }


    void handleR2RIRxActivity(NEMId src, 
                              std::uint64_t u64SeqNum,
                              const uuid_t & uuid,
                              float fSINR,
                              float fNoiseFloor,
                              const EMANE::TimePoint & rxTime,
                              const EMANE::Microseconds & durationMicroseconds,
                              std::uint64_t u64DataRatebps)
      {
        NeighborData * pNeighborMetric = lookupR2RIMetric(src);

        updateRxActivityData_i(pNeighborMetric, u64SeqNum, uuid, rxTime);

        updateRxActivityChannelData_i(pNeighborMetric, fSINR, fNoiseFloor, durationMicroseconds, u64DataRatebps);
      }


    void updateNeighborRxActivity(NEMId src, 
                                  std::uint64_t u64SeqNum,
                                  const uuid_t & uuid,
                                  const EMANE::TimePoint & rxTime)
      {
         auto neighborDataPair = lookupNeighborData(src);

         // update the data
         updateRxActivityData_i(neighborDataPair.first, u64SeqNum, uuid, rxTime);

         updateRxActivityData_i(neighborDataPair.second, u64SeqNum, uuid, rxTime);

         // update the long term stats
         updateRxActivityNeighborMetricStatistics_i(neighborDataPair.second);
      }



    void updateNeighborRxActivity(NEMId src, 
                                  std::uint64_t u64SeqNum,
                                  const uuid_t & uuid,
                                  float fSINR,
                                  float fNoiseFloor,
                                  const EMANE::TimePoint & rxTime,
                                  const EMANE::Microseconds & durationMicroseconds,
                                  std::uint64_t u64DataRatebps)
      {
        auto neighborDataPair = lookupNeighborData(src);

        // update the data
        updateRxActivityData_i(neighborDataPair.first, u64SeqNum, uuid, rxTime);

        updateRxActivityData_i(neighborDataPair.second, u64SeqNum, uuid, rxTime);

        updateRxActivityChannelData_i(neighborDataPair.first, fSINR, fNoiseFloor, durationMicroseconds, u64DataRatebps);

        updateRxActivityChannelData_i(neighborDataPair.second, fSINR, fNoiseFloor, durationMicroseconds, u64DataRatebps);

        // update the long term stats
        updateRxActivityNeighborMetricStatistics_i(neighborDataPair.second);

        updateRxActivityChannelStatistics_i(neighborDataPair.second);
      }



    void updateTxActivityData_i(NeighborData * pNeighborMetric, std::uint64_t u64DataRatebps, const TimePoint & txTime)
      {
        pNeighborMetric->u64NumTxFrames_ += 1;

        pNeighborMetric->lastTxTime_ = txTime;

        updateRunningAverage(pNeighborMetric->u64TxDataRateAvg_, 
                             pNeighborMetric->u64NumTxFrames_, 
                             u64DataRatebps);

        updateMinMax(pNeighborMetric->u64TxDataRateMin_, pNeighborMetric->u64TxDataRateMax_, u64DataRatebps);
      }





    void updateTxActivityNeighborMetricStatistics_i(NeighborData * pNeighborMetric)
      {
        StatisticTable<NEMId> * pTable = pStatisticNeighborMetricTable_;

        // set statistic(s)
        pTable->setCell(pNeighborMetric->nemId_, 
                         NBR_METRIC_TX_FRAMES, 
                         Any{pNeighborMetric->u64NumTxFrames_});

        pTable->setCell(pNeighborMetric->nemId_, 
                         NBR_METRIC_TX_DATARATE_AVG, 
                         Any{pNeighborMetric->u64TxDataRateAvg_});

        pTable->setCell(pNeighborMetric->nemId_, 
                         NBR_METRIC_LAST_TX_TIME, 
                         Any{std::chrono::duration_cast<DoubleSeconds>(
                         pNeighborMetric->lastTxTime_.time_since_epoch()).count()});
      }


     void updateRxActivityData_i(NeighborData * pNeighborMetric, 
                                 std::uint64_t u64SeqNum, 
                                 const uuid_t & uuid,
                                 const EMANE::TimePoint & rxTime)
      {
        if(!uuid_compare(uuid,pNeighborMetric->uuid_))
          {
            if(u64SeqNum > pNeighborMetric->u64LastRxSeqNum_)
              {
                pNeighborMetric->u64NumRxMissedFrames_ += 
                  u64SeqNum - pNeighborMetric->u64LastRxSeqNum_ - 1;
                
                pNeighborMetric->u64LastRxSeqNum_ = u64SeqNum;
              }
            else
              {
                // out of order, previously accounted for as missed
                if(pNeighborMetric->u64NumRxMissedFrames_)
                  {
                    pNeighborMetric->u64NumRxMissedFrames_ -= 1;
                  }
              }
          }
        else
          {
            // first packet or emulator restart
            uuid_copy(pNeighborMetric->uuid_,uuid);
            
            pNeighborMetric->u64LastRxSeqNum_ = u64SeqNum;
          }
        
        pNeighborMetric->u64NumRxFrames_ += 1;
        
        pNeighborMetric->lastRxTime_ = rxTime;
        
        pNeighborMetric->bHaveEverHadRxActivity_ = true;
      }


    void updateRxActivityNeighborMetricStatistics_i(NeighborData * pNeighborMetric)
      {
        StatisticTable<NEMId> * pTable = pStatisticNeighborMetricTable_;

        // update statistic(s)
        pTable->setCell(pNeighborMetric->nemId_, 
                         NBR_METRIC_RX_FRAMES, 
                         Any{pNeighborMetric->u64NumRxFrames_});

        pTable->setCell(pNeighborMetric->nemId_, 
                         NBR_METRIC_MISSED_FRAMES, 
                         Any{pNeighborMetric->u64NumRxMissedFrames_});
      }


    void updateRxActivityChannelData_i(NeighborData * pNeighborMetric,
                                   float fSINR,
                                   float fNoiseFloor,
                                   const Microseconds & durationMicroseconds,
                                   std::uint64_t u64DataRatebps)
      {
        pNeighborMetric->rxUtilizationMicroseconds_ += durationMicroseconds;

        pNeighborMetric->fSINRSum_  += fSINR;
        pNeighborMetric->fSINRSum2_ += (fSINR * fSINR);

        pNeighborMetric->fNoiseFloorSum_  += fNoiseFloor;
        pNeighborMetric->fNoiseFloorSum2_ += (fNoiseFloor * fNoiseFloor);

        updateMinMax(pNeighborMetric->u64RxDataRateMin_, pNeighborMetric->u64RxDataRateMax_, u64DataRatebps);

        updateRunningAverage(pNeighborMetric->u64RxDataRateAvg_, pNeighborMetric->u64NumRxFrames_, u64DataRatebps);
      }


    void updateRxActivityChannelStatistics_i(NeighborData * pNeighborMetric)
      {
        float fSINRAvg{}, fSINRStdv{}, fNoiseFloorAvg{}, fNoiseFloorStdv{};

        // get sinr avg and standard deviation 
        getAvgAndStd(pNeighborMetric->fSINRSum_, 
                     pNeighborMetric->fSINRSum2_, 
                     pNeighborMetric->u64NumRxFrames_, 
                     fSINRAvg,
                     fSINRStdv);

        // get noise floor avg and standard deviation 
        getAvgAndStd(pNeighborMetric->fNoiseFloorSum_, 
                     pNeighborMetric->fNoiseFloorSum2_, 
                     pNeighborMetric->u64NumRxFrames_, 
                     fNoiseFloorAvg,
                     fNoiseFloorStdv);

        StatisticTable<NEMId> * pTable = pStatisticNeighborMetricTable_;

        // update statistic(s)
        pTable->setCell(pNeighborMetric->nemId_, 
                         NBR_METRIC_BW_CONSUMPTION, 
                         Any{pNeighborMetric->rxUtilizationMicroseconds_.count()});

        pTable->setCell(pNeighborMetric->nemId_, 
                         NBR_METRIC_LAST_RX_TIME, 
                         Any{std::chrono::duration_cast<DoubleSeconds>(
                         pNeighborMetric->lastRxTime_.time_since_epoch()).count()});

        pTable->setCell(pNeighborMetric->nemId_, 
                         NBR_METRIC_SINR_AVG, 
                         Any{fSINRAvg});

        pTable->setCell(pNeighborMetric->nemId_, 
                         NBR_METRIC_SINR_STDV, 
                         Any{fSINRStdv});

        pTable->setCell(pNeighborMetric->nemId_, 
                         NBR_METRIC_NOISE_FLOOR_AVG, 
                         Any{fNoiseFloorAvg});

        pTable->setCell(pNeighborMetric->nemId_, 
                         NBR_METRIC_NOISE_FLOOR_STDV, 
                         Any{fNoiseFloorStdv});

        pTable->setCell(pNeighborMetric->nemId_, 
                         NBR_METRIC_RX_DATARATE_AVG, 
                         Any{pNeighborMetric->u64RxDataRateAvg_});
      }


    void clearData_i(NeighborData * pNeighborMetric)
      {
         // clear just about everything but the lastRx seq, time and activity flag
        
         // reset missed frames
         pNeighborMetric->u64NumRxMissedFrames_ = 0;
  
         // reset rx/tx frames
         pNeighborMetric->u64NumRxFrames_ = 0;
         pNeighborMetric->u64NumTxFrames_ = 0;

         // reset bw consumption
         pNeighborMetric->rxUtilizationMicroseconds_ = EMANE::Microseconds::zero();

         // reset the SINR
         pNeighborMetric->fSINRSum_  = 0.0f;
         pNeighborMetric->fSINRSum2_ = 0.0f;

         // reset the noise floor
         pNeighborMetric->fNoiseFloorSum_  = 0.0f;
         pNeighborMetric->fNoiseFloorSum2_ = 0.0f;

         // reset data rate 
         pNeighborMetric->u64RxDataRateMin_ = 0;
         pNeighborMetric->u64RxDataRateMax_ = 0;
         pNeighborMetric->u64RxDataRateAvg_ = 0;

         pNeighborMetric->u64TxDataRateMin_ = 0;
         pNeighborMetric->u64TxDataRateMax_ = 0;
         pNeighborMetric->u64TxDataRateAvg_ = 0;
      }


    float getAvg(float fSum, size_t numSamples)
      {
        if(numSamples > 1)
          {
            return fSum / numSamples;
          }
        else if(numSamples == 1)
          {
             return fSum;
          }
        else
          {
             return  0.0f;
          }
      }


    void getAvgAndStd(float fSum, float fSumSquared, size_t numSamples, float & fAvg, float & fStd)
      {
        fAvg = getAvg(fSum, numSamples);

        if(numSamples > 1)
          {
            float fDelta{fSumSquared - (fSum * fSum)};

            if(fDelta > 0.0f)
              {
                 fStd = sqrt(fDelta / numSamples) / (numSamples - 1.0f);
              }
          }
      }

    inline void updateMinMax(std::uint64_t & min, std::uint64_t & max, std::uint64_t value)
      {
        if(min > value || min == 0)
          {
            min = value;
          }

        if(max < value || max == 0)
          {
            max = value;
          }
      }

    // set the running avg,  based on count, and new value
    inline void updateRunningAverage(std::uint64_t &avg, std::uint64_t count, std::uint64_t val)
     {
       avg = (count == 1) ? val : ((avg * count) + val) / (count + 1);
     }
 };



EMANE::NeighborMetricManager::NeighborMetricManager(EMANE::NEMId nemId) :
  pImpl_{new Implementation{nemId}}
{ }


EMANE::NeighborMetricManager::~NeighborMetricManager()
{ }


void
EMANE::NeighborMetricManager::setNeighborDeleteTimeMicroseconds(const Microseconds & ageMicroseconds)
{
  pImpl_->setNeighborDeleteTimeMicroseconds(ageMicroseconds);
}


void 
EMANE::NeighborMetricManager::updateNeighborTxMetric(NEMId dst, std::uint64_t u64DataRatebps, const TimePoint & txTime)
{
  // tx activity
  pImpl_->handleTxActivity(dst, u64DataRatebps, txTime);
}


void 
EMANE::NeighborMetricManager::updateNeighborRxMetric(NEMId src, 
                                                     std::uint64_t u64SeqNum, 
                                                     const uuid_t & uuid,
                                                     const TimePoint & rxTime)
{
  // rx activity (short form)
  pImpl_->handleRxActivity(src, u64SeqNum, uuid, rxTime);
}


void EMANE::NeighborMetricManager::updateNeighborRxMetric(NEMId src, 
                                                          std::uint64_t u64SeqNum,
                                                          const uuid_t & uuid,
                                                          float fSINR,
                                                          float fNoiseFloor,
                                                          const TimePoint & rxTime,
                                                          const Microseconds & durationMicroseconds,
                                                          std::uint64_t u64DataRatebps)
{
   // rx activity (short form)
   pImpl_->handleRxActivity(src, 
                            u64SeqNum,
                            uuid,
                            fSINR,
                            fNoiseFloor,
                            rxTime,
                            durationMicroseconds,
                            u64DataRatebps);
}


EMANE::Controls::R2RINeighborMetrics
EMANE::NeighborMetricManager::getNeighborMetrics()
{
   // when neighbor metrics are pulled we update the short term statistics too
   pImpl_->handleNeighborStatusUpdate();

   return pImpl_->getNeighborMetrics();
}


void EMANE::NeighborMetricManager::updateNeighborStatus()
{
   pImpl_->handleNeighborStatusUpdate();
}


void EMANE::NeighborMetricManager::registerStatistics(StatisticRegistrar & statisticRegistrar)
{
  pImpl_->registerStatistics(statisticRegistrar);
}

