/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
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
  static const std::uint16_t MAX_SEQ_NUM = 0xffff;

  const EMANE::StatisticTableLabels sNeighborMetricLables {"NEM",
                                                           "Rx Pkts", 
                                                           "Tx Pkts", 
                                                           "Missed Pkts", 
                                                           "BW Utilization", 
                                                           "Last Rx Time",
                                                           "Last Tx Time",
                                                           "SINR Avg", 
                                                           "SINR Stdv", 
                                                           "NF Avg", 
                                                           "NF Stdv", 
                                                           "Rx Rate Avg", 
                                                           "Tx Rate Avg"};
}

class EMANE::NeighborMetricManager::Implementation
  {
    public:
      Implementation(EMANE::NEMId nemId) :
       nemId_{nemId}
      { }

     ~Implementation()
      { }


    void setNeighborDeleteTimeMicroseconds(const Microseconds & ageMicroseconds)
      {
        std::lock_guard<std::mutex> m(reportDataMutex_);

        neighborDeleteAgeMicroseconds_ = ageMicroseconds;
      }


    void updateNeighborTxMetric(NEMId dst, std::uint64_t u64DataRatebps, const TimePoint & txTime)
      {
        // exclude tx broadcast from the reports
        if(dst != EMANE::NEM_BROADCAST_MAC_ADDRESS)
          {
            updateNeighborReportTxMetric(dst, u64DataRatebps, txTime);
          }

        updateNeighborStatisticTxMetric(dst, u64DataRatebps, txTime);
      }


    void updateNeighborRxMetric(NEMId src, 
                                std::uint16_t u16SeqNum, 
                                const EMANE::TimePoint & rxTime)
      {
        updateNeighborReportRxMetric(src, u16SeqNum, rxTime);

        updateNeighborStatisticRxMetric(src, u16SeqNum, rxTime);
      }



    void updateNeighborRxMetric(NEMId src, 
                                std::uint16_t u16SeqNum,
                                float fSINR,
                                float fNoiseFloor,
                                const EMANE::TimePoint & rxTime,
                                const EMANE::Microseconds & durationMicroseconds,
                                std::uint64_t u64DataRatebps)
      {
        updateReportRxMetric(src, 
                             u16SeqNum, 
                             fSINR, 
                             fNoiseFloor,
                             rxTime,
                             durationMicroseconds,
                             u64DataRatebps);

        updateStatisticRxMetric(src, 
                                u16SeqNum, 
                                fSINR, 
                                fNoiseFloor,
                                rxTime,
                                durationMicroseconds,
                                u64DataRatebps);
      }



     EMANE::Controls::R2RINeighborMetrics getNeighborMetrics()
      {
        std::lock_guard<std::mutex> m(reportDataMutex_);

        EMANE::TimePoint currentTime{Clock::now()};

        EMANE::Controls::R2RINeighborMetrics neighborMetrics;

        for(auto iter = neighborDataReportMap_.begin(); iter != neighborDataReportMap_.end(); /* bump/erase below */)
          {
            // get the age
            const Microseconds ageMicroseconds{
              std::chrono::duration_cast<Microseconds>(currentTime - iter->second.lastRxTime_)};  
     
            if(ageMicroseconds > neighborDeleteAgeMicroseconds_)
              {
                // erase and bump
                neighborDataReportMap_.erase(iter++);
              }
            else
              {
                float fSINRAvg{}, fSINRStd{}, fNoiseFloorAvg{}, fNoiseFloorStdv{};

                // get sinr avg and standard deviation 
                getAvgAndStd(iter->second.fSINRSum_, 
                             iter->second.fSINRSum2_, 
                             iter->second.u32NumRxFrames_, 
                             fSINRAvg,
                             fSINRStd);

                // get noise floor avg and standard deviation 
                getAvgAndStd(iter->second.fNoiseFloorSum_, 
                             iter->second.fNoiseFloorSum2_, 
                             iter->second.u32NumRxFrames_, 
                             fNoiseFloorAvg,
                             fNoiseFloorStdv);

                EMANE::Controls::R2RINeighborMetric 
                  neighborMetric{iter->first,                             // nbr id
                                 iter->second.u32NumRxFrames_,            // num rx frames (samples)
                                 iter->second.u32NumTxFrames_,            // num tx frames (samples)
                                 iter->second.u32NumRxMissedFrames_,      // num rx missed frames
                                 iter->second.rxUtilizationMicroseconds_, // bandwidth consumption
                                 fSINRAvg,                                // sinr avg 
                                 fSINRStd,                                // sinr std deviation
                                 fNoiseFloorAvg,                          // noisefloor avg
                                 fNoiseFloorStdv,                         // noise floor std deviation
                                 iter->second.u64RxDataRateAvg_,          // avg rx datarate
                                 iter->second.u64TxDataRateAvg_};         // avg tx datarate

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
                                                  [this](StatisticTablePublisher * pTable)
                                                  {
                                                    std::lock_guard<std::mutex> m(statisticTableMutex_);
                                                    for(auto & iter : neighborDataStatisticMap_)
                                                      {
                                                        clearData_i(iter.second);
                                                      }
                                                    pTable->clear();
                                                  },
                                                  "Neighbor Metric Table");
      }


    private:
      // keep this is line with the NeighborMetricLabels
      enum NeighborMetricLables { RX_FRAMES = 1,
                                  TX_FRAMES = 2, 
                                  MISSED_FRAMES = 3, 
                                  BW_CONSUMPTION = 4,
                                  LAST_RX_TIME = 5,
                                  LAST_TX_TIME = 6,
                                  SINR_AVG = 7, 
                                  SINR_STDV = 8, 
                                  NOISE_FLOOR_AVG = 9, 
                                  NOISE_FLOOR_STDV = 10,
                                  RX_DATARATE_AVG = 11, 
                                  TX_DATARATE_AVG = 12 };

      struct NeighborData {
        const NEMId      nemId_;

        bool bCleared_;

        std::uint16_t    u16LastRxSeqNum_;

        EMANE::TimePoint lastRxTime_;
        EMANE::TimePoint lastTxTime_;

        std::uint32_t    u32NumRxFrames_;
        std::uint32_t    u32NumRxMissedFrames_;
        std::uint32_t    u32NumTxFrames_;

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

        NeighborData(NEMId nemId) :
          nemId_{nemId},
          bCleared_{true},
          u16LastRxSeqNum_{},
          lastRxTime_{},
          lastTxTime_{},
          u32NumRxFrames_{},
          u32NumRxMissedFrames_{},
          u32NumTxFrames_{},
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
        { }
      };


    using NeighborDataMap = std::map<EMANE::NEMId, NeighborData>;
  
    using Counter = std::uint64_t;

    EMANE::NEMId nemId_;

    // neighbor metric data used for reports, cleared after each report
    NeighborDataMap neighborDataReportMap_;

    // neighbor metric data used for statistics, may or may not be cleared
    NeighborDataMap neighborDataStatisticMap_;

    Microseconds neighborDeleteAgeMicroseconds_;

    std::mutex reportDataMutex_;

    std::mutex statisticTableMutex_;

    StatisticTable<NEMId> * pStatisticNeighborMetricTable_;


    NeighborData & lookupReportMetric_i(NEMId nemId)
      {
        auto iter = neighborDataReportMap_.find(nemId);

        if(iter == neighborDataReportMap_.end())
         {
            // add the report data for this nem
            iter = neighborDataReportMap_.insert(std::make_pair(nemId, NeighborData(nemId))).first;
         }

        iter->second.bCleared_ = false;

        return iter->second;
      }


    NeighborData & lookupStatisticMetric_i(NEMId nemId)
      {
        auto iter = neighborDataStatisticMap_.find(nemId);

        if(iter == neighborDataStatisticMap_.end() || iter->second.bCleared_)
          {
            if(iter == neighborDataStatisticMap_.end())
              {
                // add the statistic data for this nem
                iter = neighborDataStatisticMap_.insert(std::make_pair(nemId, NeighborData(nemId))).first;
              }

            // first column is nemid
            std::vector<Any> v{Any{nemId}};

            // next 4 are counters
            v.insert(v.end(), 4, Any{Counter{}});
 
            // the rest are floats
            v.insert(v.end(), sNeighborMetricLables.size() - 5, Any{float{}});

            // add the row for this nem 
            pStatisticNeighborMetricTable_->addRow(nemId, v);
          }

        iter->second.bCleared_ = false;

        return iter->second;
      }


    void updateNeighborReportTxMetric(NEMId dst, std::uint64_t u64DataRatebps, const TimePoint & txTime)
      {
        std::lock_guard<std::mutex> m(reportDataMutex_);

        NeighborData & rNeighborMetric = lookupReportMetric_i(dst);

        updateReportTxActivity_i(rNeighborMetric, u64DataRatebps, txTime);
      }


    void updateNeighborStatisticTxMetric(NEMId dst, std::uint64_t u64DataRatebps, const TimePoint & txTime)
      {
        std::lock_guard<std::mutex> m(statisticTableMutex_);

        NeighborData & rNeighborMetric = lookupStatisticMetric_i(dst);

        updateStatisticTxActivity_i(rNeighborMetric, u64DataRatebps, txTime);
      }


    void updateNeighborReportRxMetric(NEMId src, 
                                      std::uint16_t u16SeqNum, 
                                      const EMANE::TimePoint & rxTime)
      {
         std::lock_guard<std::mutex> m(reportDataMutex_);

         NeighborData & rNeighborMetric = lookupReportMetric_i(src);

         updateReportRxActivity_i(rNeighborMetric, u16SeqNum, rxTime);
      }


    void updateNeighborStatisticRxMetric(NEMId src, 
                                         std::uint16_t u16SeqNum, 
                                         const EMANE::TimePoint & rxTime)
      {
         std::lock_guard<std::mutex> m(statisticTableMutex_);

         NeighborData & rNeighborMetric = lookupStatisticMetric_i(src);

         updateStatisticRxActivity_i(rNeighborMetric, u16SeqNum, rxTime);
      }


    void updateReportRxMetric(NEMId src, 
                              std::uint16_t u16SeqNum,
                              float fSINR,
                              float fNoiseFloor,
                              const EMANE::TimePoint & rxTime,
                              const EMANE::Microseconds & durationMicroseconds,
                              std::uint64_t u64DataRatebps)
      {
        std::lock_guard<std::mutex> m(reportDataMutex_);

        NeighborData & rNeighborMetric = lookupReportMetric_i(src);

        updateReportRxActivity_i(rNeighborMetric, u16SeqNum, rxTime);

        updateReportChannelRxActivity_i(rNeighborMetric, 
                                        fSINR, 
                                        fNoiseFloor, 
                                        durationMicroseconds, 
                                        u64DataRatebps);
      }


    void updateStatisticRxMetric(NEMId src, 
                                 std::uint16_t u16SeqNum,
                                 float fSINR,
                                 float fNoiseFloor,
                                 const EMANE::TimePoint & rxTime,
                                 const EMANE::Microseconds & durationMicroseconds,
                                 std::uint64_t u64DataRatebps)
      {
        std::lock_guard<std::mutex> m(statisticTableMutex_);

        NeighborData & rNeighborMetric = lookupStatisticMetric_i(src);

        updateStatisticRxActivity_i(rNeighborMetric, u16SeqNum, rxTime);

        updateStatisticChannelRxActivity_i(rNeighborMetric, 
                                           fSINR, 
                                           fNoiseFloor, 
                                           durationMicroseconds, 
                                           u64DataRatebps);
      }




    void updateTxActivity_i(NeighborData & rNeighborMetric, std::uint64_t u64DataRatebps, const TimePoint & txTime)
      {
        rNeighborMetric.u32NumTxFrames_ += 1;

        rNeighborMetric.lastTxTime_ = txTime;

        updateRunningAverage(rNeighborMetric.u64TxDataRateAvg_, 
                             rNeighborMetric.u32NumTxFrames_, 
                             u64DataRatebps);

        updateMinMax(rNeighborMetric.u64TxDataRateMin_, rNeighborMetric.u64TxDataRateMax_, u64DataRatebps);
      }


    void updateReportTxActivity_i(NeighborData & rNeighborMetric, std::uint64_t u64DataRatebps, const TimePoint & txTime)
      {
        updateTxActivity_i(rNeighborMetric, u64DataRatebps, txTime);
      }


    void updateStatisticTxActivity_i(NeighborData & rNeighborMetric, std::uint64_t u64DataRatebps, const TimePoint & txTime)
      {
        updateTxActivity_i(rNeighborMetric, u64DataRatebps, txTime);

        // set statistic(s)
        pStatisticNeighborMetricTable_->setCell(rNeighborMetric.nemId_, 
                                                TX_FRAMES, 
                                                Any{rNeighborMetric.u32NumTxFrames_});

        pStatisticNeighborMetricTable_->setCell(rNeighborMetric.nemId_, 
                                                TX_DATARATE_AVG, 
                                                Any{rNeighborMetric.u64TxDataRateAvg_});

        pStatisticNeighborMetricTable_->setCell(rNeighborMetric.nemId_, 
                                                LAST_TX_TIME, 
                                                Any{std::chrono::duration_cast<DoubleSeconds>(
                                                  rNeighborMetric.lastTxTime_.time_since_epoch()).count()});
      }


     void updateRxActivity_i(NeighborData & rNeighborMetric, 
                             std::uint16_t u16SeqNum, 
                             const EMANE::TimePoint & rxTime)
      {
         std::int32_t iSeqDelta{u16SeqNum - rNeighborMetric.u16LastRxSeqNum_};

         // check for roll over
         if(iSeqDelta < 0)
           {
             iSeqDelta = MAX_SEQ_NUM - iSeqDelta;
           }

         // check for missed frames
         if(iSeqDelta > 1)
           {
             rNeighborMetric.u32NumRxMissedFrames_ += (iSeqDelta - 1);
           }

         rNeighborMetric.u32NumRxFrames_ += 1;
 
         rNeighborMetric.u16LastRxSeqNum_ = u16SeqNum;

         rNeighborMetric.lastRxTime_ = rxTime;
      }


   
    void updateReportRxActivity_i(NeighborData & rNeighborMetric, 
                                  std::uint16_t u16SeqNum, 
                                  const EMANE::TimePoint & rxTime)
      {
        updateRxActivity_i(rNeighborMetric, u16SeqNum, rxTime);
      }


    void updateStatisticRxActivity_i(NeighborData & rNeighborMetric, 
                                    std::uint16_t u16SeqNum, 
                                    const EMANE::TimePoint & rxTime)
      {
        updateRxActivity_i(rNeighborMetric, u16SeqNum, rxTime);

        // update statistic(s)
        pStatisticNeighborMetricTable_->setCell(rNeighborMetric.nemId_, 
                                                RX_FRAMES, 
                                                Any{rNeighborMetric.u32NumRxFrames_});

        pStatisticNeighborMetricTable_->setCell(rNeighborMetric.nemId_, 
                                                MISSED_FRAMES, 
                                                Any{rNeighborMetric.u32NumRxMissedFrames_});
      }


    void updateChannelRxActivity_i(NeighborData & rNeighborMetric,
                                   float fSINR,
                                   float fNoiseFloor,
                                   const Microseconds & durationMicroseconds,
                                   std::uint64_t u64DataRatebps)
      {
        rNeighborMetric.rxUtilizationMicroseconds_ += durationMicroseconds;

        rNeighborMetric.fSINRSum_  += fSINR;
        rNeighborMetric.fSINRSum2_ += (fSINR * fSINR);

        rNeighborMetric.fNoiseFloorSum_  += fNoiseFloor;
        rNeighborMetric.fNoiseFloorSum2_ += (fNoiseFloor * fNoiseFloor);

        updateMinMax(rNeighborMetric.u64RxDataRateMin_, rNeighborMetric.u64RxDataRateMax_, u64DataRatebps);

        updateRunningAverage(rNeighborMetric.u64RxDataRateAvg_, rNeighborMetric.u32NumRxFrames_, u64DataRatebps);
      }


    void updateReportChannelRxActivity_i(NeighborData & rNeighborMetric,
                                         float fSINR,
                                         float fNoiseFloor,
                                         const Microseconds & durationMicroseconds,
                                         std::uint64_t u64DataRatebps)
      {
        updateChannelRxActivity_i(rNeighborMetric, fSINR, fNoiseFloor, durationMicroseconds, u64DataRatebps);
      }


    void updateStatisticChannelRxActivity_i(NeighborData & rNeighborMetric,
                                            float fSINR,
                                            float fNoiseFloor,
                                            const Microseconds & durationMicroseconds,
                                            std::uint64_t u64DataRatebps)
      {
        updateChannelRxActivity_i(rNeighborMetric, fSINR, fNoiseFloor, durationMicroseconds, u64DataRatebps);

        float fSINRAvg{}, fSINRStdv{}, fNoiseFloorAvg{}, fNoiseFloorStdv{};

        // get sinr avg and standard deviation 
        getAvgAndStd(rNeighborMetric.fSINRSum_, 
                     rNeighborMetric.fSINRSum2_, 
                     rNeighborMetric.u32NumRxFrames_, 
                     fSINRAvg,
                     fSINRStdv);

        // get noise floor avg and standard deviation 
        getAvgAndStd(rNeighborMetric.fNoiseFloorSum_, 
                     rNeighborMetric.fNoiseFloorSum2_, 
                     rNeighborMetric.u32NumRxFrames_, 
                     fNoiseFloorAvg,
                     fNoiseFloorStdv);


        // update statistic(s)
        pStatisticNeighborMetricTable_->setCell(rNeighborMetric.nemId_, 
                                                BW_CONSUMPTION, 
                                                Any{rNeighborMetric.rxUtilizationMicroseconds_.count()});

        pStatisticNeighborMetricTable_->setCell(rNeighborMetric.nemId_, 
                                                LAST_RX_TIME, 
                                                Any{std::chrono::duration_cast<DoubleSeconds>(
                                                  rNeighborMetric.lastRxTime_.time_since_epoch()).count()});

        pStatisticNeighborMetricTable_->setCell(rNeighborMetric.nemId_, 
                                                SINR_AVG, 
                                                Any{fSINRAvg});

        pStatisticNeighborMetricTable_->setCell(rNeighborMetric.nemId_, 
                                                SINR_STDV, 
                                                Any{fSINRStdv});

        pStatisticNeighborMetricTable_->setCell(rNeighborMetric.nemId_, 
                                                NOISE_FLOOR_AVG, 
                                                Any{fNoiseFloorAvg});

        pStatisticNeighborMetricTable_->setCell(rNeighborMetric.nemId_, 
                                                NOISE_FLOOR_STDV, 
                                                Any{fNoiseFloorStdv});

        pStatisticNeighborMetricTable_->setCell(rNeighborMetric.nemId_, 
                                                RX_DATARATE_AVG, 
                                                Any{rNeighborMetric.u64RxDataRateAvg_});
      }


    void clearData_i(NeighborData & rNeighborMetric)
      {
         // clear just about everything but the lastRx seq and time
        
         // reset missed frames
         rNeighborMetric.u32NumRxMissedFrames_ = 0;
  
         // reset rx/tx frames
         rNeighborMetric.u32NumRxFrames_ = 0;
         rNeighborMetric.u32NumTxFrames_ = 0;

         // reset bw consumption
         rNeighborMetric.rxUtilizationMicroseconds_ = EMANE::Microseconds::zero();

         // reset the SINR
         rNeighborMetric.fSINRSum_  = 0.0f;
         rNeighborMetric.fSINRSum2_ = 0.0f;

         // reset the noise floor
         rNeighborMetric.fNoiseFloorSum_  = 0.0f;
         rNeighborMetric.fNoiseFloorSum2_ = 0.0f;

         // reset data rate 
         rNeighborMetric.u64RxDataRateMin_ = 0;
         rNeighborMetric.u64RxDataRateMax_ = 0;
         rNeighborMetric.u64RxDataRateAvg_ = 0;

         rNeighborMetric.u64TxDataRateMin_ = 0;
         rNeighborMetric.u64TxDataRateMax_ = 0;
         rNeighborMetric.u64TxDataRateAvg_ = 0;

         rNeighborMetric.bCleared_ = true;
      }


    // assumes in/out variable are initialized by the caller, so only set values as needed
    void getAvgAndStd(float fSum, float fSumSquared, size_t numSamples, float & fAvg, float & fStd)
      {
        if(numSamples > 1)
          {
            fAvg = fSum / numSamples;

            const float fDelta{fSumSquared - (fSum * fSum)};

            if(fDelta > 0.0f)
              {
                 fStd = sqrt(fDelta / numSamples) / (numSamples - 1.0f);
              }
          }
        else if(numSamples == 1)
          {
             fAvg = fSum;
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
    inline void updateRunningAverage(std::uint64_t &avg, std::uint32_t count, std::uint64_t val)
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
  pImpl_->updateNeighborTxMetric(dst, u64DataRatebps, txTime);
}


void 
EMANE::NeighborMetricManager::updateNeighborRxMetric(NEMId src, 
                                                     std::uint16_t u16SeqNum, 
                                                     const TimePoint & rxTime)
{
  pImpl_->updateNeighborRxMetric(src, u16SeqNum, rxTime);
}


void EMANE::NeighborMetricManager::updateNeighborRxMetric(NEMId src, 
                                                          std::uint16_t u16SeqNum,
                                                          float fSINR,
                                                          float fNoiseFloor,
                                                          const TimePoint & rxTime,
                                                          const Microseconds & durationMicroseconds,
                                                          std::uint64_t u64DataRatebps)
{
   pImpl_->updateNeighborRxMetric(src, 
                                  u16SeqNum,
                                  fSINR,
                                  fNoiseFloor,
                                  rxTime,
                                  durationMicroseconds,
                                  u64DataRatebps);
}


EMANE::Controls::R2RINeighborMetrics
EMANE::NeighborMetricManager::getNeighborMetrics()
{
   return pImpl_->getNeighborMetrics();
}


void EMANE::NeighborMetricManager::registerStatistics(StatisticRegistrar & statisticRegistrar)
{
  pImpl_->registerStatistics(statisticRegistrar);
}

