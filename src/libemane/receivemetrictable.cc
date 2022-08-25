/*
 * Copyright (c) 2022 - Adjacent Link LLC, Bridgewater, New Jersey
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
#include "emane/receivemetrictable.h"
#include "emane/statistictable.h"
#include "emane/utils/conversionutils.h"

#include <map>
#include <string>
#include <tuple>


namespace {
  const EMANE::StatisticTableLabels sReceiveMetricTableLables {"NEM",           // 0
                                                               "AntennaId",     // 1
                                                               "FrequencyHz",   // 2
                                                               "NumSamples",    // 3
                                                               "AvgRxPower",    // 4
                                                               "AvgNoiseFloor", // 5
                                                               "AvgSINR",       // 6
                                                               "AvgINR"};       // 7

  const EMANE::AntennaIndex AntennaIndexDontCare = 0;
  const std::uint64_t       FrequencyDontCare    = 0;

  // accumulated running average
  inline void getRunningAvg(double &avg, std::uint64_t count, double newValue)
   {
     avg = (count == 1) ? newValue : ((avg * count) + newValue) / (count + 1);
   }
}


class EMANE::ReceiveMetricTable::Implementation
  {
    public:
      Implementation(EMANE::NEMId nemId) :
       nemId_{nemId},
       pStatisticReceiveMetricTable_{},
       bAverageAllAntenna_{},
       bAverageAllFrequencies_{}
       { }
 
      ~Implementation()
       { }

      void initialize(EMANE::Registrar & registrar)
       {
         auto & statisticRegistrar = registrar.statisticRegistrar();

         pStatisticReceiveMetricTable_ =
           statisticRegistrar.registerTable<std::string>("ReceiveMetricTable", 
                                                         sReceiveMetricTableLables,
                                                         StatisticProperties::NONE,
                                                         "Receive Metric Table");


         auto & configRegistrar = registrar.configurationRegistrar();

         configRegistrar.registerNumeric<bool>("rxmetrictable.averageallantennas",
                                               ConfigurationProperties::DEFAULT,
                                               {false},
                                               "Defines whether statistics for all antennas with be averaged together.");

         configRegistrar.registerNumeric<bool>("rxmetrictable.averageallfrequencies",
                                               ConfigurationProperties::DEFAULT,
                                               {false},
                                               "Defines whether statistics for all frequencies with be averaged together.");
       }


      void configure(const EMANE::ConfigurationUpdate & configurationUpdate)
       {
         for(const auto & item : configurationUpdate)
          {
            if(item.first == "rxmetrictable.averageallantennas")
             {
               bAverageAllAntenna_ = item.second[0].asBool();
             }
            else if(item.first == "rxmetrictable.averageallfrequencies")
             {
               bAverageAllFrequencies_ = item.second[0].asBool();
             }
            else
             {
               const std::string sErr = "Unexpected rxmetrictable config " + item.first;

               throw(EMANE::ConfigurationException(sErr));
             }
          }

          // layout a typical table row
          tableRowTemplate_.clear();

          // set these when new entry is added
          // 0 - NEMId
          tableRowTemplate_.push_back(Any{std::uint16_t{}});

          // 1 - AntenaId
          if(bAverageAllAntenna_)
           {
             tableRowTemplate_.push_back(Any{std::string{"NA"}});
           }
          else
           {
             tableRowTemplate_.push_back(Any{std::uint16_t{}});
           }

          // 2 - Frequency
          if(bAverageAllFrequencies_)
           {
             tableRowTemplate_.push_back(Any{std::string{"NA"}});
           }
          else
           {
             tableRowTemplate_.push_back(Any{std::uint64_t{}});
           }

          // set these on each update
          // 3 - Count
          tableRowTemplate_.push_back(Any{std::uint64_t{}});

          // 4 - Avg RxPower
          tableRowTemplate_.push_back(Any{double{}});

          // 5 - Avg Noise
          tableRowTemplate_.push_back(Any{double{}});

          // 6 - Avg SINR
          tableRowTemplate_.push_back(Any{double{}});

          // 7 - Avg INR
          tableRowTemplate_.push_back(Any{double{}});
       }


      void reset(EMANE::AntennaIndex rxAntennaId)
       {
          if(bAverageAllAntenna_)
           {
             // lump all antenna
             rxAntennaId = AntennaIndexDontCare;
           }

          const auto iter = antennaTracker_.find(rxAntennaId);

          if(iter != antennaTracker_.end())
           {
             for(const auto & e : iter->second)
              {
                const auto key = makeKey(std::get<0>(e), rxAntennaId, std::get<1>(e));

                receiveMetricCache_.erase(key);                

                pStatisticReceiveMetricTable_->deleteRow(key);
              }

             antennaTracker_.erase(rxAntennaId);
           }
       }


      void resetAll()
       {
         pStatisticReceiveMetricTable_->clear();

         receiveMetricCache_.clear();

         antennaTracker_.clear();
       }


      void update(NEMId src,
                  EMANE::AntennaIndex rxAntennaId,
                  std::uint64_t frequencyHz,
                  double fRxPower_mW,
                  double fNoiseFloor_mW,
                  double fReceiverSensitivity_mW)
       { 
          // sanity check, we have not been initialized
          if(! pStatisticReceiveMetricTable_)
           {
             return;
           }

          if(bAverageAllAntenna_)
           {
             // lump all antenna(s) into the average
             rxAntennaId = AntennaIndexDontCare;
           }

          if(bAverageAllFrequencies_)
           {
             // lump all freqs(s) into the average
             frequencyHz = FrequencyDontCare;
           }

          // cache and table key
          const auto key = makeKey(src, rxAntennaId, frequencyHz);

          auto iter = receiveMetricCache_.find(key);

          if(iter == receiveMetricCache_.end())
           {
             // new entry
             const auto result = receiveMetricCache_.emplace(key, ReceiveMetricCacheEntry{});

             // insert sanity check
             if(result.second)
              {
                iter = result.first;

                // add row to statistic table
                pStatisticReceiveMetricTable_->addRow(key, tableRowTemplate_);

                // set constant values that represent this key
                pStatisticReceiveMetricTable_->setCell(key, RX_METRIC_TBL_NEMID, Any{src});

                if(! bAverageAllAntenna_)
                 {
                   pStatisticReceiveMetricTable_->setCell(key, RX_METRIC_TBL_ANTENNA_ID, Any{rxAntennaId});
                 }

                if(! bAverageAllFrequencies_)
                 {
                   pStatisticReceiveMetricTable_->setCell(key, RX_METRIC_TBL_FREQUENCY,  Any{frequencyHz});
                 }

                // add src/frequency to the antenna tracker
                antennaTracker_[rxAntennaId].emplace(std::tuple<EMANE::NEMId, std::uint64_t>{src, frequencyHz});
              }
             else
              {
                // not an expected condition
                return;
              }
           }

          // update our cache
          const auto tableData = iter->second.update(fRxPower_mW, fNoiseFloor_mW, fReceiverSensitivity_mW);

          // update table cells
          pStatisticReceiveMetricTable_->setCell(key, RX_METRIC_TBL_NUM_SAMPLES,     Any{std::get<0>(tableData)});
          pStatisticReceiveMetricTable_->setCell(key, RX_METRIC_TBL_AVG_RX_POWER,    Any{std::get<1>(tableData)});
          pStatisticReceiveMetricTable_->setCell(key, RX_METRIC_TBL_AVG_NOISE_FLOOR, Any{std::get<2>(tableData)});
          pStatisticReceiveMetricTable_->setCell(key, RX_METRIC_TBL_AVG_SINR,        Any{std::get<3>(tableData)});
          pStatisticReceiveMetricTable_->setCell(key, RX_METRIC_TBL_AVG_INR ,        Any{std::get<4>(tableData)});
       }

    // cache update result in dB, <numSamples, avgRxPower, avgNoiseFloor, avgSinr, avgInr>
    using ReceiveMetricCacheUpdateResult = std::tuple<std::uint64_t, double, double, double, double>;

    private:
      // !!! keep this is line with sReceiveMetircTableLables !!!
      enum ReceiveMetircTableIds { RX_METRIC_TBL_NEMID           = 0,   // src nem id
                                   RX_METRIC_TBL_ANTENNA_ID      = 1,   // rx antenna id
                                   RX_METRIC_TBL_FREQUENCY       = 2,   // frequency hz
                                   RX_METRIC_TBL_NUM_SAMPLES     = 3,   // num samples
                                   RX_METRIC_TBL_AVG_RX_POWER    = 4,   // avg rx power
                                   RX_METRIC_TBL_AVG_NOISE_FLOOR = 5,   // avg noise floor
                                   RX_METRIC_TBL_AVG_SINR        = 6,   // avg sinr
                                   RX_METRIC_TBL_AVG_INR         = 7};  // avg inr
                         
 
      // make a string key for the receive metric statistic table and receive metric cache/database
      // example: src = 1, rxAntennaId = 0, frequency = 1000000, result = 1:0:1000000
      inline std::string makeKey(EMANE::NEMId src, EMANE::AntennaIndex rxAntennaId, std::uint64_t frequencyHz)
       {
         return std::to_string(src) + 
                std::string{":"}    + 
                std::to_string(rxAntennaId) + 
                std::string{":"} + 
                std::to_string(frequencyHz);
       }

      // receive metric cache entry 
      class ReceiveMetricCacheEntry {
        public:
         ReceiveMetricCacheEntry() :
           u64NumSamples_{},
           fAvgRxPower_mW_{},
           fAvgNoiseFloor_mW_{},
           fAvgSinr_mW_{},
           fAvgInr_mW_{}
          { }

         ReceiveMetricCacheUpdateResult update(double fRxPower_mW, double fNoiseFloor_mW, double receiverSensitivity_mW)
          {
             getRunningAvg(fAvgRxPower_mW_,    u64NumSamples_, fRxPower_mW);
             getRunningAvg(fAvgNoiseFloor_mW_, u64NumSamples_, fNoiseFloor_mW);

             if(fAvgNoiseFloor_mW_ != 0.0)
              {
                fAvgSinr_mW_ = fAvgRxPower_mW_/fAvgNoiseFloor_mW_;
              }

             if(receiverSensitivity_mW != 0.0)
              {
                fAvgInr_mW_ = fAvgNoiseFloor_mW_/receiverSensitivity_mW;
              }

             ++u64NumSamples_;

             // parameter order is important here, convert to db
             return ReceiveMetricCacheUpdateResult{u64NumSamples_, 
                                                   EMANE::Utils::MILLIWATT_TO_DB(fAvgRxPower_mW_),
                                                   EMANE::Utils::MILLIWATT_TO_DB(fAvgNoiseFloor_mW_), 
                                                   EMANE::Utils::MILLIWATT_TO_DB(fAvgSinr_mW_), 
                                                   EMANE::Utils::MILLIWATT_TO_DB(fAvgInr_mW_)};
          }

        private:
         std::uint64_t  u64NumSamples_;

         double fAvgRxPower_mW_;
         double fAvgNoiseFloor_mW_;
         double fAvgSinr_mW_;
         double fAvgInr_mW_;
      }; // end cache entry


    // receive metric cache
    std::map<std::string, ReceiveMetricCacheEntry> receiveMetricCache_;

    // antenna tracking src/frequency
    std::map<EMANE::AntennaIndex, std::set<std::tuple<EMANE::NEMId, std::uint64_t>>> antennaTracker_;

    // our nem id
    EMANE::NEMId nemId_;

    StatisticTable<std::string> * pStatisticReceiveMetricTable_;

    bool bAverageAllAntenna_;

    bool bAverageAllFrequencies_;

    // typical row entry
    std::vector<Any> tableRowTemplate_;
 };


EMANE::ReceiveMetricTable::ReceiveMetricTable(EMANE::NEMId nemId) :
  pImpl_{new Implementation{nemId}}
{ }


EMANE::ReceiveMetricTable::~ReceiveMetricTable()
{ }


void EMANE::ReceiveMetricTable::configure(const EMANE::ConfigurationUpdate & configurationUpdate)
{
  pImpl_->configure(configurationUpdate);
}


void EMANE::ReceiveMetricTable::initialize(EMANE::Registrar & registrar)
{
  pImpl_->initialize(registrar);
}


void EMANE::ReceiveMetricTable::reset(EMANE::AntennaIndex rxAntennaId)
{
  pImpl_->reset(rxAntennaId);
}


void EMANE::ReceiveMetricTable::resetAll()
{
  pImpl_->resetAll();
}


void
EMANE::ReceiveMetricTable::update(NEMId src,
                                  EMANE::AntennaIndex rxAntennaId,
                                  std::uint64_t frequencyHz,
                                  double fRxPower_mW,
                                  double fNoiseFloor_mW,
                                  double fReceiverSensitivity_mW)
{
  pImpl_->update(src, rxAntennaId, frequencyHz, fRxPower_mW, fNoiseFloor_mW, fReceiverSensitivity_mW);
}
