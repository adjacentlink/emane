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
#include "emane/rfsignaltable.h"
#include "emane/statistictable.h"
#include "emane/utils/conversionutils.h"

#include <map>
#include <string>
#include <tuple>


namespace {
  // keep in sync with RFSignalTableIds
  const EMANE::StatisticTableLabels sRFSignalTableLables {"NEM",           // 0
                                                          "AntennaId",     // 1
                                                          "FrequencyHz",   // 2
                                                          "NumSamples",    // 3
                                                          "AvgRxPower",    // 4
                                                          "AvgNoiseFloor", // 5
                                                          "AvgSINR",       // 6
                                                          "AvgINR"};       // 7

  // keep in sync sRFSignalTableLables
  enum RFSignalTableIds { RFSIGNALTABLE_NEMID           = 0,   // src nem id
                          RFSIGNALTABLE_ANTENNAID       = 1,   // rx antenna id
                          RFSIGNALTABLE_FREQUENCY       = 2,   // frequency hz
                          RFSIGNALTABLE_NUM_SAMPLES     = 3,   // num samples
                          RFSIGNALTABLE_AVG_RX_POWER    = 4,   // avg rx power
                          RFSIGNALTABLE_AVG_NOISE_FLOOR = 5,   // avg noise floor
                          RFSIGNALTABLE_AVG_SINR        = 6,   // avg sinr
                          RFSIGNALTABLE_AVG_INR         = 7};  // avg inr
 
  const EMANE::AntennaIndex AntennaIndexDontCare = 0;
  const std::uint64_t       FrequencyDontCare    = 0;
}

const std::string EMANE::RFSignalTable::CONFIG_PREFIX = "rfsignaltable.";

class EMANE::RFSignalTable::Implementation
  {
    public:
      Implementation(EMANE::NEMId nemId) :
       nemId_{nemId},
       pStatisticRFSignalTable_{},
       bAverageAllAntenna_{},
       bAverageAllFrequencies_{}
       { }
 
      ~Implementation()
       { }

      void initialize(EMANE::Registrar & registrar)
       {
         auto & statisticRegistrar = registrar.statisticRegistrar();

         pStatisticRFSignalTable_ =
           statisticRegistrar.registerTable<std::string>("RFSignalTable", 
                                                         sRFSignalTableLables,
                                                         StatisticProperties::NONE,
                                                         "Rf Signal Table");


         auto & configRegistrar = registrar.configurationRegistrar();

         configRegistrar.registerNumeric<bool>(CONFIG_PREFIX+"averageallantennas",
                                               ConfigurationProperties::DEFAULT,
                                               {false},
                                               "Defines whether statistics for all antennas with be averaged together.");

         configRegistrar.registerNumeric<bool>(CONFIG_PREFIX+"averageallfrequencies",
                                               ConfigurationProperties::DEFAULT,
                                               {false},
                                               "Defines whether statistics for all frequencies with be averaged together.");
       }


      void configure(const EMANE::ConfigurationUpdate & configurationUpdate)
       {
         for(const auto & item : configurationUpdate)
          {
            if(item.first == CONFIG_PREFIX+"averageallantennas")
             {
               bAverageAllAntenna_ = item.second[0].asBool();
             }
            else if(item.first == CONFIG_PREFIX+"averageallfrequencies")
             {
               bAverageAllFrequencies_ = item.second[0].asBool();
             }
            else
             {
               const std::string sErr = "Unexpected RFSignalTable config item" + item.first;

               throw(EMANE::ConfigurationException(sErr));
             }
          }

          // layout a typical table row
          tableRowTemplate_.clear();

          // these will be set when new entry is added
          // 0 - NEMId
          tableRowTemplate_.push_back(Any{std::uint16_t{}});

          // 1 - AntenaId
          tableRowTemplate_.push_back(Any{std::string{"NA"}});

          // 2 - Frequency
          tableRowTemplate_.push_back(Any{std::string{"NA"}});

          // these will be set on each update
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
          // adjust if antenna combining
          getAdjustedAntennaId_i(rxAntennaId);

          const auto iter = antennaTracker_.find(rxAntennaId);

          if(iter != antennaTracker_.end())
           {
             for(const auto & e : iter->second)
              {
                const auto key = makeKey(std::get<0>(e), rxAntennaId, std::get<1>(e));

                rfReceiveMetricCache_.erase(key);                

                pStatisticRFSignalTable_->deleteRow(key);
              }

             antennaTracker_.erase(rxAntennaId);
           }
       }


      void resetAll()
       {
         pStatisticRFSignalTable_->clear();

         rfReceiveMetricCache_.clear();

         antennaTracker_.clear();
       }


      void update(NEMId src,
                  EMANE::AntennaIndex rxAntennaId,
                  std::uint64_t frequencyHz,
                  double dRxPower_mW,
                  double dNoiseFloor_mW,
                  double dReceiverSensitivity_mW)
       { 
          // sanity check, we have not been initialized
          if(! pStatisticRFSignalTable_)
           {
             return;
           }

          // adjust if antenna combining
          getAdjustedAntennaId_i(rxAntennaId);

          // adjust if frequency combining
          getAdjustedFrequency_i(frequencyHz);

          const auto key = makeKey(src, rxAntennaId, frequencyHz);

          auto iter = rfReceiveMetricCache_.find(key);

          if(iter == rfReceiveMetricCache_.end())
           {
             // new entry
             const auto result = rfReceiveMetricCache_.emplace(key, RFSignalCacheEntry{});

             // insert sanity check
             if(result.second)
              {
                iter = result.first;

                // add row template to statistic table
                pStatisticRFSignalTable_->addRow(key, tableRowTemplate_);

                // set specific values that represent this new entry 
                pStatisticRFSignalTable_->setCell(key, RFSIGNALTABLE_NEMID, Any{src});

                if(! bAverageAllAntenna_)
                 {
                   pStatisticRFSignalTable_->setCell(key, RFSIGNALTABLE_ANTENNAID, Any{rxAntennaId});
                 }

                if(! bAverageAllFrequencies_)
                 {
                   pStatisticRFSignalTable_->setCell(key, RFSIGNALTABLE_FREQUENCY,  Any{frequencyHz});
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
          const auto tableData = iter->second.update(dRxPower_mW, dNoiseFloor_mW, dReceiverSensitivity_mW);

          // update table cells
          pStatisticRFSignalTable_->setCell(key, RFSIGNALTABLE_NUM_SAMPLES,     Any{std::get<0>(tableData)});
          pStatisticRFSignalTable_->setCell(key, RFSIGNALTABLE_AVG_RX_POWER,    Any{std::get<1>(tableData)});
          pStatisticRFSignalTable_->setCell(key, RFSIGNALTABLE_AVG_NOISE_FLOOR, Any{std::get<2>(tableData)});
          pStatisticRFSignalTable_->setCell(key, RFSIGNALTABLE_AVG_SINR,        Any{std::get<3>(tableData)});
          pStatisticRFSignalTable_->setCell(key, RFSIGNALTABLE_AVG_INR ,        Any{std::get<4>(tableData)});
       }

    // cache update result in dB, <numSamples, avgRxPower, avgNoiseFloor, avgSinr, avgInr>
    using RFSignalCacheUpdateResult = std::tuple<std::uint64_t, double, double, double, double>;

    private:
                        
 
      // make a string key for the rf receive metric statistic table and receive metric cache/database
      // example: src = 1, rxAntennaId = 0, frequency = 1000000, result = 1:0:1000000
      inline std::string makeKey(EMANE::NEMId src, EMANE::AntennaIndex rxAntennaId, std::uint64_t frequencyHz)
       {
         return std::to_string(src) + 
                std::string{":"}    + 
                std::to_string(rxAntennaId) + 
                std::string{":"} + 
                std::to_string(frequencyHz);
       }

      // rf receive metric cache entry 
      class RFSignalCacheEntry {
        public:
         RFSignalCacheEntry() :
           u64NumSamples_{},
           dAvgRxPower_mW_{},
           dAvgNoiseFloor_mW_{},
           dAvgSinr_mW_{},
           dAvgInr_mW_{}
          { }

         RFSignalCacheUpdateResult update(double dRxPower_mW, double dNoiseFloor_mW, double receiverSensitivity_mW)
          {
             getRunningAvg_i(dAvgRxPower_mW_,    u64NumSamples_, dRxPower_mW);
             getRunningAvg_i(dAvgNoiseFloor_mW_, u64NumSamples_, dNoiseFloor_mW);

             // no divide by 0
             if(dAvgNoiseFloor_mW_ != 0.0)
              {
                dAvgSinr_mW_ = dAvgRxPower_mW_/dAvgNoiseFloor_mW_;
              }

             // no divide by 0
             if(receiverSensitivity_mW != 0.0)
              {
                dAvgInr_mW_ = dAvgNoiseFloor_mW_/receiverSensitivity_mW;
              }

             ++u64NumSamples_;

             // parameter order is important here, convert each to db
             return RFSignalCacheUpdateResult{u64NumSamples_, 
                                              EMANE::Utils::MILLIWATT_TO_DB(dAvgRxPower_mW_),
                                              EMANE::Utils::MILLIWATT_TO_DB(dAvgNoiseFloor_mW_), 
                                              EMANE::Utils::MILLIWATT_TO_DB(dAvgSinr_mW_), 
                                              EMANE::Utils::MILLIWATT_TO_DB(dAvgInr_mW_)};
          }

        private:
         std::uint64_t  u64NumSamples_;

         double dAvgRxPower_mW_;
         double dAvgNoiseFloor_mW_;
         double dAvgSinr_mW_;
         double dAvgInr_mW_;

        // accumulated running average
        inline void getRunningAvg_i(double &rAvg, const uint64_t count, const double newValue)
         {
           rAvg = (count == 1) ? newValue : ((rAvg * count) + newValue) / (count + 1);
         }
      }; // end cache entry definition

     inline void getAdjustedAntennaId_i(AntennaIndex & rAntennaId)
      {
        if(bAverageAllAntenna_)
         {
           // lump all antenna
           rAntennaId = AntennaIndexDontCare;
         }
      }

     inline void getAdjustedFrequency_i(std::uint64_t & rFrequencyHz)
      {
        if(bAverageAllFrequencies_)
         {
           // lump all freqs(s) into the average
           rFrequencyHz = FrequencyDontCare;
         }
      }

    // receive metric cache
    std::map<std::string, RFSignalCacheEntry> rfReceiveMetricCache_;

    // antenna tracking src/frequency
    std::map<EMANE::AntennaIndex, std::set<std::tuple<EMANE::NEMId, std::uint64_t>>> antennaTracker_;

    // our nem id
    EMANE::NEMId nemId_;

    StatisticTable<std::string> * pStatisticRFSignalTable_;

    bool bAverageAllAntenna_;

    bool bAverageAllFrequencies_;

    // typical row entry template
    std::vector<Any> tableRowTemplate_;
 };




EMANE::RFSignalTable::RFSignalTable(EMANE::NEMId nemId) :
  pImpl_{new Implementation{nemId}}
{ }


EMANE::RFSignalTable::~RFSignalTable()
{ }


void EMANE::RFSignalTable::configure(const EMANE::ConfigurationUpdate & configurationUpdate)
{
  pImpl_->configure(configurationUpdate);
}


void EMANE::RFSignalTable::initialize(EMANE::Registrar & registrar)
{
  pImpl_->initialize(registrar);
}


void EMANE::RFSignalTable::reset(EMANE::AntennaIndex rxAntennaId)
{
  pImpl_->reset(rxAntennaId);
}


void EMANE::RFSignalTable::resetAll()
{
  pImpl_->resetAll();
}


void
EMANE::RFSignalTable::update(NEMId src,
                             EMANE::AntennaIndex rxAntennaId,
                             std::uint64_t frequencyHz,
                             double dRxPower_mW,
                             double dNoiseFloor_mW,
                             double dReceiverSensitivity_mW)
{
  pImpl_->update(src, rxAntennaId, frequencyHz, dRxPower_mW, dNoiseFloor_mW, dReceiverSensitivity_mW);
}
