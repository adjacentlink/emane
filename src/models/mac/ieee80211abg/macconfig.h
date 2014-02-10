/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANEMODELSIEEE802ABGMACCONFIG_HEADER_
#define EMANEMODELSIEEE802ABGMACCONFIG_HEADER_


#include "emane/types.h"
#include "emane/packetinfo.h"
#include "emane/registrar.h"
#include "emane/configurationupdate.h"
#include "emane/logserviceprovider.h"

#include <string>
#include <vector>

namespace EMANE
 {
   namespace Models
    {
      namespace IEEE80211ABG
       {

        // currently supporting modes 802.11a, 802.11b, 802.11b/g
        enum MODULATION_TYPE
        { MODULATION_TYPE_DEFAULT = 0x0,
          MODULATION_TYPE_80211A  = 0x1,
          MODULATION_TYPE_80211B  = 0x2,
          MODULATION_TYPE_80211BG = 0x3
        };

       // number of queues
       const std::uint8_t  MAX_ACCESS_CATEGORIES{4};

       const std::uint8_t  QUEUE_SIZE_DEFAULT{255};

       // max packet size
       const std::uint16_t MAX_PACKET_SIZE{0xFFFF};

       // modulation type
       const std::uint8_t MODULATION_TYPE_INDEX_MIN{0};
       const std::uint8_t MODULATION_TYPE_INDEX_MAX{3};


       typedef std::vector<float> CWRatioVector;
       typedef std::vector<CWRatioVector> CWRatioTable;

      /**
        *
        * @brief mac configuration items.
        *
        */
        struct ConfigItems
        {
          bool           bWmmEnable_;                               // enable wmm
          bool           bPromiscousModeEnable_;                    // enable promiscous mode
          bool           bFlowControlEnable_;                       // flow control enable
          bool           bRadioMetricEnable_;                       // radio metrics enable

          std::uint8_t   u8ModeIndex_;                              // mode index (modulation type)

          std::uint8_t   u8UnicastDataRateIndex_;                   // unicast data rate index
          std::uint8_t   u8BroadcastDataRateIndex_;                 // broadcast data rate index
          std::uint16_t  u16RtsThreshold_;                          // rtc cts enable threshold
          std::uint16_t  u16FlowControlTokens_;                     // flow control tokens

          std::uint32_t  u32MaxP2PDistance_;                        // max p2p distance

          std::string    sPcrUri_;                                  // pcr uri

          Microseconds   neighborTimeoutMicroseconds_;              // neighbor timeout
          Microseconds   channelActivityIntervalMicroseconds_;      // channel activity interval
          Microseconds   radioMetricReportIntervalMicroseconds_;    // radio metric report intrval
          Microseconds   neighborMetricDeleteTimeMicroseconds_;     // nbr metric delate time (age)

          std::uint16_t  u16CWMin0_;                                // contention window min category 1
          std::uint16_t  u16CWMin1_;                                // contention window min category 2
          std::uint16_t  u16CWMin2_;                                // contention window min category 3
          std::uint16_t  u16CWMin3_;                                // contention window min category 4

          std::uint16_t  u16CWMax0_;                                // contention window min category 1
          std::uint16_t  u16CWMax1_;                                // contention window min category 2
          std::uint16_t  u16CWMax2_;                                // contention window min category 3
          std::uint16_t  u16CWMax3_;                                // contention window min category 4

          Microseconds   txopMicroseconds0_;                        // tx opportunity category 0
          Microseconds   txopMicroseconds1_;                        // tx opportunity category 1
          Microseconds   txopMicroseconds2_;                        // tx opportunity category 2
          Microseconds   txopMicroseconds3_;                        // tx opportunity category 3

          Microseconds   aifsMicroseconds0_;                        // aifs duration microseconds category 0
          Microseconds   aifsMicroseconds1_;                        // aifs duration microseconds category 1
          Microseconds   aifsMicroseconds2_;                        // aifs duration microseconds category 2
          Microseconds   aifsMicroseconds3_;                        // aifs duration microseconds category 3

          std::uint8_t   u8RetryLimit0_;                            // retry limit category 0
          std::uint8_t   u8RetryLimit1_;                            // retry limit category 1
          std::uint8_t   u8RetryLimit2_;                            // retry limit category 2
          std::uint8_t   u8RetryLimit3_;                            // retry limit category 3

          std::uint8_t  u8QueueSize0_;                              // queue size category 0
          std::uint8_t  u8QueueSize1_;                              // queue size category 1
          std::uint8_t  u8QueueSize2_;                              // queue size category 2
          std::uint8_t  u8QueueSize3_;                              // queue size category 3

          std::uint16_t u16MSDU0_;                                  // max queue entry size category 0
          std::uint16_t u16MSDU1_;                                  // max queue entry size category 1
          std::uint16_t u16MSDU2_;                                  // max queue entry size category 2
          std::uint16_t u16MSDU3_;                                  // max queue entry size category 3

          CWRatioTable CWMinRatioTable_;

          ConfigItems();
        };


      /**
      *
      * @brief class used to define the mac layer configuration items
      *
      */
        class MACConfig
        {
          public:

            MACConfig(LogServiceProvider & logServiceProvider, NEMId id);

            ~MACConfig();

            bool getPromiscuosEnable() const;

            bool getWmmEnable() const;
 
            void registerConfiguration(ConfigurationRegistrar & configRegistrar);

            MODULATION_TYPE getModulationType() const;

            std::uint8_t getUnicastDataRateIndex() const;

            std::uint8_t getBroadcastDataRateIndex() const;

            std::uint32_t getUnicastDataRateKbps() const;

            std::uint32_t getBroadcastDataRateKbps() const;

            std::uint32_t getMaxDataRateKbps() const;

            std::uint32_t getUnicastDataRateKbps(std::uint8_t) const;

            std::uint32_t getBroadcastDataRateKbps(std::uint8_t) const;

            std::uint32_t getMaxP2pDistance() const;

            std::uint8_t getNumAccessCategories() const;

            std::uint16_t getRtsThreshold() const;

            std::uint8_t getQueueSize(std::uint8_t) const;

            std::uint16_t getQueueEntrySize(std::uint8_t) const;

            std::uint16_t getCWMin(std::uint8_t) const;

            std::uint16_t getCWMax(std::uint8_t) const;

            void setCWMin0(std::uint16_t u16Value);
  
            void setCWMin1(std::uint16_t u16Value);
  
            void setCWMin2(std::uint16_t u16Value);

            void setCWMin3(std::uint16_t u16Value);

            void setCWMax0(std::uint16_t u16Value);

            void setCWMax1(std::uint16_t u16Value);

            void setCWMax2(std::uint16_t u16Value);

            void setCWMax3(std::uint16_t u16Value);

            Microseconds getAifsMicroseconds(std::uint8_t) const;

            Microseconds getTxOpMicroseconds(std::uint8_t) const;

            std::uint8_t getRetryLimit(std::uint8_t) const;

            std::uint16_t getFlowControlTokens() const;

            bool getFlowControlEnable() const;

            std::string getPcrUri() const;

            Microseconds getNeighborTimeoutMicroseconds() const;

            Microseconds getChannelActivityIntervalMicroseconds() const;

            bool configure(const ConfigurationUpdate & update);

            bool processConfiguration(const ConfigurationUpdate & update);

            CWRatioVector getCWMinRatioVector(std::uint8_t) const;

            Microseconds getNeighborMetricDeleteTimeMicroseconds() const;

            Microseconds getRadioMetricReportIntervalMicroseconds() const;

            bool getRadioMetricEnable() const;

          private:
            LogServiceProvider & logServiceProvider_;

            NEMId id_;

            ConfigItems configItems_;

            bool configureStaticItems(const ConfigurationNameAnyValues & item);

            bool configureDynamicItems(const ConfigurationNameAnyValues & item);

            void setCWMinRatioVector(std::uint8_t u8Category);

            void initCWMinRatioTable();
       };
     }
  } 
}

#endif //EMANEMODELSIEEE802ABGMACCONFIG_HEADER_
