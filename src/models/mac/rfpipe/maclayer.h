/*
 * Copyright (c) 2013-2016 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2008,2009,2010 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef RFPIPEMAC_MACLAYER_HEADER_
#define RFPIPEMAC_MACLAYER_HEADER_

#include "emane/maclayerimpl.h"
#include "emane/mactypes.h"
#include "emane/flowcontrolmanager.h"
#include "emane/neighbormetricmanager.h"
#include "emane/queuemetricmanager.h"
#include "emane/statisticnumeric.h"
#include "emane/rfsignaltable.h"

#include "emane/utils/runningaverage.h"
#include "emane/utils/randomnumberdistribution.h"
#include "emane/utils/commonlayerstatistics.h"

#include "downstreamqueue.h"
#include "pcrmanager.h"

#include <memory>

namespace EMANE
{
  namespace Models
  {
    namespace RFPipe
    {
      /**
       *
       * @class MACLayer
       *
       * @brief Implementation of the rf pipe mac layer.
       *
       */

      class MACLayer : public MACLayerImplementor
      {
      public:
        /**
         * constructor
         *
         * @param id this NEM id.
         * @param pPlatformServiceProvider reference to the platform service provider
         * @param pRadioServiceProvider reference to the radio service provider
         *
         */
        MACLayer(NEMId id,
                 PlatformServiceProvider *pPlatformServiceProvider,
                 RadioServiceProvider * pRadioServiceProvider);

        /**
         *
         * destructor
         *
         */
        ~MACLayer();


        // mac layer implementor api below

        void initialize(Registrar & registrar) override;

        void configure(const ConfigurationUpdate & update) override;

        void start() override;

        void postStart() override;

        void stop() override;

        void destroy() throw() override;

        void processUpstreamControl(const ControlMessages & msgs) override;


        void processUpstreamPacket(const CommonMACHeader & hdr,
                                   UpstreamPacket & pkt,
                                   const ControlMessages & msgs) override;

        void processDownstreamControl(const ControlMessages & msgs) override;


        void processDownstreamPacket(DownstreamPacket & pkt,
                                     const ControlMessages & msgs) override;

        void processConfiguration(const ConfigurationUpdate & update) override;

      private:
        /**
         *
         * @brief  the emane rf pipe registration id
         *
         */
        static const RegistrationId type_ = REGISTERED_EMANE_MAC_RF_PIPE;

        DownstreamQueue downstreamQueue_;

        std::uint64_t u64TxSequenceNumber_;

        FlowControlManager flowControlManager_;

        PCRManager pcrManager_;

        NeighborMetricManager neighborMetricManager_;

        QueueMetricManager queueMetricManager_;

        RFSignalTable rfSignalTable_;

        // config items
        bool bPromiscuousMode_;

        std::uint64_t u64DataRatebps_;

        Microseconds delayMicroseconds_;

        bool bFlowControlEnable_;

        bool bRadioMetricEnable_;

        std::uint16_t u16FlowControlTokens_;

        std::string sPCRCurveURI_;

        Microseconds radioMetricReportIntervalMicroseconds_;

        Microseconds neighborMetricDeleteTimeMicroseconds_;

        TimerEventId radioMetricTimedEventId_;

        Utils::CommonLayerStatistics commonLayerStatistics_;

        Utils::RandomNumberDistribution<std::mt19937,
                                        std::uniform_real_distribution<float>> RNDZeroToOne_;

        std::unique_ptr<Utils::RandomNumberDistribution<std::mt19937,
                                                        std::uniform_real_distribution<float>>> pRNDJitter_;

        float fJitterSeconds_;

        StatisticNumeric<std::uint64_t> * pNumDownstreamQueueDelay_;

        Utils::RunningAverage<float>  avgDownstreamQueueDelay_;

        TimerEventId downstreamQueueTimedEventId_;

        bool bHasPendingDownstreamQueueEntry_;

        DownstreamQueueEntry pendingDownstreamQueueEntry_;

        TimePoint currentEndOfTransmissionTime_;

        Microseconds currentDelay_;

        void handleDownstreamQueueEntry(TimePoint sot,
                                        std::uint64_t u64TxSequenceNumber);

        Microseconds getDurationMicroseconds(size_t lengthInBytes);

        Microseconds getJitter();

        bool checkPOR(float fSINR, size_t packetSize);
      };
    }
  }
}

#endif //RFPIPEMAC_MACLAYER_HEADER_
