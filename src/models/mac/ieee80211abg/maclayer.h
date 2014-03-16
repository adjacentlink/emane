/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2008-2012 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANEMODELSIEEE802ABGMACLAYER_HEADER_
#define EMANEMODELSIEEE802ABGMACLAYER_HEADER_

#include "emane/maclayerimpl.h"
#include "emane/mactypes.h"
#include "emane/flowcontrolmanager.h"
#include "emane/transmitter.h"
#include "emane/frequencysegment.h"
#include "emane/neighbormetricmanager.h"
#include "emane/queuemetricmanager.h"

#include "emane/utils/randomnumberdistribution.h"
#include "emane/utils/commonlayerstatistics.h"

#include "macheaderparams.h"
#include "downstreamqueue.h"
#include "transmissiontxstate.h"
#include "macstatistics.h"

#include "macconfig.h"
#include "modetimingparameters.h"
#include "pcrmanager.h"
#include "neighbormanager.h"
#include "collisiontable.h"

#include <memory>

#include <map>
#include <vector>

namespace EMANE
{
  namespace Models
  {
    namespace IEEE80211ABG
    {
      /**
       *
       * @brief structure used to define parameters to detect duplicate frames
       *
       */
      struct SequenceEntry
      {
        std::uint16_t     seq_;        // sequence number
        TimePoint         tp_;         // validity time

        /**
         *
         * default constructor
         *
         */

        SequenceEntry(std::uint16_t seq, const TimePoint & tp) : 
          seq_(seq), 
          tp_(tp) 
        { }
      };

      using SequenceVector = std::vector<SequenceEntry>;

      using DuplicateMap = std::map<NEMId, SequenceVector>;

      class TransmissionTxState;

      /**
       * @class MACLayer 
       *  
       * @brief IEEE 80211 ABG MAC implementation
       *
       */
      class MACLayer : public MACLayerImplementor
      {
      public:
        MACLayer(NEMId id,
                 PlatformServiceProvider *pPlatformServiceProvider,
                 RadioServiceProvider * pRadioServiceProvider);

        ~MACLayer();

        /*
         * Component Interface
         */
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


        void processEvent(const EventId &, const Serialization &);

        void processTimedEvent(TimerEventId eventId,
                               const TimePoint & expireTime,
                               const TimePoint & scheduleTime,
                               const TimePoint & fireTime,
                               const void * arg) override;

        void processConfiguration(const ConfigurationUpdate & update) override;

        /**
         *
         * collision type none, clobber or noise
         *
         */
        enum COLLISION_TYPE { COLLISION_TYPE_NONE                    = 0x00, 
                              COLLISION_TYPE_CLOBBER_RX_DURING_TX    = 0x01,
                              COLLISION_TYPE_NOISE_COMMON_RX         = 0x02, 
                              COLLISION_TYPE_CLOBBER_RX_HIDDEN_BUSY  = 0x04, 
                              COLLISION_TYPE_NOISE_HIDDEN_RX         = 0x08 };


        /**
         *
         * get the NEM id
         *
         * @retval NEM id
         *
         */
        NEMId getId()const;


        /**
         *
         * send a downstream broadcast data packet
         * 
         * @param entry downstream queue entry
         * 
         */
        void sendDownstreamBroadcastData(DownstreamQueueEntry &);


        /**
         *
         * send a downstream unicast data packet
         * 
         * @param entry downstream queue entry
         * 
         */
        void sendDownstreamUnicastData(DownstreamQueueEntry &entry);


        /**
         *
         * send a downstream unicast cts ctrl packet
         * 
         * @param entry downstream queue entry
         * @param origin the origin of the unicast rts/cts exchange
         * 
         */
        void sendDownstreamUnicastCts(DownstreamQueueEntry & entry,
                                      NEMId origin);

        /**
         *
         * send a downstream message
         * 
         * @param entry downstream queue entry
         * @param macHeaderParams specific mac header parameters for the message
         * 
         */
        void sendDownstreamMessage(DownstreamQueueEntry & entry,
                                   MACHeaderParams & macHeaderParams);

        /**
         *
         * check if a duplicate packet has been received
         *
         * @param src the source nem
         * @param seq sequence number of packet
         * 
         * @retval true if src and seq match the last received values, otherwise false.
         *
         */
        bool isDuplicate(NEMId src, std::uint16_t seq);


        /**
         *
         * add token to flow control
         *
         * @retval true on success false on failure
         *
         */
        bool addToken();


        /**
         *
         * remove token from flow control
         *
         * @retval true on success false on failure
         *
         */
        bool removeToken();


        /**
         *
         * set the pre and post delays for a downstream queue entry
         *
         * @param entry the downstream entry
         *
         */
        void setDelayTime(IEEE80211ABG::DownstreamQueueEntry &entry);


        /**
         *
         * get the type of collision during rx
         *
         * @param src the src of the pkt 
         *
         * @param u8Category the queue service type
         *
         * @param retries the number of retries
         *
         * @retval returns the COLLISION_TYPE
         *
         */
        COLLISION_TYPE checkForRxCollision(NEMId src, std::uint8_t u8Category, std::uint8_t retries);


        /**
         * provides access to the MAC layer statistics
         *
         * @return reference to the statistics object
         */
        MACStatistics & getStatistics();


        /**
         * provides access to the MAC layer mode timing 
         *
         * @return reference to the configuration object
         */
        ModeTimingParameters & getModeTiming();


      private:
        static const RegistrationId registrationId_ {EMANE::REGISTERED_EMANE_MAC_IEEE_802_11_ABG};

        NEMId id_;

        MACConfig  macConfig_;

        MACStatistics macStatistics_;

        DownstreamQueue downstreamQueue_;

        TransmissionTxState *pTxState_;

        PCRManager pcrManager_;

        NeighborManager neighborManager_;

        NeighborMetricManager neighborMetricManager_;

        QueueMetricManager queueMetricManager_;

        FlowControlManager flowControlManager_;

        CollisionTable collisionTable_;

        std::uint64_t u64SequenceNumber_;

        std::uint16_t u16EntrySequenceNumber_;

        DuplicateMap duplicateMap_;

        ModeTimingParameters modeTiming_;

        TimerEventId channelUsageTimedEventId_;

        TimerEventId radioMetricTimedEventId_;

        TimerEventId downstreamQueueTimedEventId_;

        bool bHasPendingDownstreamQueueEntry_;

        DownstreamQueueEntry pendingDownstreamQueueEntry_;

        std::unique_ptr<std::function<bool()>> pChannelUsageCallback_;

        std::unique_ptr<std::function<bool()>> pRadioMetricCallback_;

        Utils::RandomNumberDistribution<std::mt19937, 
                                        std::uniform_real_distribution<float>> RNDZeroToOne_;

        std::vector<std::unique_ptr<Utils::CommonLayerStatistics>> commonLayerStatistics_;

        bool handleDownstreamQueueEntry();

        void setEntrySequenceNumber(DownstreamQueueEntry &entry);

        void handleUpstreamPacket(UpstreamPacket & pkt,
                                  double dRxPowerdBm, 
                                  double dNoiseFloordBm,
                                  std::uint64_t u64SequenceNumber,
                                  const TimePoint & rBeginTime,
                                  std::uint8_t u8Category);
 

        std::pair<bool, std::uint16_t> checkUpstremReception(UpstreamPacket &pkt,
                                   const TimePoint & currentTime,
                                   std::uint64_t u64SequenceNumber,
                                   double dRxPowerdBm, 
                                   double dNoiseFloordBm,
                                   const MACHeaderParams & macHeaderParams,
                                   int tryNum,
                                   std::uint8_t u8Category);

        void changeDownstreamState(TransmissionTxState *);
       
        bool checkPOR(float fSINR, size_t packetSize, std::uint16_t u16DataRateIndex);

        std::uint8_t dscpToCategory(std::uint8_t dscp) const;

        inline bool isBroadcast(NEMId nemId)
        {
          return nemId == NEM_BROADCAST_MAC_ADDRESS;
        }


        friend class TransmissionTxState;
      };
    }
  }
}

#endif //EMANEMODELSIEEE802ABGMACLAYER_HEADER_
