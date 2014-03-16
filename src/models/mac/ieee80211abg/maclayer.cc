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

#include "maclayer.h"
#include "macstatistics.h"
#include "idletxstate.h"
#include "utils.h"
#include "ieee80211abgmacheadermessage.h"
#include "macheaderparamsformatter.h"
#include "configurationvalidator.h"

#include "emane/controls/flowcontrolcontrolmessage.h"
#include "emane/controls/frequencycontrolmessage.h"
#include "emane/controls/receivepropertiescontrolmessage.h"
#include "emane/controls/serializedcontrolmessage.h"
#include "emane/controls/r2riselfmetriccontrolmessage.h"
#include "emane/controls/receivepropertiescontrolmessageformatter.h"
#include "emane/controls/frequencycontrolmessageformatter.h"
#include "emane/controls/timestampcontrolmessage.h"

#include "emane/configureexception.h"
#include "emane/startexception.h"
#include "emane/controlmessage.h"
#include "emane/event.h"
#include "emane/constants.h"
#include "emane/spectrumserviceexception.h"

#include "emane/utils/parameterconvert.h"
#include "emane/utils/spectrumwindowutils.h"
#include "emane/utils/conversionutils.h"

#include <sstream>

#include <cmath>

namespace
{
  const char *pzLayerName{"IEEEMACLayer"};

  // undocumented test params
  // tx retry logic enable
  const bool ENABLE_TX_RETRY{false};

  // wmm overhead enable
  const bool ENABLE_WMM_OVERHEAD{false};

  const std::uint16_t DROP_CODE_SINR               = 1;
  const std::uint16_t DROP_CODE_REGISTRATION_ID    = 2;
  const std::uint16_t DROP_CODE_DST_MAC            = 3;
  const std::uint16_t DROP_CODE_QUEUE_OVERFLOW     = 4;
  const std::uint16_t DROP_CODE_BAD_CONTROL_INFO   = 5;
  const std::uint16_t DROP_CODE_BAD_SPECTRUM_QUERY = 6;
  const std::uint16_t DROP_CODE_FLOW_CONTROL_ERROR = 7;
  const std::uint16_t DROP_CODE_DUPLICATE          = 8;

  EMANE::StatisticTableLabels STATISTIC_TABLE_LABELS
  {"SINR",
      "Reg Id",
      "Dst MAC",
      "Queue Overflow",
      "Bad Control",
      "Bad Spectrum Query",
      "Flow Control",
      "Duplicate"};
}



EMANE::Models::IEEE80211ABG::MACLayer::MACLayer(NEMId id,
                                                PlatformServiceProvider * pPlatformServiceProvider,
                                                RadioServiceProvider * pRadioServiceProvider):
  MACLayerImplementor{id, pPlatformServiceProvider, pRadioServiceProvider},
  id_{id},
  macConfig_{pPlatformServiceProvider->logService(), id},
  macStatistics_{id},
  downstreamQueue_{id},
  pTxState_{IdleTxStateSingleton::instance()},
  pcrManager_{id_, pPlatformService_},
  neighborManager_{id_, pPlatformService_, this}, 
  neighborMetricManager_{id},
  queueMetricManager_{id},
  flowControlManager_{*this}, 
  u64SequenceNumber_{},
  u16EntrySequenceNumber_{},
  modeTiming_{macConfig_},
  channelUsageTimedEventId_{},
  radioMetricTimedEventId_{},
  downstreamQueueTimedEventId_{},
  bHasPendingDownstreamQueueEntry_{},
  RNDZeroToOne_{0.0f, 1.0f},
  commonLayerStatistics_(MAX_ACCESS_CATEGORIES)
{
  commonLayerStatistics_[0].reset(
    new Utils::CommonLayerStatistics{STATISTIC_TABLE_LABELS,
                                     STATISTIC_TABLE_LABELS, "0"});
  commonLayerStatistics_[1].reset(
    new Utils::CommonLayerStatistics{STATISTIC_TABLE_LABELS,
                                     STATISTIC_TABLE_LABELS, "1"});
  commonLayerStatistics_[2].reset(
    new Utils::CommonLayerStatistics{STATISTIC_TABLE_LABELS,
                                     STATISTIC_TABLE_LABELS, "2"});
  commonLayerStatistics_[3].reset(
    new Utils::CommonLayerStatistics{STATISTIC_TABLE_LABELS,
                                     STATISTIC_TABLE_LABELS, "3"});
}


EMANE::Models::IEEE80211ABG::MACLayer::~MACLayer()
{ }


void
EMANE::Models::IEEE80211ABG::MACLayer::initialize(Registrar & registrar)
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL, 
                          "MACI %03hu %s::%s", 
                          id_, pzLayerName, __func__);

  /** [configurationregistrar-registervalidator-snippet] */

  auto & configRegistrar = registrar.configurationRegistrar();

  macConfig_.registerConfiguration(configRegistrar);

  configRegistrar.registerValidator(&configurationValidator);

  /** [configurationregistrar-registervalidator-snippet] */

  auto & statisticRegistrar = registrar.statisticRegistrar();

  macStatistics_.registerStatistics(statisticRegistrar);

  downstreamQueue_.registerStatistics(statisticRegistrar);

  neighborManager_.registerStatistics(statisticRegistrar);

  for(auto & iter : commonLayerStatistics_)
    {
      iter->registerStatistics(statisticRegistrar);
    }

   neighborMetricManager_.registerStatistics(statisticRegistrar);

   auto & eventRegistrar = registrar.eventRegistrar();

   eventRegistrar.registerEvent(OneHopNeighborsEvent::IDENTIFIER);
}



void
EMANE::Models::IEEE80211ABG::MACLayer::configure(const ConfigurationUpdate & update)
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL, 
                          "MACI %03hu %s::%s",
                          id_, pzLayerName, __func__);

  if(!macConfig_.configure(update))
    {
      throw(ConfigureException{"IEEE80211ABG MACLayer configuration failure"});
    }

  // load pcr curve
  pcrManager_.load(macConfig_.getPcrUri());
}



void
EMANE::Models::IEEE80211ABG::MACLayer::start()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "MACI %03hu %s::%s",
                          id_, pzLayerName, __func__);

  // number of access categories (queues)
  std::uint8_t u8NumCategories{macConfig_.getNumAccessCategories()};

  // set downstream queue categories and sizes
  downstreamQueue_.setCategories(u8NumCategories);

  for(std::uint8_t u8Category = 0; u8Category < u8NumCategories; ++u8Category)
    {
      size_t maxCapacity{macConfig_.getQueueSize(u8Category)};

      size_t maxEntrySize{macConfig_.getQueueEntrySize(u8Category)};

      downstreamQueue_.setMaxCapacity(maxCapacity, u8Category);

      downstreamQueue_.setMaxEntrySize(maxEntrySize, u8Category);

      queueMetricManager_.addQueueMetric(u8Category, maxCapacity);
    }

  // set neighbor timeout and num categories
  neighborManager_.setNeighborTimeoutMicroseconds(macConfig_.getNeighborTimeoutMicroseconds());

  neighborManager_.setCategories(u8NumCategories);

  // the the neighbor delete time (age)
  neighborMetricManager_.setNeighborDeleteTimeMicroseconds(
    macConfig_.getNeighborMetricDeleteTimeMicroseconds());
}



void
EMANE::Models::IEEE80211ABG::MACLayer::postStart()
{
  auto timeNow = Clock::now();

  if(macConfig_.getFlowControlEnable())
    {
      flowControlManager_.start(macConfig_.getFlowControlTokens());
    }

  neighborManager_.start();

  pChannelUsageCallback_.reset(new std::function<bool()>{
      [this]()
        {
          // reset statistics
          neighborManager_.resetStatistics();
          
          // do not delete after executing
          return false;
        }
    });

    
  channelUsageTimedEventId_ =
    pPlatformService_->timerService().
    scheduleTimedEvent(timeNow + 
                       macConfig_.getChannelActivityIntervalMicroseconds(),
                       pChannelUsageCallback_.get(), 
                       macConfig_.getChannelActivityIntervalMicroseconds());


  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "MACI %03hu %s::%s: added channel activity timed eventId %zu", 
                          id_, 
                          pzLayerName,
                          __func__,
                          channelUsageTimedEventId_);

  pRadioMetricCallback_.reset(new std::function<bool()>{
      [this]()
        {
          if(! macConfig_.getRadioMetricEnable())
            {
              neighborMetricManager_.updateNeighborStatus();
            }
          else
            {
              sendUpstreamControl({
                Controls::R2RISelfMetricControlMessage::create(
                  macConfig_.getBroadcastDataRateKbps() * 1000ULL,
                  macConfig_.getMaxDataRateKbps() * 1000ULL,
                  macConfig_.getRadioMetricReportIntervalMicroseconds()),
                Controls::R2RINeighborMetricControlMessage::create(neighborMetricManager_.getNeighborMetrics()),
                Controls::R2RIQueueMetricControlMessage::create(queueMetricManager_.getQueueMetrics())});
            }
                
            // do not delete after executing
            return false;
          }
    });
        
  radioMetricTimedEventId_ = 
    pPlatformService_->timerService().
    scheduleTimedEvent(timeNow + macConfig_.getRadioMetricReportIntervalMicroseconds(),
                       pRadioMetricCallback_.get(), 
                       macConfig_.getRadioMetricReportIntervalMicroseconds());
    
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "MACI %03hu %s::%s: added radio metric timed eventId %zu", 
                          id_, 
                          pzLayerName,
                          __func__,
                          radioMetricTimedEventId_);
}


void
EMANE::Models::IEEE80211ABG::MACLayer::processConfiguration(const ConfigurationUpdate & update)
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "MACI %03hu %s::%s",
                          id_, pzLayerName, __func__);

  macConfig_.processConfiguration(update);
}


void
EMANE::Models::IEEE80211ABG::MACLayer::stop()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "MACI %03hu %s::%s",
                          id_,
                          pzLayerName,
                          __func__);

  pPlatformService_->timerService().cancelTimedEvent(channelUsageTimedEventId_);

  channelUsageTimedEventId_ = 0;

  pPlatformService_->timerService().cancelTimedEvent(radioMetricTimedEventId_);

  radioMetricTimedEventId_ = 0;

  pPlatformService_->timerService().cancelTimedEvent(downstreamQueueTimedEventId_);

  downstreamQueueTimedEventId_ = 0;

  if(macConfig_.getFlowControlEnable())
    {
      flowControlManager_.stop();
    }
}



void
EMANE::Models::IEEE80211ABG::MACLayer::destroy()
  throw()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "MACI %03hu %s::%s",
                          id_,
                          pzLayerName,
                          __func__);
}



void
EMANE::Models::IEEE80211ABG::MACLayer::processUpstreamControl(const ControlMessages &)
{
  LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                         DEBUG_LEVEL,
                         "MACI %03hu %s::%s",
                         id_,
                         pzLayerName,
                         __func__);
}



void
EMANE::Models::IEEE80211ABG::MACLayer::processDownstreamControl(const ControlMessages & msgs)
{
  for(const auto & pMessage : msgs)
    {
      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                             DEBUG_LEVEL,
                             "MACI %03hu %s::%s downstream control message id %hu", 
                             id_,
                             pzLayerName,
                             __func__,
                             pMessage->getId());

      switch(pMessage->getId())
        {
        case Controls::FlowControlControlMessage::IDENTIFIER:
          {
            const auto pFlowControlControlMessage =
              static_cast<const Controls::FlowControlControlMessage *>(pMessage);

            flowControlManager_.processFlowControlMessage(pFlowControlControlMessage);
          }

          break;

        case Controls::SerializedControlMessage::IDENTIFIER:
          {
            const auto pSerializedControlMessage =
              static_cast<const Controls::SerializedControlMessage *>(pMessage); 
        
            switch(pSerializedControlMessage->getSerializedId())
              {
              case Controls::FlowControlControlMessage::IDENTIFIER:
                {
                  std::unique_ptr<Controls::FlowControlControlMessage> 
                    pFlowControlControlMessage{Controls::FlowControlControlMessage::create
                      (pSerializedControlMessage->getSerialization())};
                    
                  flowControlManager_.processFlowControlMessage(pFlowControlControlMessage.get());
                }
                  
                break;
              }
          }
        }
    }
}



void
EMANE::Models::IEEE80211ABG::MACLayer::processUpstreamPacket(const CommonMACHeader & commonMACHeader,
                                                             UpstreamPacket & pkt,
                                                             const ControlMessages & msgs)
{
  // get current time
  TimePoint timeNow{Clock::now()};

  const PacketInfo & pktInfo{pkt.getPacketInfo()};

  std::uint8_t u8Category{dscpToCategory(pktInfo.getPriority())};

  commonLayerStatistics_[u8Category]->processInbound(pkt);


  LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                         DEBUG_LEVEL,
                         "MACI %03hu %s::%s: src %hu, dst %hu",
                         id_,
                         pzLayerName,
                         __func__,
                         pktInfo.getSource(),
                         pktInfo.getDestination());

  // check mac header registration id(ieee80211abg)
  if(commonMACHeader.getRegistrationId() != registrationId_)
    {
      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                             ERROR_LEVEL, "MACI %03hu %s::%s: src %hu, dst %hu, "
                             "MAC registration id %hu does not match our id %hu, drop.",
                             id_,
                             pzLayerName,
                             __func__,
                             pktInfo.getSource(),
                             pktInfo.getDestination(),
                             commonMACHeader.getRegistrationId(),
                             registrationId_);

      commonLayerStatistics_[u8Category]->processOutbound(
                                                          pkt, 
                                                          std::chrono::duration_cast<Microseconds>(Clock::now() - timeNow), 
                                                          DROP_CODE_REGISTRATION_ID);

      // drop
      return;
    }

  const Controls::ReceivePropertiesControlMessage * pReceivePropertiesControlMessage{};

  const Controls::FrequencyControlMessage * pFrequencyControlMessage{};
      
  for(auto & pControlMessage : msgs)
    {
      switch(pControlMessage->getId())
        {
        case EMANE::Controls::ReceivePropertiesControlMessage::IDENTIFIER:
          {
            pReceivePropertiesControlMessage =
              static_cast<const Controls::ReceivePropertiesControlMessage *>(pControlMessage); 

            LOGGER_VERBOSE_LOGGING_FN_VARGS(pPlatformService_->logService(),
                                            DEBUG_LEVEL,
                                            Controls::ReceivePropertiesControlMessageFormatter(pReceivePropertiesControlMessage),
                                            "MACI %03hu %s::%s Receiver Properties Control Message",
                                            id_,
                                            pzLayerName,
                                            __func__);
          }
          break;
              
        case Controls::FrequencyControlMessage::IDENTIFIER:
          {
            pFrequencyControlMessage =
              static_cast<const Controls::FrequencyControlMessage *>(pControlMessage); 

            LOGGER_VERBOSE_LOGGING_FN_VARGS(pPlatformService_->logService(),
                                            DEBUG_LEVEL,
                                            Controls::FrequencyControlMessageFormatter(pFrequencyControlMessage),
                                            "MACI %03hu %s::%s Frequency Control Message",
                                            id_,
                                            pzLayerName,
                                            __func__);
                  
          }
                
          break;
        }
    }
  
  if(!pReceivePropertiesControlMessage || !pFrequencyControlMessage ||  
     pFrequencyControlMessage->getFrequencySegments().empty())
    {
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              ERROR_LEVEL,
                              "MACI %03hu %s::%s: phy control message not"
                              " provided from src %hu, drop",
                              id_, 
                              pzLayerName,
                              __func__,
                              pktInfo.getSource());

      commonLayerStatistics_[u8Category]->processOutbound(pkt, 
                                                          std::chrono::duration_cast<Microseconds>(Clock::now() - timeNow), 
                                                          DROP_CODE_BAD_CONTROL_INFO);
     
      //drop
      return;
    }
  else
    {
      const auto & frequencySegments = pFrequencyControlMessage->getFrequencySegments();

      TimePoint startOfReception{pReceivePropertiesControlMessage->getTxTime() +
          pReceivePropertiesControlMessage->getPropagationDelay() +
          frequencySegments.begin()->getOffset()};

      Microseconds span{pReceivePropertiesControlMessage->getSpan()};

      auto pCallback =
        new std::function<bool()>{std::bind([this,
                                             startOfReception,
                                             frequencySegments,
                                             span,
                                             timeNow](UpstreamPacket & pkt,
                                                      std::uint64_t u64SequenceNumber,
                                                      std::uint8_t u8Category)
        {
          const PacketInfo & pktInfo{pkt.getPacketInfo()};
              
          const FrequencySegment & frequencySegment{*frequencySegments.begin()};
              
          LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                 DEBUG_LEVEL,
                                 "MACI %03hu %s upstream EOR processing: src %hu, dst %hu, len %zu, freq %ju, offset %ju, duration %ju",
                                 id_,
                                 pzLayerName,
                                 pktInfo.getSource(),
                                 pktInfo.getDestination(),
                                 pkt.length(),
                                 frequencySegment.getFrequencyHz(),
                                 frequencySegment.getOffset().count(),
                                 frequencySegment.getDuration().count());
              
              
          double dNoiseFloordB{};
              
          try
            {
              // get the spectrum info for the entire span, where a span
              // is the total time between the start of the signal of the
              // earliest segment and the end of the signal of the latest
              // segment. This is not necessarily the signal duration.
              auto window = pRadioService_->spectrumService().request(frequencySegment.getFrequencyHz(),
                                                                      span,
                                                                      startOfReception);
                  
              // since we only have a single segment the span will equal the segment duration.
              // For simple noise processing we will just pull out the max noise segment, we can
              // use the maxBinNoiseFloor utility function for this. More elaborate noise window analysis
              // will require a more complex algorithm, although you should get a lot of mileage out of 
              // this utility function.
              bool bSignalInNoise{};

              std::tie(dNoiseFloordB,bSignalInNoise) =
                Utils::maxBinNoiseFloor(window,frequencySegment.getRxPowerdBm());
                 
              if(bSignalInNoise)
                {
                  LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                         ERROR_LEVEL,
                                         "MACI %03hu %s upstream EOR processing: spectrum service reporting signal in noise."
                                         " This is an unallowable noise mode. Valid PHY noise modes are: none and outofband.",
                                         id_,
                                         pzLayerName);
                  
                  commonLayerStatistics_[u8Category]->processOutbound(pkt, 
                                                                      std::chrono::duration_cast<Microseconds>(Clock::now() - timeNow), 
                                                                      DROP_CODE_BAD_CONTROL_INFO);
                  
                  return true;

                }
                  
            }
          catch(SpectrumServiceException & exp)
            {
              LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                     ERROR_LEVEL,
                                     "MACI %03hu %s upstream EOR processing: spectrum service request error: %s",
                                     id_,
                                     pzLayerName,
                                     exp.what());
              commonLayerStatistics_[u8Category]->processOutbound(pkt, 
                                                                  std::chrono::duration_cast<Microseconds>(Clock::now() - timeNow), 
                                                                  DROP_CODE_BAD_SPECTRUM_QUERY);
              // drop
              return true;
            }
              
          handleUpstreamPacket(pkt, 
                               frequencySegment.getRxPowerdBm(),
                               dNoiseFloordB,
                               u64SequenceNumber,
                               timeNow,
                               u8Category);

          return true;

        },pkt,commonMACHeader.getSequenceNumber(),u8Category)};
    
 
      auto eor = startOfReception + frequencySegments.begin()->getDuration();

      if(eor > timeNow)
        {
          // wait for end of reception to complete processing
          pPlatformService_->timerService().scheduleTimedEvent(eor,pCallback);
        }
      else
        {
          // we can process now, end of reception has past
          (*pCallback)();
          
          delete pCallback;
        }
    }
}


void 
EMANE::Models::IEEE80211ABG::MACLayer::handleUpstreamPacket(UpstreamPacket & pkt, 
                                                            double dRxPowerdBm, 
                                                            double dNoiseFloordBm,
                                                            std::uint64_t u64SequenceNumber,
                                                            const TimePoint & timeNow,
                                                            std::uint8_t u8Category)
{ 
  size_t len{pkt.stripLengthPrefixFraming()};

  const PacketInfo & pktInfo{pkt.getPacketInfo()};

  if(len && pkt.length() >= len)
    {
      MACHeaderMessage ieeeMACHeader{pkt.get(), len};

      pkt.strip(len);

      MACHeaderParams macHeaderParams{ieeeMACHeader};

      double dRxPowermW{Utils::DB_TO_MILLIWATT(dRxPowerdBm)};

      // unicast cts ctrl
      if(macHeaderParams.getMessageType() == MSG_TYPE_UNICAST_CTS_CTRL)
        {
          LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                 DEBUG_LEVEL,
                                 "MACI %03hu %s::%s: cts pkt from %hu, mac seq %ju entry seq %hu",
                                 id_,
                                 pzLayerName,
                                 __func__, 
                                 macHeaderParams.getSrcNEM(),
                                 u64SequenceNumber,
                                 macHeaderParams.getSequenceNumber());

          // update ctrl channel activity
          neighborManager_.updateCtrlChannelActivity(macHeaderParams.getSrcNEM(),                // src
                                                     macHeaderParams.getDstNEM(),                // dst (origin)
                                                     macHeaderParams.getMessageType(),           // msg type
                                                     dRxPowermW,                                 // rx power
                                                     timeNow,                                    // time
                                                     macHeaderParams.getDurationMicroseconds(),  // duration
                                                     u8Category);                                // category


          neighborMetricManager_.updateNeighborRxMetric(pktInfo.getSource(),  // nbr (src)
                                                        u64SequenceNumber,    // seq
                                                        pktInfo.getUUID(),    // uuid
                                                        timeNow);             // rx time

          // bump counter
          macStatistics_.incrementUpstreamUnicastCtsRxFromPhy();

          // done with this cts pkt
          return;
        }
      // unicast rts-cts data
      else if((macHeaderParams.getMessageType() == MSG_TYPE_UNICAST_RTS_CTS_DATA))
        {
          // for this nem
          if(macHeaderParams.getDstNEM() == id_)
            {
              // cts pkt info src is us, dst is broadcast, dscp is 0
              PacketInfo ctsinfo{id_, NEM_BROADCAST_MAC_ADDRESS, 0, Clock::now()};

              DownstreamPacket ctsPkt{ctsinfo, nullptr, 0};

              DownstreamQueueEntry entry{ctsPkt, 
                  timeNow, 
                  macConfig_.getTxOpMicroseconds(0), 
                  0, 
                  0};

              entry.durationMicroseconds_ = macHeaderParams.getDurationMicroseconds();

              sendDownstreamUnicastCts(entry, macHeaderParams.getSrcNEM());
            }

          // bump counter
          macStatistics_.incrementUpstreamUnicastRtsCtsDataRxFromPhy();
        }

      // get overhead if enabled
      Microseconds overheadMicroseconds{ENABLE_WMM_OVERHEAD ? 
          modeTiming_.getOverheadMicroseconds(u8Category) :
          Microseconds::zero()};

      // update data channel activity
      neighborManager_.updateDataChannelActivity(macHeaderParams.getSrcNEM(),                  // src
                                                 macHeaderParams.getMessageType(),             // msg type
                                                 dRxPowermW,                                   // rx power
                                                 timeNow,                                      // time
                                                 macHeaderParams.getDurationMicroseconds() +   // duration 
                                                 overheadMicroseconds,
                                                 u8Category);                                  // categroy


      if(ENABLE_TX_RETRY)
        {
          // check reception for collision and sinr
          if(!checkUpstremReception(pkt,
                                    timeNow,
                                    u64SequenceNumber,
                                    dRxPowerdBm, 
                                    dNoiseFloordBm,
                                    macHeaderParams, 
                                    macHeaderParams.getNumRetries(),
                                    u8Category))
            {  
              LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                     DEBUG_LEVEL,
                                     "MACI %03hu %s::%s: src %hu, dst %hu, reception failed, drop", 
                                     id_, 
                                     pzLayerName,
                                     __func__,  
                                     macHeaderParams.getSrcNEM(),
                                     macHeaderParams.getDstNEM());

              commonLayerStatistics_[u8Category]->processOutbound(
                                                                  pkt, 
                                                                  std::chrono::duration_cast<Microseconds>(Clock::now() - timeNow),
                                                                  DROP_CODE_SINR);

              // drop
              return;
            }
        }
      else
        {
          // check for duplicate packet based on previous sender
          if(isDuplicate(macHeaderParams.getSrcNEM(), macHeaderParams.getSequenceNumber()))
            {
              LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                     DEBUG_LEVEL,
                                     "MACI %03hu %s::%s: duplicate pkt from %hu, entry seq %hu, drop",
                                     id_,
                                     pzLayerName,
                                     __func__, 
                                     macHeaderParams.getSrcNEM(),
                                     macHeaderParams.getSequenceNumber());

              commonLayerStatistics_[u8Category]->processOutbound(pkt, 
                                                                  std::chrono::duration_cast<Microseconds>(Clock::now() - timeNow), 
                                                                  DROP_CODE_DUPLICATE);

              // drop
              return;
            }

          // assume failed
          bool bPass{};

          int tryNum{};

          // check until retry limit reached or pkt passes
          for(tryNum = 0; (tryNum <= macHeaderParams.getNumRetries()) && !bPass; ++tryNum)
            {
              // did the pkt pass
              if(checkUpstremReception(pkt, 
                                       timeNow,
                                       u64SequenceNumber,
                                       dRxPowerdBm, 
                                       dNoiseFloordBm,
                                       macHeaderParams, 
                                       tryNum,
                                       u8Category))
                {
                  LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                         DEBUG_LEVEL, "MACI %03hu %s::%s: src %hu, dst %hu, reception passed, retires %d", 
                                         id_,
                                         pzLayerName,
                                         __func__, 
                                         macHeaderParams.getSrcNEM(),
                                         macHeaderParams.getDstNEM(),
                                         tryNum);

                  // set passed flag
                  bPass = true;
                }
            }

          if(!bPass)
            {
              LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                     DEBUG_LEVEL, "MACI %03hu %s::%s: src %hu, dst %hu, reception failed in %d tries", 
                                     id_,
                                     pzLayerName,
                                     __func__,  
                                     macHeaderParams.getSrcNEM(),
                                     macHeaderParams.getDstNEM(),
                                     tryNum);

              commonLayerStatistics_[u8Category]->processOutbound(
                                                                  pkt, 
                                                                  std::chrono::duration_cast<Microseconds>(Clock::now() - timeNow), 
                                                                  DROP_CODE_SINR);

              // drop
              return;
            }
        }
   


      // check dst
      if(macHeaderParams.getDstNEM() != id_ && !isBroadcast(macHeaderParams.getDstNEM()))
        {
          if(!macConfig_.getPromiscuosEnable())
            {
              LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                     DEBUG_LEVEL,
                                     "MACI %03hu %s::%s: nexthop %hu, not this nem, promiscous mode disabled, drop", 
                                     id_,
                                     pzLayerName,
                                     __func__,
                                     macHeaderParams.getDstNEM());

              commonLayerStatistics_[u8Category]->processOutbound(
                                                                  pkt, 
                                                                  std::chrono::duration_cast<Microseconds>(Clock::now() - timeNow), 
                                                                  DROP_CODE_DST_MAC);

              // drop
              return;
            }
        }

      commonLayerStatistics_[dscpToCategory(pkt.getPacketInfo().getPriority())]->
        processOutbound(pkt,std::chrono::duration_cast<Microseconds>(Clock::now() - timeNow));
      
      sendUpstreamPacket(pkt);
      
      LOGGER_VERBOSE_LOGGING_FN_VARGS(pPlatformService_->logService(),
                                      DEBUG_LEVEL,
                                      MACHeaderParamsFormatter(&macHeaderParams),
                                      "MACI %03hu %s::%s: send upsream packet tx state %s", 
                                      id_,
                                      pzLayerName,
                                      __func__, 
                                      pTxState_->statename());

    }
  else
    {
      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                             ERROR_LEVEL,
                             "MACI %03hu %s::%s, malformed pkt, len %zd, prefix len %zd",
                             id_,
                             __func__,
                             pzLayerName,
                             pkt.length(),
                             len);
    } 
}



bool 
EMANE::Models::IEEE80211ABG::MACLayer::checkUpstremReception(UpstreamPacket & pkt,
                                                             const TimePoint & timeNow,
                                                             std::uint64_t u64SequenceNumber,
                                                             double dRxPowerdBm,
                                                             double dNoiseFloordBm,
                                                             const MACHeaderParams & rMACHeaderParams,
                                                             int tryNum,
                                                             std::uint8_t u8Category)
{
  // get the noise floor in milli watts
  double dNoiseFloorMilliWatts{Utils::DB_TO_MILLIWATT(dNoiseFloordBm)};

  double dNoiseFloorAdjustmentMilliWatts{};

  const PacketInfo & pktInfo{pkt.getPacketInfo()};

  // check for rx collision, use num retries from pkt
  COLLISION_TYPE collisionType {checkForRxCollision(rMACHeaderParams.getSrcNEM(), 
                                                    u8Category, 
                                                    tryNum)};

  // clobbered rx during tx
  if(collisionType & COLLISION_TYPE_CLOBBER_RX_DURING_TX)
    {
      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                             DEBUG_LEVEL,
                             "MACI %03hu %s::%s: clobber rx during tx, drop",
                             id_,
                             pzLayerName,
                             __func__);


      // bump counter
      isBroadcast(rMACHeaderParams.getDstNEM()) ?
        macStatistics_.incrementUpstreamBroadcastDataDiscardDueToClobberRxDuringTx() :
        macStatistics_.incrementUpstreamUnicastDataDiscardDueToClobberRxDuringTx();

      // drop
      return false;
    }

  // clobbered rx hidden busy
  if(collisionType & COLLISION_TYPE_CLOBBER_RX_HIDDEN_BUSY)
    {
      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                             DEBUG_LEVEL,
                             "MACI %03hu %s::%s: clobber rx hidden busy, drop",
                             id_,
                             pzLayerName,
                             __func__);

      // bump counter
      isBroadcast(rMACHeaderParams.getDstNEM()) ?
        macStatistics_.incrementUpstreamBroadcastDataDiscardDueToClobberRxHiddenBusy() :
        macStatistics_.incrementUpstreamUnicastDataDiscardDueToClobberRxHiddenBusy();

      // drop
      return false;
    }

  // noise common rx
  if(collisionType & COLLISION_TYPE_NOISE_COMMON_RX)
    {
      // get random rx power common
      double dRandomNoiseMilliWatts{neighborManager_.getRandomRxPowerCommonNodesMilliWatts(pktInfo.getSource())};

      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                             DEBUG_LEVEL,
                             "MACI %03hu %s::%s: common rx noise, avg %3.2f, random %3.2f dBm of "
                             "noise to noise floor %3.2f dBm",
                             id_,
                             pzLayerName,
                             __func__,
                             Utils::MILLIWATT_TO_DB(neighborManager_.getAverageRxPowerPerMessageMilliWatts()),
                             Utils::MILLIWATT_TO_DB(dRandomNoiseMilliWatts),
                             Utils::MILLIWATT_TO_DB(dNoiseFloorMilliWatts));

      // adjust noise floor
      dNoiseFloorAdjustmentMilliWatts += dRandomNoiseMilliWatts;

      // bump counter
      isBroadcast(rMACHeaderParams.getDstNEM()) ?
        macStatistics_.incrementUpstreamBroadcastNoiseRxCommon() :
        macStatistics_.incrementUpstreamUnicastNoiseRxCommon();
    }

  // noise hidden rx
  if(collisionType & COLLISION_TYPE_NOISE_HIDDEN_RX)
    {
      // get random rx power hidden
      double dRandomNoiseMilliWatts{neighborManager_.getRandomRxPowerHiddenNodesMilliWatts(pktInfo.getSource())};

      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                             DEBUG_LEVEL,
                             "MACI %03hu %s::%s: hidden rx noise, avg %3.2f, random %3.2f, dBm of "
                             "noise to noise floor %3.2f dBm",
                             id_,
                             pzLayerName,
                             __func__,
                             Utils::MILLIWATT_TO_DB(neighborManager_.getAverageRxPowerPerMessageHiddenNodesMilliWatts()),
                             Utils::MILLIWATT_TO_DB(dRandomNoiseMilliWatts),
                             Utils::MILLIWATT_TO_DB(dNoiseFloorMilliWatts));
      // adjust noise floor
      dNoiseFloorAdjustmentMilliWatts += dRandomNoiseMilliWatts;

      // bump counter
      isBroadcast(rMACHeaderParams.getDstNEM()) ?
        macStatistics_.incrementUpstreamBroadcastNoiseHiddenRx() :
        macStatistics_.incrementUpstreamUnicastNoiseHiddenRx();
    }

  double dNoiseFloorAdjusteddBm{Utils::MILLIWATT_TO_DB(dNoiseFloorMilliWatts + dNoiseFloorAdjustmentMilliWatts)};

  // check sinr for this pkt
  if(!checkPOR(dRxPowerdBm - dNoiseFloorAdjusteddBm, pkt.length(), rMACHeaderParams.getDataRateIndex()))
    {
      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                             DEBUG_LEVEL,
                             "MACI %03hu %s::%s: rxpwr %3.2f dBm, adjusted noise floor %3.2f dBm, datarate %hu, drop",
                             id_,
                             pzLayerName,
                             __func__,
                             dRxPowerdBm, 
                             dNoiseFloorAdjusteddBm,
                             rMACHeaderParams.getDataRateIndex());

      // bump counter
      isBroadcast(rMACHeaderParams.getDstNEM()) ?
        macStatistics_.incrementUpstreamBroadcastDataDiscardDueToSinr() :
        macStatistics_.incrementUpstreamUnicastDataDiscardDueToSinr();

      // drop
      return false;
    }
  else
    {
      double dSINR{dRxPowerdBm - dNoiseFloorAdjusteddBm};

      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                             DEBUG_LEVEL,
                             "MACI %03hu %s::%s: rxpwr %3.2f dBm, adjusted noise floor %3.2f dBm, sinr %3.2f, datarate %hu, pass",
                             id_,
                             pzLayerName,
                             __func__,
                             dRxPowerdBm, 
                             dNoiseFloorAdjusteddBm,
                             dSINR,
                             rMACHeaderParams.getDataRateIndex());

      neighborMetricManager_.updateNeighborRxMetric(
        pktInfo.getSource(),                                           // nbr (src)
        u64SequenceNumber,                                             // seq
        pktInfo.getUUID(),                                             // uuid
        dSINR,                                                         // sinr in dBm
        dNoiseFloorAdjustmentMilliWatts,                               // noise floor in dBm
        timeNow,                                                       // rx time
        rMACHeaderParams.getDurationMicroseconds(),                    // duration
        rMACHeaderParams.getMessageType() == MSG_TYPE_BROADCAST_DATA ? // data rate bps
        macConfig_.getBroadcastDataRateKbps(rMACHeaderParams.getDataRateIndex()) * 1000ULL :
        macConfig_.getUnicastDataRateKbps(rMACHeaderParams.getDataRateIndex())   * 1000ULL);

      // pass
      return true;
    }
}



void
EMANE::Models::IEEE80211ABG::MACLayer::processDownstreamPacket(DownstreamPacket & pkt,
                                                               const ControlMessages &)
{
  TimePoint timeNow{Clock::now()};

  const PacketInfo & pktInfo{pkt.getPacketInfo()};

  std::uint8_t u8Category{dscpToCategory(pktInfo.getPriority())};

  commonLayerStatistics_[u8Category]->processInbound(pkt);

  if(!removeToken())
    {
      commonLayerStatistics_[u8Category]->processOutbound(
                                                          pkt, 
                                                          std::chrono::duration_cast<Microseconds>(Clock::now() - timeNow), 
                                                          DROP_CODE_FLOW_CONTROL_ERROR);

      // drop
      return;
    }

  std::uint8_t u8Retries{isBroadcast(pktInfo.getDestination()) ?
      std::uint8_t{} : macConfig_.getRetryLimit(u8Category)};

  DownstreamQueueEntry entry{pkt, 
      timeNow, 
      macConfig_.getTxOpMicroseconds(u8Category), 
      u8Category, 
      u8Retries};

  // check rts cts enable
  if((macConfig_.getRtsThreshold() != 0) &&(macConfig_.getRtsThreshold() <= pkt.length()))
    {
      // set rts cts flag
      entry.bRtsCtsEnable_ = true;
    }

  if(bHasPendingDownstreamQueueEntry_)
    {
      std::vector<DownstreamQueueEntry> result{downstreamQueue_.enqueue(entry)};

      // check for discarded
      for(auto & iter : result)
        {
          commonLayerStatistics_[u8Category]->
            processOutbound(iter.pkt_, 
                            std::chrono::duration_cast<Microseconds>(Clock::now() - iter.acquireTime_), 
                            DROP_CODE_QUEUE_OVERFLOW);
          
          // drop, replace token
          addToken();
        }
    }
  else
    {
      bHasPendingDownstreamQueueEntry_ = true;

      pendingDownstreamQueueEntry_ = std::move(entry);

      auto optionalWait = pTxState_->getWaitTime(pendingDownstreamQueueEntry_);

      if(optionalWait.second)
        {
          downstreamQueueTimedEventId_ = 
            pPlatformService_->timerService().
            scheduleTimedEvent(optionalWait.first,
                               new std::function<bool()>{std::bind(&MACLayer::handleDownstreamQueueEntry,
                                                                   this)});
        }
      else
        {
          handleDownstreamQueueEntry();
        }
    }
}




void
EMANE::Models::IEEE80211ABG::MACLayer::processEvent(const EventId & eventId,
                                                    const Serialization & serialization)
{
  LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                         DEBUG_LEVEL,
                         "MACI %03hu %s::%s: event id %hu", 
                         id_,
                         pzLayerName,
                         __func__,
                         eventId);

  // check event id
  switch(eventId)
    {
    case EMANE_EVENT_IEEE80211ABG_ONEHOP_NEIGHBORS:
      {
        // the nbr manager knows how to handle these
        neighborManager_.handleOneHopNeighborsEvent(serialization);
      }
      break;
    }

  // no other events to be handled
}


void
EMANE::Models::IEEE80211ABG::MACLayer::sendDownstreamBroadcastData(DownstreamQueueEntry & entry)
{
  setEntrySequenceNumber(entry);

  // check retry logic enable
  std::uint8_t numRetries{ENABLE_TX_RETRY ? entry.numRetries_ : std::uint8_t{}};

  MACHeaderParams macHeaderParams{MSG_TYPE_BROADCAST_DATA,                      // msg type
      numRetries,                                   // num retries
      macConfig_.getBroadcastDataRateIndex(),       // data rate
      entry.u16Seq_,                                // sequence number
      id_,                                          // src
      entry.pkt_.getPacketInfo().getDestination(),  // dst
      entry.durationMicroseconds_};                 // duration 


  entry.u64DataRatebps_ = macConfig_.getBroadcastDataRateKbps() * 1000ULL;

  sendDownstreamMessage(entry, macHeaderParams);
}




void
EMANE::Models::IEEE80211ABG::MACLayer::sendDownstreamUnicastData(DownstreamQueueEntry & entry)
{
  setEntrySequenceNumber(entry);

  // check retry logic enable
  std::uint8_t numRetries{ENABLE_TX_RETRY ? entry.numRetries_ : entry.maxRetries_};

  MACHeaderParams macHeaderParams{entry.bRtsCtsEnable_ ? 
      MSG_TYPE_UNICAST_RTS_CTS_DATA : 
      MSG_TYPE_UNICAST_DATA,                       // msg type
      numRetries,                                  // num retries
      macConfig_.getUnicastDataRateIndex(),        // data rate index
      entry.u16Seq_,                               // sequence number
      id_,                                         // src
      entry.pkt_.getPacketInfo().getDestination(), // dst
      entry.durationMicroseconds_};                // duration

  entry.u64DataRatebps_ = macConfig_.getUnicastDataRateKbps() * 1000ULL;

  // send msg
  sendDownstreamMessage(entry, macHeaderParams);
}



void
EMANE::Models::IEEE80211ABG::MACLayer::sendDownstreamUnicastCts(DownstreamQueueEntry & entry, NEMId origin)
{
  setEntrySequenceNumber(entry);

  // check retry logic enable
  std::uint8_t numRetries{ENABLE_TX_RETRY ? entry.numRetries_ : entry.maxRetries_};

  MACHeaderParams macHeaderParams{MSG_TYPE_UNICAST_CTS_CTRL,  // msg type
      numRetries,                                             // num retries
      macConfig_.getUnicastDataRateIndex(),                   // data rate index
      entry.u16Seq_,                                          // sequence number
      id_,                                                    // src
      origin,                                                 // dst is the origin
      entry.durationMicroseconds_};                           // duration

  // reset duration for cts message
  entry.durationMicroseconds_ = Microseconds::zero();

  entry.u64DataRatebps_ = macConfig_.getUnicastDataRateKbps() * 1000ULL;

  LOGGER_VERBOSE_LOGGING_FN_VARGS(pPlatformService_->logService(),
                                  DEBUG_LEVEL,
                                  MACHeaderParamsFormatter(&macHeaderParams),
                                  "MACI %03hu %s::%s", 
                                  id_,
                                  pzLayerName,
                                  __func__);

  // send msg
  sendDownstreamMessage(entry, macHeaderParams);
}



void
EMANE::Models::IEEE80211ABG::MACLayer::sendDownstreamMessage(DownstreamQueueEntry & entry, 
                                                             MACHeaderParams & rMACHeaderParams)
{
  TimePoint currentTime{Clock::now()};

  // get overhead if enabled
  Microseconds overheadMicroseconds{ENABLE_WMM_OVERHEAD ? 
      modeTiming_.getOverheadMicroseconds(entry.u8Category_) :
      Microseconds::zero()};


  // update channel activity(self nem)
  neighborManager_.updateDataChannelActivity(id_,                                                 // id
                                             rMACHeaderParams.getMessageType(),                   // msg type
                                             0.0f,                                                // rx power (don't include our pwr)
                                             currentTime,                                         // current time
                                             entry.durationMicroseconds_ + overheadMicroseconds,  // duration
                                             entry.u8Category_);                                  // category
                                  


  // update queue metrics
  queueMetricManager_.updateQueueMetric(entry.u8Category_,                                        // queue id
                                        downstreamQueue_.getMaxCapacity(entry.u8Category_),       // max queue size
                                        downstreamQueue_.getDepth(entry.u8Category_),             // current queue depth
                                        downstreamQueue_.getNumOverFlow(entry.u8Category_, true), // queue discards (clear counter)
                                        std::chrono::duration_cast<Microseconds>
                                        (currentTime - entry.acquireTime_));                    // time in queue

  // update nbr tx metrics
  neighborMetricManager_.updateNeighborTxMetric(entry.pkt_.getPacketInfo().getDestination(),  // dst 
                                                entry.u64DataRatebps_,                        // data rate bps
                                                currentTime);                                 // current time


  MACHeaderMessage ieeeMACHeader{rMACHeaderParams.getMessageType(),  // msg type
      rMACHeaderParams.getNumRetries(),                              // num retries
      rMACHeaderParams.getDataRateIndex(),                           // data rate
      rMACHeaderParams.getSequenceNumber(),                          // sequence number
      rMACHeaderParams.getSrcNEM(),                                  // src
      rMACHeaderParams.getDstNEM(),                                  // dst
      rMACHeaderParams.getDurationMicroseconds()};                   // duration 

  Serialization serialization{ieeeMACHeader.serialize()};

  // prepend mac header to outgoing packet
  entry.pkt_.prepend(serialization.c_str(), serialization.size());

  // next prepend the serialization length
  entry.pkt_.prependLengthPrefixFraming(serialization.size());

  commonLayerStatistics_[entry.u8Category_]->
    processOutbound(entry.pkt_, 
                    std::chrono::duration_cast<Microseconds>(Clock::now() - entry.acquireTime_),
                    rMACHeaderParams.getMessageType() == MSG_TYPE_UNICAST_CTS_CTRL);

  // send the packet
  sendDownstreamPacket(CommonMACHeader{registrationId_, u64SequenceNumber_++},           
                       entry.pkt_,                                       
                       {Controls::FrequencyControlMessage::create(
                         0,                                                 // bandwidth (0 uses phy default)
                         {{0,rMACHeaderParams.getDurationMicroseconds()}}), // freq (0 uses phy default)
                       Controls::TimeStampControlMessage::create(entry.txTime_)}); 
}




/**
 *
 * @brief callback to change current tx state
 * 
 * @param pState new state
 * 
 */
void
EMANE::Models::IEEE80211ABG::MACLayer::changeDownstreamState(TransmissionTxState * pState)
{
  LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                         DEBUG_LEVEL,
                         "MACI %03hu %s::%s:  %s => %s",
                         id_,
                         pzLayerName,
                         __func__, 
                         pTxState_->statename(), 
                         pState->statename());

  // change state
  pTxState_ = pState;
}




EMANE::NEMId 
EMANE::Models::IEEE80211ABG::MACLayer::getId() const
{
  return id_;
}




void
EMANE::Models::IEEE80211ABG::MACLayer::setDelayTime(DownstreamQueueEntry & entry)
{
  // get current time
  TimePoint timeNow{Clock::now()};

  // estimated number of one hop nbrs
  float fNumEstimatedOneHopNeighbors{neighborManager_.getNumberOfEstimatedOneHopNeighbors()};

  // estimated number of two hop nbrs
  float fNumEstimatedTwoHopNeighbors{neighborManager_.getNumberOfEstimatedTwoHopNeighbors()};

  // estimated number of one and two hop nbrs
  float fNumEstimatedOneAndTwoHopNeighbors{fNumEstimatedOneHopNeighbors + fNumEstimatedTwoHopNeighbors};

  // total one and two hop utilization
  Microseconds totalOneAndTwoHopUtilizationMicroseconds {neighborManager_.getTotalOneHopUtilizationMicroseconds() +
      neighborManager_.getTotalTwoHopUtilizationMicroseconds()};

  // get hidden channel activity
  float fHiddenChannelActivity{neighborManager_.getHiddenChannelActivity(entry.pkt_.getPacketInfo().getDestination())};

  // avg msg duration this is for one hops nbrs
  Microseconds averageMessageDurationMicroseconds{neighborManager_.getAverageMessageDurationMicroseconds()};

  // activity timer interval
  Microseconds deltaT{macConfig_.getChannelActivityIntervalMicroseconds()};

  // amount of additional delay(if required)
  float X2{RNDZeroToOne_()};

  // get the contention window
  int iCW{modeTiming_.getContentionWindow(entry.u8Category_, entry.numRetries_)};

  // set initial pre delay
  Microseconds preDealyMicroseconds{};

  // set the initial post delay
  Microseconds postDelayMicroseconds{};

  // check number of estimated one and two hop nbrs
  if(fNumEstimatedOneAndTwoHopNeighbors > 1)
    {
      // calcuating excess overhead per neighbor in excess of 2 for the estimated average message duration
      Microseconds messageDurationOverheadExtraMicroseconds{ 
        std::chrono::duration_cast<Microseconds>(
           DoubleSeconds{(((fNumEstimatedOneAndTwoHopNeighbors - 2.0f) * 
              ((iCW * modeTiming_.getSlotSizeMicroseconds().count()) / 2.0f)) / USEC_PER_SEC_F)})};

      // probability of additional deley required
      float X1{RNDZeroToOne_()};

      float fDelayTimeFactor{getRatio(totalOneAndTwoHopUtilizationMicroseconds, deltaT)};

      if(X1 <= fDelayTimeFactor)
        {
          // get pre delay
          Microseconds nodeDelayMicroseconds{std::chrono::duration_cast<Microseconds>(DoubleSeconds{(
            floorf(X2 * fNumEstimatedOneAndTwoHopNeighbors) / USEC_PER_SEC_F)})};

          // add to the pre dealy
          preDealyMicroseconds += Microseconds{
            nodeDelayMicroseconds.count() * averageMessageDurationMicroseconds.count()};

          // if have node delay
          if(nodeDelayMicroseconds > Microseconds::zero())
            {
              // remove the overhead
              preDealyMicroseconds -= messageDurationOverheadExtraMicroseconds;
            }

          // set post delay
          postDelayMicroseconds = 
            std::chrono::duration_cast<Microseconds>(DoubleSeconds{((powf(fDelayTimeFactor, 2.0f) * 
              ((fNumEstimatedOneAndTwoHopNeighbors - 1.0f) * averageMessageDurationMicroseconds.count())) / USEC_PER_SEC_F)});
        }
    }

  // add defer time to pre delay
  preDealyMicroseconds += modeTiming_.getDeferIntervalMicroseconds(entry.u8Category_);

  // initial set flag that collision will not occur
  entry.bCollisionOccured_ = false;

  // check tx retry enable
  if(!ENABLE_TX_RETRY)
    {
      // probability of tx collision
      float X3{RNDZeroToOne_()};

      float A{neighborManager_.getLocalNodeTx()};

      Microseconds totalOneHopUtilizationMicroseconds{neighborManager_.getTotalOneHopUtilizationMicroseconds()};

      Microseconds localUtilizationMicroseconds{neighborManager_.getAllUtilizationMicroseconds(id_)};

      Microseconds remoteUtilizationMicroseconds{neighborManager_.getAllUtilizationMicroseconds(
                                                 entry.pkt_.getPacketInfo().getDestination())};

      Microseconds deltaT{macConfig_.getChannelActivityIntervalMicroseconds()};

      float fUtilizationFactorAdjusted{getRatio((totalOneHopUtilizationMicroseconds - 
                                             localUtilizationMicroseconds - 
                                             remoteUtilizationMicroseconds), deltaT)};

      // bump adjusted utilization
      if(fUtilizationFactorAdjusted > 1.0f)
        {
          fUtilizationFactorAdjusted = 1.0f;
        }

      float fPreTran = (fUtilizationFactorAdjusted * fUtilizationFactorAdjusted) *
                         (1.0 - A) * (fNumEstimatedOneHopNeighbors / iCW);

      // check probability
      if(X3 < fPreTran)
        {
          // set flag that collision will occur
          entry.bCollisionOccured_ = true;
        }
      else
        {
          // probability of hidden collision
          float X4{RNDZeroToOne_()};

          // C2
          float C2{0.1f};

          float H{fHiddenChannelActivity - C2};

          // clamp low
          if(H < 0.0f)
            {
              H = 0.0f;
            }
          // clamp high
          else if(H >(1.0f - C2))
            {
              H =(1.0f - C2);
            }
   
          // check probability
          if(X4 < H)
            {
              // set flag that collision will occur
              entry.bCollisionOccured_ = true;
            }
        }
    }
 
  // set pre delay time 
  entry.preTxDelayTime_ = timeNow + preDealyMicroseconds;

  // set post delay duration
  entry.postTxDelayMicroseconds_ = postDelayMicroseconds;
 
  LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                         DEBUG_LEVEL, 
                         "MACI %03hu %s::%s: est nbrs 1 hop %3.2f, "
                         "2 hop %3.2f, bw %lf, avg duration %lf,"
                         " pre delay %lf,  post delay %lf, tx collision %s",
                         id_,
                         pzLayerName,
                         __func__, 
                         fNumEstimatedOneHopNeighbors, 
                         fNumEstimatedTwoHopNeighbors, 
                         std::chrono::duration_cast<DoubleSeconds>(totalOneAndTwoHopUtilizationMicroseconds).count(), 
                         std::chrono::duration_cast<DoubleSeconds>(averageMessageDurationMicroseconds).count(), 
                         std::chrono::duration_cast<DoubleSeconds>(preDealyMicroseconds).count(), 
                         std::chrono::duration_cast<DoubleSeconds>(postDelayMicroseconds).count(),
                         entry.bCollisionOccured_ ? "true" : "false");
}




EMANE::Models::IEEE80211ABG::MACLayer::COLLISION_TYPE
EMANE::Models::IEEE80211ABG::MACLayer::checkForRxCollision(NEMId src, std::uint8_t u8Category, std::uint8_t u8Retries)
{
  int collisionStatus{COLLISION_TYPE_NONE};

  WMMManager::UtilizationRatioVector utilizationRatioVector{neighborManager_.getUtilizationRatios()};

  CWRatioVector cwMinVector{macConfig_.getCWMinRatioVector(u8Category)};

  float fNumEstimatedOneHopNeighbors{neighborManager_.getNumberOfEstimatedOneHopNeighbors()};

  float fNumEstimatedCommonNeighbors{neighborManager_.getNumberOfEstimatedCommonNeighbors(src)};

  float fHiddenChannelActivity{neighborManager_.getHiddenChannelActivity(src)};

  Microseconds totalOneHopUtilizationMicroseconds{neighborManager_.getTotalOneHopUtilizationMicroseconds()};

  Microseconds localUtilizationMicroseconds{neighborManager_.getAllUtilizationMicroseconds(id_)};

  Microseconds remoteUtilizationMicroseconds{neighborManager_.getAllUtilizationMicroseconds(src)};

  Microseconds deltaT{macConfig_.getChannelActivityIntervalMicroseconds()};

  float fUtilizationFactorAdjusted{getRatio((totalOneHopUtilizationMicroseconds - 
                                             localUtilizationMicroseconds - 
                                             remoteUtilizationMicroseconds), deltaT)};

  float fUtilizationFactorActual{getRatio(totalOneHopUtilizationMicroseconds, deltaT)};

  float A{neighborManager_.getLocalNodeTx()};

  float X1{RNDZeroToOne_()};

  float X2{RNDZeroToOne_()};

  float X3{RNDZeroToOne_()};

  int iCW{modeTiming_.getContentionWindow(u8Category, u8Retries)};

  float C1{};

  float P1{}, P2{}, P3{}, P4{}, PP1{}, PP2{};

  // bump adjusted utilization
  if(fUtilizationFactorAdjusted > 1.0f)
    {
      fUtilizationFactorAdjusted = 1.0f;
    }

  // cap actual utilization
  if(fUtilizationFactorActual > 1.0f)
    {
      fUtilizationFactorActual = 1.0f;
    }

  // check for estimated nbrs 
  if(fNumEstimatedOneHopNeighbors > 0.0f)
    {
      P1 = fUtilizationFactorAdjusted * fUtilizationFactorAdjusted * A *(fNumEstimatedOneHopNeighbors / iCW);
    
      if(X1 < P1)
        {
          // set rx during tx collision clobber, no need to continue
          return COLLISION_TYPE_CLOBBER_RX_DURING_TX;
        }
    }

  // check for estimated nbrs 
  if(fNumEstimatedOneHopNeighbors > 0.0f)
    {
      for(size_t u8Category = 0; u8Category < macConfig_.getNumAccessCategories(); ++u8Category)
        {
          // sum the products of local_bw_utilization * cw_min_ratio / cw_min
          PP1 += utilizationRatioVector[u8Category].second * cwMinVector[u8Category] / macConfig_.getCWMin(u8Category);

          LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                 DEBUG_LEVEL,
                                 "MACI %03hu %s::%s: idx %zu, PP1 %4.3f, LBWU %4.3f, CWMINR %3.2f, CWMIN %hhu",
                                 id_,
                                 pzLayerName,
                                 __func__,
                                 u8Category,
                                 PP1,
                                 utilizationRatioVector[u8Category].second,
                                 cwMinVector[u8Category],
                                 macConfig_.getCWMin(u8Category));
        }

      P1 = fUtilizationFactorActual * fUtilizationFactorActual * PP1;
    
      if(X1 < P1)
        {
          // set rx during tx collision clobber, no need to continue
          return COLLISION_TYPE_CLOBBER_RX_DURING_TX;
        }
    }


  // check for hidden channel activity
  if(fHiddenChannelActivity > 0.0f)
    {
      fUtilizationFactorActual = 1.0f + log10f(fUtilizationFactorActual);

      float fBWDelta{fUtilizationFactorActual - (2.0f * fHiddenChannelActivity)};

      if(fBWDelta < 0.0f)
        {
          fBWDelta = 0.0f;
        }

      P3 = 1.0f - fUtilizationFactorActual + fBWDelta + C1;

      if(X2 > P3)
        {
          P4 = 0.5f;

          if(X3 < P4)
            {
              // set hidden rx noise
              collisionStatus |= COLLISION_TYPE_NOISE_HIDDEN_RX;
            }
          else
            {
              // set rx hidden busy clobber, no need to continue
              return COLLISION_TYPE_CLOBBER_RX_HIDDEN_BUSY;
            }
        }
    }


  // check for estimated common nbrs
  if(fNumEstimatedCommonNeighbors > 2.0f)
    {
      if(iCW > fNumEstimatedCommonNeighbors)
        {
          for(size_t u8Category = 0; u8Category < macConfig_.getNumAccessCategories(); ++u8Category)
            {
              // sum the products of total_bw_utilization * cw_min_ratio / cw_min
              PP2 += utilizationRatioVector[u8Category].first * cwMinVector[u8Category] / macConfig_.getCWMin(u8Category);

              LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                     DEBUG_LEVEL,
                                     "MACI %03hu %s::%s: idx %zu, PP2 %4.3f, TBWU %4.3f, CWMINR %3.2f, CWMIN %hhu",
                                     id_,
                                     pzLayerName,
                                     __func__, 
                                     u8Category,
                                     PP2,
                                     utilizationRatioVector[u8Category].first,
                                     cwMinVector[u8Category],
                                     macConfig_.getCWMin(u8Category));
            }

          P2 = fUtilizationFactorAdjusted * fUtilizationFactorAdjusted * (fNumEstimatedCommonNeighbors - 2) * PP2;
        }
      else
        {
          P2 = 1.0f;
        }

      if(P2 >= X1)
        {
          // set common rx noise
          collisionStatus |= COLLISION_TYPE_NOISE_COMMON_RX;
        }
    }


  LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                         DEBUG_LEVEL, "MACI %03hu %s::%s: src %hu, nbrs [active %zd, est %5.3f, common %5.3f, hidden %5.3f]"
                         " hidden activity %5.3f, utilization factor [actual %5.3f, adjusted %5.3f "
                         "utilization [one hop %lf, local %lf, remote %lf] "
                         "CW %d, X1 %5.3f, X2 %5.3f, X3 %5.3f, C1 %5.3f, P1 %5.3f, P2 %5.3f, P3 %5.3f P4 %5.3f, "
                         "PP1 %5.3f, PP2 %5.3f, 0x%02X",
                         id_, pzLayerName, __func__, 
                         src,
                         neighborManager_.getTotalActiveOneHopNeighbors(),
                         fNumEstimatedOneHopNeighbors,
                         fNumEstimatedCommonNeighbors,
                         neighborManager_.getNumberOfEstimatedHiddenNeighbors(src),
                         fHiddenChannelActivity,
                         fUtilizationFactorActual,
                         fUtilizationFactorAdjusted,
                         std::chrono::duration_cast<DoubleSeconds>(totalOneHopUtilizationMicroseconds).count(),
                         std::chrono::duration_cast<DoubleSeconds>(localUtilizationMicroseconds).count(),
                         std::chrono::duration_cast<DoubleSeconds>(remoteUtilizationMicroseconds).count(),
                         iCW, X1, X2, X3, C1, P1, P2, P3, P4, PP1, PP2, collisionStatus);

  return static_cast<COLLISION_TYPE>(collisionStatus);
}


bool EMANE::Models::IEEE80211ABG::MACLayer::handleDownstreamQueueEntry()
{
  // there are two ways to end processing: schedule a timer or have no
  //  other packets in the downstream queue once you finish processing
  //  the pending packet
  while(bHasPendingDownstreamQueueEntry_)
    {
      // if there is no further processing for the pending packet
      //  update the state and see if there is another packet
      //  pending
      if(!pTxState_->process(this,pendingDownstreamQueueEntry_))
        {
          pTxState_->update(this,pendingDownstreamQueueEntry_);

          std::tie(pendingDownstreamQueueEntry_,bHasPendingDownstreamQueueEntry_) =
            downstreamQueue_.dequeue();

          addToken();
        }

      // if something is pending we need to schedule it - this could be the
      // same packet entry that we began with
      if(bHasPendingDownstreamQueueEntry_)
        {
          auto optionalWait = pTxState_->getWaitTime(pendingDownstreamQueueEntry_);

          if(optionalWait.second && optionalWait.first > Clock::now())
            {
              downstreamQueueTimedEventId_ =
                pPlatformService_->timerService().
                scheduleTimedEvent(optionalWait.first,
                                   new std::function<bool()>{std::bind(&MACLayer::handleDownstreamQueueEntry,
                                                                       this)});

              // we are finished handling the queue entry (for now)
              break;
            }
        }
    }
  
  // delete after executing
  return true;
}

void 
EMANE::Models::IEEE80211ABG::MACLayer::processTimedEvent(TimerEventId,
                                                         const TimePoint &,
                                                         const TimePoint &,
                                                         const TimePoint &,
                                                         const void *arg)
{
  auto pCallBack = reinterpret_cast<const std::function<bool()> *>(arg);
  
  if((*pCallBack)())
    {
      delete pCallBack;
    }
}


bool 
EMANE::Models::IEEE80211ABG::MACLayer::isDuplicate(NEMId src, std::uint16_t seq)
{
  TimePoint timeNow {Clock::now()};

  size_t historySize{16};

  // entry valid interval 5 seconds
  Microseconds vaildIntervalMicroseconds{std::chrono::duration_cast<Microseconds>(DoubleSeconds{5.0f})};

  auto dupIter = duplicateMap_.find(src);

  if(dupIter == duplicateMap_.end())
    {
      SequenceVector v{};

      v.reserve(historySize);

      v.push_back(SequenceEntry(seq, timeNow));

      duplicateMap_.insert(std::make_pair(src, v));

      // not a duplicate
      return false;
    }
  else
    {
      // oldest index
      size_t oldestIndex{};

      // oldest time starts as current time
      TimePoint oldestTime{timeNow};

      for(size_t idx = 0; idx < dupIter->second.size(); ++idx)
        {
          if(dupIter->second.at(idx).seq_ == seq)
            {
              // within the time window
              if((dupIter->second.at(idx).tp_ + vaildIntervalMicroseconds) >= timeNow)
                {
                  // is duplicate
                  return true;
                }
              else
                {
                  // update entry
                  dupIter->second.at(idx) = SequenceEntry(seq, timeNow);

                  // not a duplicate
                  return false;
                }
            }
          // no match
          else
            {
              // check entry age
              if(dupIter->second.at(idx).tp_ < oldestTime)
                {
                  // new oldest time
                  oldestTime = dupIter->second.at(idx).tp_;

                  // new oldest index
                  oldestIndex = idx;
                }
            }
        }

      if(dupIter->second.size() < historySize)
        {
          // add entry
          dupIter->second.push_back(SequenceEntry(seq, timeNow));
        }
      else
        {
          // replace entry
          dupIter->second.at(oldestIndex) = SequenceEntry(seq, timeNow);
        }

      // not a duplicate
      return false;
    }
}



bool 
EMANE::Models::IEEE80211ABG::MACLayer::addToken()
{
  if(macConfig_.getFlowControlEnable())
    {
      if(!flowControlManager_.addToken())
        {
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  ERROR_LEVEL,
                                  "MACI %03hu %s::%s: failed to add token",
                                  id_,
                                  pzLayerName,
                                  __func__);
          
          // failed
          return false;
        }
    }

  // success
  return true;
}



bool 
EMANE::Models::IEEE80211ABG::MACLayer::removeToken()
{
  if(macConfig_.getFlowControlEnable())
    {
      if(!flowControlManager_.removeToken())
        {
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  ERROR_LEVEL,
                                  "MACI %03hu %s::%s: failed to remove token",
                                  id_,
                                  pzLayerName,
                                  __func__);


          // failed
          return false;
        }
    }

  // success
  return true;
}



bool
EMANE::Models::IEEE80211ABG::MACLayer::checkPOR(float fSINR, size_t packetSize, std::uint16_t u16DataRateIndex)
{
  // find pcr
  float fPCR{pcrManager_.getPCR(fSINR, packetSize, u16DataRateIndex)};

  // random value from 0.0 to 1.0 inclusive
  float fRandom{RNDZeroToOne_()};

  // pcr >= random value
  bool bResult{fPCR >= fRandom};

  LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                         DEBUG_LEVEL,
                         "MACI %03hu %s::%s: sinr %3.2f, pcr %3.2f %s rand %3.3f", 
                         id_, 
                         pzLayerName,
                         __func__, 
                         fSINR,
                         fPCR,
                         bResult ? ">=" : "<",
                         fRandom);

  // return result
  return bResult;
}



std::uint8_t 
EMANE::Models::IEEE80211ABG::MACLayer::dscpToCategory(std::uint8_t dscp) const
{
  // default value is 0
  std::uint8_t u8Category{};

  if(macConfig_.getWmmEnable())
    {
      // with WMM, map the 4 access categories to different queues
      if(dscp >= 8 && dscp <= 23)
        {
          u8Category = 1;
        }
      else if(dscp >= 32 && dscp <= 47)
        {
          u8Category = 2;
        }
      else if(dscp >= 48 && dscp <= 63)
        {
          u8Category = 3;
        }
    }

  LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                         DEBUG_LEVEL,
                         "MACI %03hu %s::%s: wmm %s, dscp %hhu, category %hhu",
                         id_,
                         pzLayerName,
                         __func__,
                         (macConfig_.getWmmEnable() ? "on" : "off"),
                         dscp,
                         u8Category);

  return u8Category;
}



EMANE::Models::IEEE80211ABG::MACStatistics & 
EMANE::Models::IEEE80211ABG::MACLayer::getStatistics()
{
  return macStatistics_;
}



EMANE::Models::IEEE80211ABG::ModeTimingParameters & 
EMANE::Models::IEEE80211ABG::MACLayer::getModeTiming()
{
  return modeTiming_;
}

void 
EMANE::Models::IEEE80211ABG::MACLayer::setEntrySequenceNumber(DownstreamQueueEntry &entry)
{
  // set sequence number on first try, else retain current seq number
  if(entry.numRetries_ == 0)
    {
      entry.u16Seq_ = u16EntrySequenceNumber_++;
    }
}

DECLARE_MAC_LAYER(EMANE::Models::IEEE80211ABG::MACLayer);
