/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2009-2010 - DRS CenGen, LLC, Columbia, Maryland
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
 *
 */

#include "shim.h"
#include "shimheader.h"

#include "emane/events/commeffectevent.h"
#include "emane/commonphyheader.h"
#include "emane/phytypes.h"
#include "emane/configureexception.h"
#include "emane/startexception.h"


namespace
{
  const char * pzLayerName = "CommEffect::Shim";

  const std::uint32_t TIMED_EVENT_UPSTREAM_PACKET{1};

  // time event arg
  struct TimedEventArg {
     std::uint32_t       u32EventType_;
     void *              pData_;
     EMANE::TimePoint    timePoint_;
     EMANE::Microseconds delay_;

     TimedEventArg(std::uint32_t type, void * pData, EMANE::TimePoint timePoint, EMANE::Microseconds delay) :
       u32EventType_{type},
       pData_{pData},
       timePoint_{timePoint},
       delay_{delay}
      { }
   };


  const std::uint16_t DROP_CODE_DISCARD            = 1;
  const std::uint16_t DROP_CODE_GROUP_ID           = 2;
  const std::uint16_t DROP_CODE_REGISTRATION_ID    = 3;
  const std::uint16_t DROP_CODE_DST_MAC            = 4;
  const std::uint16_t DROP_CODE_BAD_MSG            = 5;
  const std::uint16_t DROP_CODE_NO_PROFILE_DATA    = 6;
  const std::uint16_t DROP_CODE_RX_BUFF            = 7;
  const std::uint16_t DROP_CODE_TIMER_ERROR        = 8;

  EMANE::StatisticTableLabels STATISTIC_TABLE_LABELS {"Effect",
                                                      "Grp Id",
                                                      "Reg Id",
                                                      "Dst MAC",
                                                      "Bad Msg",
                                                      "No Profile",
                                                      "Rx Buff",
                                                      "Timer"};
}

EMANE::Models::CommEffect::Shim::Shim(NEMId id, 
                                      PlatformServiceProvider * pPlatformService,
                                      RadioServiceProvider * pRadioService) :
  ShimLayerImplementor{id,pPlatformService,pRadioService},
  profileManager_{id, pPlatformService_},
  u32UpstreamSequenceNumber_{},
  u32DownstreamSequenceNumber_{},
  bDefaultConnectivity_{true},
  bEnablePromiscousMode_{},
  bEnableTightTimingMode_{},
  u32GroupId_{},
  receiveBufferPeriod_{},
  RNDZeroToOneHundred_{0.0f, 100.0f},
  RNG_(std::chrono::system_clock::now().time_since_epoch().count()),
  commonLayerStatistics_{STATISTIC_TABLE_LABELS,{},"0"}
{ }

EMANE::Models::CommEffect::Shim::~Shim() 
{ }


void EMANE::Models::CommEffect::Shim::initialize(Registrar & registrar) 
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL, 
                          "SHIMI %03hu %s::%s", 
                          id_,
                          pzLayerName,
                          __func__);

  auto & configRegistrar = registrar.configurationRegistrar();
  
  configRegistrar.registerNumeric<bool>("defaultconnectivitymode",
                                        EMANE::ConfigurationProperties::DEFAULT,
                                        {true},
                                        "Defines the default connectivity mode for Comm Effects. When"
                                        " set to on, full connectivity will be engaged until a valid"
                                        " Comm Effect event is received.");

  configRegistrar.registerNonNumeric<std::string>("filterfile",
                                                  ConfigurationProperties::NONE,
                                                  {},
                                                  "Defines the absolute URI of the effects filter XML"
                                                  " file which contains static filters to control"
                                                  " network impairments.");


  configRegistrar.registerNumeric<std::uint32_t>("groupid",
                                                 EMANE::ConfigurationProperties::DEFAULT,
                                                 {0},
                                                 "Defines the Comm Effect Group Id.  Only NEMs in the"
                                                 " same Comm Effect Group can communicate.");

  configRegistrar.registerNumeric<bool>("enablepromiscuousmode",
                                        ConfigurationProperties::DEFAULT,
                                        {false},
                                        "Defines whether promiscuous mode is enabled or not. If promiscuous"
                                        " mode is enabled, all received packets (intended for the given node"
                                        " or not) that pass the receive test are sent upstream to the transport.");


  configRegistrar.registerNumeric<double>("receivebufferperiod",
                                          EMANE::ConfigurationProperties::DEFAULT,
                                         {1.0},
                                         "Defines the max buffering time in seconds for packets received from an NEM."
                                          " The buffering interval for a given packet is determined by the bitrate"
                                          " for the source NEM and the packet size. Packets are then placed in a"
                                          " timed queue based on this interval and all packets that would cause the"
                                          " receive buffer period to be exceeded are discarded. A value of 0.0"
                                          " disables the limit and allows all received packets to stack up in the"
                                          " queue.");

  auto & statisticRegistrar = registrar.statisticRegistrar();

  commonLayerStatistics_.registerStatistics(statisticRegistrar);

  auto & eventRegistrar = registrar.eventRegistrar();

  eventRegistrar.registerEvent(Events::CommEffectEvent::IDENTIFIER);
}


void EMANE::Models::CommEffect::Shim::configure(const ConfigurationUpdate & update)
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL, 
                          "SHIMI %03hu %s::%s", 
                          id_,
                          pzLayerName,
                          __func__);
  
  for(const auto & item : update)
    {
      if(item.first == "defaultconnectivitymode")
        {
          bDefaultConnectivity_ = item.second[0].asBool();
          
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "SHIMI %03hu %s::%s: %s = %d", 
                                  id_,
                                  pzLayerName,
                                  __func__, 
                                  item.first.c_str(),
                                  bDefaultConnectivity_);
        }
      else if(item.first == "enablepromiscuousmode")
        {
          bEnablePromiscousMode_ = item.second[0].asBool();
          
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "SHIMI %03hu %s::%s: %s = %d", 
                                  id_,
                                  pzLayerName,
                                  __func__, 
                                  item.first.c_str(),
                                  bEnablePromiscousMode_);
        }
      else if(item.first == "groupid")
        {
          u32GroupId_ = item.second[0].asUINT32();
          
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "SHIMI %03hu %s::%s: %s = %u", 
                                  id_,
                                  pzLayerName,
                                  __func__, 
                                  item.first.c_str(),
                                  u32GroupId_);
        }
      else if(item.first == "filterfile")
        {
          sFilterFile_ = item.second[0].asString();
          
          
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "SHIMI %03hu %s::%s: %s = %s", 
                                  id_,
                                  pzLayerName,
                                  __func__, 
                                  item.first.c_str(),
                                  sFilterFile_.c_str());
        }
      else if(item.first == "receivebufferperiod")
        {
          DoubleSeconds receiveBufferPeriodSeconds{item.second[0].asDouble()};
          
          receiveBufferPeriod_ =
            std::chrono::duration_cast<Microseconds>(receiveBufferPeriodSeconds);
          
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "SHIMI %03hu %s::%s: %s = %lf", 
                                  id_,
                                  pzLayerName,
                                  __func__,
                                  item.first.c_str(),
                                  receiveBufferPeriodSeconds.count());
        }
    }
}


void EMANE::Models::CommEffect::Shim::start()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL, 
                          "SHIMI %03hu %s::%s", 
                          id_,
                          pzLayerName,
                          __func__);
  
  if(!sFilterFile_.empty() && profileManager_.load(sFilterFile_.c_str()) < 0)
    {
      throw makeException<StartException>("%s : Could not open filter file %s",
                                          pzLayerName,
                                          sFilterFile_.c_str());
    }
}


void EMANE::Models::CommEffect::Shim::stop()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL, 
                          "SHIMI %03hu %s::%s", 
                          id_,
                          pzLayerName,
                          __func__);

  // clear EOR map
  EORTimeMap_.clear();
}


void EMANE::Models::CommEffect::Shim::destroy()
  throw()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL, 
                          "SHIMI %03hu %s::%s", 
                          id_,
                          pzLayerName,
                          __func__);
}


void EMANE::Models::CommEffect::Shim::processUpstreamControl(const ControlMessages &)
{
  LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                         DEBUG_LEVEL, 
                         "SHIMI %03hu %s::%s, unexpected control message, drop", 
                         id_,
                         pzLayerName,
                         __func__);
}


void EMANE::Models::CommEffect::Shim::processDownstreamControl(const ControlMessages &)
{
  LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                         DEBUG_LEVEL, 
                         "SHIMI %03hu %s::%s, unexpected control message, drop", 
                         id_,
                         pzLayerName,
                         __func__);
}


void EMANE::Models::CommEffect::Shim::processUpstreamPacket(UpstreamPacket & pkt, const ControlMessages &)
{
  TimePoint beginTime{Clock::now()};

  commonLayerStatistics_.processInbound(pkt);

  const PacketInfo & pktInfo = pkt.getPacketInfo();

  CommonPHYHeader commonPHYHeader(pkt);

  // check phy type
  if(commonPHYHeader.getRegistrationId() != REGISTERED_EMANE_PHY_COMM_EFFECT)
    {
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              DEBUG_LEVEL, 
                              "SHIMI %03hu %s::%s, ignore phy type %hu != %hu, from %hu to %hu", 
                              id_,
                              pzLayerName,
                              __func__, 
                              commonPHYHeader.getRegistrationId(), 
                              REGISTERED_EMANE_PHY_COMM_EFFECT, 
                              pktInfo.getSource(), 
                              pktInfo.getDestination());

      commonLayerStatistics_.processOutbound(pkt, 
                                             std::chrono::duration_cast<Microseconds>(Clock::now() - beginTime), 
                                             DROP_CODE_REGISTRATION_ID);

      // drop
      return;
    }
 
  std::uint32_t u32GroupId{};

  TimePoint txTime{};

  auto len = pkt.stripLengthPrefixFraming();
  
  if(len && pkt.length() >= len)
    {
      ShimHeader header(pkt.get(),len);

      u32GroupId = header.getGroupId();
      
      txTime = header.getTxTimePoint();
      
      pkt.strip(len);
    }
  else
    {
      commonLayerStatistics_.processOutbound(pkt, 
                                             std::chrono::duration_cast<Microseconds>(Clock::now() - beginTime), 
                                             DROP_CODE_BAD_MSG);
      //drop
      return;
    }

  // check promiscous mode
  if(!bEnablePromiscousMode_)
    {
      // drop if not broadcast or for this NEM
      if((pktInfo.getDestination() != NEM_BROADCAST_MAC_ADDRESS) && (id_ != pktInfo.getDestination()))
        {
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  DEBUG_LEVEL,
                                  "SHIMI %03hu %s::%s, ignore transmission from %hu to %hu", 
                                  id_,
                                  pzLayerName,
                                  __func__,
                                  pktInfo.getSource(),
                                  pktInfo.getDestination());

          commonLayerStatistics_.processOutbound(pkt, 
                                                 std::chrono::duration_cast<Microseconds>(Clock::now() - beginTime), 
                                                 DROP_CODE_DST_MAC);
         
          // drop
          return;
        }
    }

  // check group id
  if(u32GroupId != u32GroupId_)
    {
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              DEBUG_LEVEL,
                              "SHIMI %03hu %s::%s, group id mismatch src %hu, src grp %u != our grp %u", 
                              id_,
                              pzLayerName,
                              __func__,
                              pktInfo.getSource(),
                              u32GroupId,
                              u32GroupId_);

      commonLayerStatistics_.processOutbound(pkt, 
                                             std::chrono::duration_cast<Microseconds>(Clock::now() - beginTime), 
                                             DROP_CODE_GROUP_ID);

      // drop
      return;
    }

  // lookup pkt profile data, will check filters first then event data for the source NEM
  auto ret = profileManager_.getProfileData(pkt.get(), pkt.length(), pktInfo.getSource());

  // check result
  if(!ret.second)
    {
      // no info found, check default connectivity mode
      if(bDefaultConnectivity_)
        {
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  DEBUG_LEVEL,
                                  "SHIMI %03hu %s::%s, default connectivity mode, src %hu",
                                  id_,
                                  pzLayerName,
                                  __func__,
                                  pktInfo.getSource());

          commonLayerStatistics_.processOutbound(pkt, 
                                                 std::chrono::duration_cast<Microseconds>(Clock::now() - beginTime));

          // send pkt upstream, no effects
          sendUpstreamPacket(pkt);

          // done
          return;
        }
      else
        {
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  DEBUG_LEVEL,
                                 "SHIMI %03hu %s::%s, drop, no profile data for src %hu, "
                                 "default connectivity is off",
                                 id_,
                                 pzLayerName,
                                 __func__,
                                 pktInfo.getSource());

          commonLayerStatistics_.processOutbound(pkt, 
                                                 std::chrono::duration_cast<Microseconds>(Clock::now() - beginTime), 
                                                 DROP_CODE_NO_PROFILE_DATA);

          // drop
          return;
        }
    }
  else
    {
      // get bitrate based on pkt destination unicast or broadcast
      std::uint64_t u64BitRate{pktInfo.getDestination() == NEM_BROADCAST_MAC_ADDRESS ? 
          ret.first.getBroadcastBitRate() : 
          ret.first.getUnicastBitRate()};


      // reception interval
      Microseconds rxIntervalMicroseconds{
        std::chrono::duration_cast<Microseconds>(DoubleSeconds{u64BitRate == 0 ? 
              0.0 : (pkt.length() * 8) / static_cast<double>(u64BitRate)})};
  
      // get previous EOR time for this src NEM
      auto optionalEORTime = getEORTime(pktInfo.getSource());
      
      // check backlog if receiver buffer period is set and we have history for this src NEM
      if(receiveBufferPeriod_ != Microseconds::zero() && optionalEORTime.second)
        {
          // check if there is any time remaining in the recieve window period
          if((optionalEORTime.first - beginTime) > receiveBufferPeriod_)
            {
              LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                      DEBUG_LEVEL,
                                      "SHIMI %03hu %s::%s, drop src %hu, receive buffer period exceeded %lf",
                                      id_,
                                      pzLayerName,
                                      __func__,
                                      pktInfo.getSource(),
                                      std::chrono::duration_cast<DoubleSeconds>(receiveBufferPeriod_).count());


              commonLayerStatistics_.processOutbound(pkt, 
                                             std::chrono::duration_cast<Microseconds>(Clock::now() - beginTime), 
                                             DROP_CODE_RX_BUFF);

              // drop
              return;
            }
        }

      if(optionalEORTime.first > beginTime)
        {
          // add rx interval to end of the previous EOR time
          optionalEORTime.first += rxIntervalMicroseconds;
        }
      else
        {
          // set EOR time to current time plus rx interval
          optionalEORTime.first = beginTime + rxIntervalMicroseconds;
        }

      // update the EOR for this NEM
      setEORTime(pktInfo.getSource(),optionalEORTime.first);

      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                             DEBUG_LEVEL, 
                             "SHIMI %03hu %s::%s, src %hu, dst %hu, len %zd, latency %lf,"
                             " jitter %lf, loss %f, dups %f, bitrate %ju, rxtime %lf",
                             id_, pzLayerName, __func__, 
                             pktInfo.getSource(),
                             pktInfo.getDestination(),
                             pkt.length(),
                             std::chrono::duration_cast<DoubleSeconds>(ret.first.getLatency()).count(),
                             std::chrono::duration_cast<DoubleSeconds>(ret.first.getJitter()).count(),
                             ret.first.getProbabilityLoss(),
                             ret.first.getProbabilityDuplicate(), 
                             u64BitRate,
                             std::chrono::duration_cast<DoubleSeconds>(rxIntervalMicroseconds).count());


      // get number of task(s) to create for loss and dups result is 0 or more tasks, each represent an rx frame
      const size_t taskCount = getTaskCount(ret.first.getProbabilityLoss(),
                                            ret.first.getProbabilityDuplicate());

      if(!taskCount)
        {
          commonLayerStatistics_.processOutbound(pkt, 
                                                 std::chrono::duration_cast<Microseconds>(Clock::now() - beginTime), 
                                                 DROP_CODE_DISCARD);
        }

      // create tasks
      for(size_t n = 0; n < taskCount; ++n)
        {
          // set absolute timeout delay = latency + jitter + EOR time
          TimePoint tpTimeout{ret.first.getLatency() + randomize(ret.first.getJitter()) + optionalEORTime.first};

          // if tight timing mode enabled subtract out the ota processing time of the packet (now - tx time)
          if(bEnableTightTimingMode_)
            {
              tpTimeout -= beginTime - txTime;
            }

          UpstreamPacket * pPacket{new UpstreamPacket(pkt)};
    
          TimedEventArg * pTimedEventArg{new TimedEventArg{TIMED_EVENT_UPSTREAM_PACKET, 
                                                           pPacket, 
                                                           beginTime,
                                                           std::chrono::duration_cast<Microseconds>(tpTimeout - Clock::now())}};

          // schedule the time event, id, pkt, timeout
          pPlatformService_->timerService().scheduleTimedEvent(tpTimeout, pTimedEventArg);


          LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                 DEBUG_LEVEL,
                                 "SHIMI %03hu %s::%s, enqueue task %zu, seq %u, delay %lf",
                                 id_,
                                 pzLayerName,
                                 __func__,
                                 n,
                                 u32UpstreamSequenceNumber_,
                                 std::chrono::duration_cast<DoubleSeconds>(tpTimeout - Clock::now()).count());

        }
    }
}


void EMANE::Models::CommEffect::Shim::processDownstreamPacket(DownstreamPacket & pkt,
                                                              const ControlMessages &)
{
  TimePoint beginTime{Clock::now()};

  commonLayerStatistics_.processInbound(pkt);

  LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                         DEBUG_LEVEL,
                         "SHIMI %03hu %s::%s, src %hu, dst %hu, seq %u", 
                         id_,
                         pzLayerName,
                         __func__,
                         pkt.getPacketInfo().getSource(),
                         pkt.getPacketInfo().getDestination(),
                         u32DownstreamSequenceNumber_);

  // create shim header with current time, group id, sequence number
  ShimHeader shimHeader{Clock::now(), u32GroupId_, u32DownstreamSequenceNumber_};

  Serialization serialization{shimHeader.serialize()};
   
  // prepend shim header
  pkt.prepend(serialization.c_str(),serialization.size());
   
  pkt.prependLengthPrefixFraming(serialization.size());
   
  CommonPHYHeader commonPHYHeader{EMANE::REGISTERED_EMANE_PHY_COMM_EFFECT,
      static_cast<std::uint16_t>(u32DownstreamSequenceNumber_),
      0,
      0,
      Clock::now(),
      FrequencySegments{},
      Transmitters{{id_,0}},
        {0,false}};

  // prepend phy header to outgoing packet
  commonPHYHeader.prependTo(pkt);
 
  commonLayerStatistics_.processOutbound(pkt, 
                                         std::chrono::duration_cast<Microseconds>(Clock::now() - beginTime));

  // send packet
  sendDownstreamPacket(pkt);

  // bump tx seq number
  ++u32DownstreamSequenceNumber_;
}



void EMANE::Models::CommEffect::Shim::processEvent(const EventId & eventId,
                                                   const Serialization & serialization)
{
  if(eventId == Events::CommEffectEvent::IDENTIFIER)
    {
      Events::CommEffectEvent event(serialization);
      
      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                             DEBUG_LEVEL,
                             "SHIMI %03hu %s::%s,event id %03hu",
                             id_,
                             pzLayerName,
                             __func__,
                             eventId);
      
      // load profile data entries
      profileManager_.load(event.getCommEffects());
      
      // connectivity information receieved - no longer in default mode
      bDefaultConnectivity_ = false;
    }
}



void EMANE::Models::CommEffect::Shim::processTimedEvent(TimerEventId eventId __attribute__((unused)),
                                                        const TimePoint & expireTime,
                                                        const TimePoint &,
                                                        const TimePoint &,
                                                        const void * arg)

{
  Microseconds latencyMicroseconds{std::chrono::duration_cast<Microseconds>(Clock::now() - expireTime)};

  const TimedEventArg * pTimedEventArg {reinterpret_cast<const TimedEventArg *>(arg)};

#ifdef VERY_VERBOSE_LOGGING
  LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                         DEBUG_LEVEL,
                         "SHIMI %03hu %s::%s: eventId %ld, latency %lf",
                         id_,
                         pzLayerName,
                         __func__,
                         eventId,
                         std::chrono::duration_cast<DoubleSeconds>(latencyMicroseconds).count());
#endif

  if(pTimedEventArg->u32EventType_ == TIMED_EVENT_UPSTREAM_PACKET)
    {
      // pkt ptr passed in as arg
      UpstreamPacket *pPacket{reinterpret_cast<UpstreamPacket*>(pTimedEventArg->pData_)};

      // processing delay is when we rx the pkt to now, minus the intended delay and latency
      commonLayerStatistics_.processOutbound(
        *pPacket, std::chrono::duration_cast<Microseconds>(
           Clock::now() - pTimedEventArg->timePoint_) - pTimedEventArg->delay_ - latencyMicroseconds);

      sendUpstreamPacket(*pPacket);

      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                             DEBUG_LEVEL,
                             "SHIMI %03hu %s::%s, latency %lf",
                             id_,
                             pzLayerName,
                             __func__,
                             std::chrono::duration_cast<DoubleSeconds>(latencyMicroseconds).count());

      // delete pkt
      delete pPacket;

      // delete the arg
      delete pTimedEventArg;
    }
}


size_t EMANE::Models::CommEffect::Shim::getTaskCount(float fLoss, float fDups)
{
  size_t count = 0;
  
  if(fLoss < 100)
    {
      if(fLoss > 0)
        {
          // if loss is less then random value
          if((RNDZeroToOneHundred_()) >= fLoss)
            {
              // add to count
              ++count;
            }
        }
      else
        {
          // no loss, add to count
          ++count;
        }
    }
  
  
  // you can only duplicate a packet you receive 
  if(count)
    {
      // calculate probability of duplicate
      // and add to the current count if any
      if(fDups >= 100.0f)
        {
          // add to count
          count += fDups / 100.0f;
          
          // reduce
          fDups = fmodf(fDups,100.0f);
        }
      
      if((fDups > 0.0f) && (fDups < 100.0f))
        {
          // if dups is greater/equal then random value
          if((RNDZeroToOneHundred_()) <= fDups)
            {
              // add to count
              ++count;
            }
        }
    }

  return count;
}

/* each commeffect entry may have its own jitter value so apply the range here */
EMANE::Microseconds
EMANE::Models::CommEffect::Shim::randomize(const Microseconds & duration)
{
  Microseconds randomized{};
  
  if(duration != Microseconds::zero())
    {
      // scale up and roll the dice
      randomized = Microseconds{RNG_() % (2 * duration.count())};
      
      // scale back
      randomized -= duration;
    }
  
  return randomized;
}


std::pair<EMANE::TimePoint,bool> EMANE::Models::CommEffect::Shim::getEORTime(NEMId src)
{
  // search for EOR time data based on NEM src id
  auto iter = EORTimeMap_.find(src);

  if(iter != EORTimeMap_.end())
    {
      // return EOR time
      return std::make_pair(iter->second,true);
    }

  // no history
  return std::make_pair(EMANE::TimePoint{},false);
}



void EMANE::Models::CommEffect::Shim::setEORTime(NEMId src, const TimePoint & timePoint)
{
  // search for EOR time data based on NEM src id
  auto iter = EORTimeMap_.find(src);

  if(iter != EORTimeMap_.end())
    {
      // update EOR time
      iter->second = timePoint;
    }
  else
    {
      // add new entry
      EORTimeMap_.insert(std::make_pair(src,timePoint));
    }
}

DECLARE_SHIM_LAYER(EMANE::Models::CommEffect::Shim);
