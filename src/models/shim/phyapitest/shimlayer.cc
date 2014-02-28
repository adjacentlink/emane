/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2012 - DRS CenGen, LLC, Columbia, Maryland
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
 * [DRS CenGen LLC] hereby grants to the U.S. Government a copyright license
 * to use [the original] computer software/computer software documentation
 * that is of the same scope as the rights set forth in the definition of
 * "unlimited rights" found in DFARS 252.227-7014(a)(15)(June 1995).
 */

#include "shimlayer.h"

#include "emane/controls/transmittercontrolmessage.h"
#include "emane/controls/frequencycontrolmessage.h"
#include "emane/controls/antennaprofilecontrolmessage.h"

#include "emane/controls/frequencycontrolmessageformatter.h"
#include "emane/controls/receivepropertiescontrolmessageformatter.h"

#include "emane/utils/parameterconvert.h"
#include "emane/utils/netutils.h"

#include "emane/configureexception.h"

namespace
{
  const char * pzLayerName = "PHYAPITest::ShimLayer";

  static std::uint8_t PAYLOAD_BUFFER[0xffff] = {0xBE};
}

EMANE::Models::PHYAPITest::ShimLayer::ShimLayer(NEMId id,
                                                PlatformServiceProvider * pPlatformService,
                                                RadioServiceProvider * pRadioService):
  ShimLayerImplementor{id,pPlatformService,pRadioService},
  u16PacketSize_{},
  txInterval_{},
  txTimedEventId_{},
  dst_{NEM_BROADCAST_MAC_ADDRESS},
  antennaProfileId_{},
  dAntennaAzimuthDegrees_{},        
  dAntennaElevationDegrees_{},
  fTxPowerdBm_{}
{}


EMANE::Models::PHYAPITest::ShimLayer::~ShimLayer()
{}


void EMANE::Models::PHYAPITest::ShimLayer::initialize(Registrar & registrar) 
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "SHIMI %03d %s::%s", 
                          id_,
                          pzLayerName,
                          __func__);

  auto & configRegistrar = registrar.configurationRegistrar();
  
  configRegistrar.registerNumeric<std::uint16_t>("packetsize",
                                                 ConfigurationProperties::DEFAULT,
                                                 {128},
                                                 "Packet size in bytes.");

  
  configRegistrar.registerNumeric<float>("packetrate",
                                         ConfigurationProperties::DEFAULT,
                                         {1},
                                        "Defines the transmit rate in packets per second.");

  configRegistrar.registerNumeric<std::uint16_t>("destination",
                                                 ConfigurationProperties::DEFAULT,
                                                 {NEM_BROADCAST_MAC_ADDRESS},
                                                 "Defines the destination NEM Id.",
                                                 1);

  configRegistrar.registerNumeric<std::uint64_t>("bandwidth",
                                                 ConfigurationProperties::REQUIRED,
                                                 {},
                                                 "Defines the transmitter bandwidth in Hz.",
                                                 1);  

  configRegistrar.registerNumeric<std::uint16_t>("antennaprofileid",
                                                 ConfigurationProperties::NONE,
                                                 {},
                                                 "Defines the antenna profile id. The antenna profile id is"
                                                 " used to identify the appropriate antenna pattern and"
                                                 " blockage pattern to use when calculating the receive"
                                                 " power for each packet.");

  configRegistrar.registerNumeric<double>("antennaazimuth",
                                         ConfigurationProperties::NONE,
                                         {},
                                         "Defines the antenna azimuth pointing angle in degrees.",
                                         0,
                                         360);
  
  configRegistrar.registerNumeric<double>("antennaelevation",
                                         ConfigurationProperties::NONE,
                                         {},
                                         "Defines the antenna elevation pointing angle in degrees.",
                                         -90,
                                         90);


  configRegistrar.registerNonNumeric<std::string>("frequency",
                                                  ConfigurationProperties::NONE,
                                                  {},
                                                  "Defines a list of transmit frquency segments with each item"
                                                  " containing the following 3 elements:  1) frequency (center"
                                                  " frequency Hz), 2) duration (semgent duraton in usec), and 3)"
                                                  " offset (transmit offset from TxTime in usec)."
                                                  " <Frequency>:<Duration>:<Offset>",
                                                  0,
                                                  255,
                                                  "^\\d+(\\.\\d+)?[GMK]?:\\d+(\\.\\d+)?:\\d+(\\.\\d+)?$");

  configRegistrar.registerNumeric<float>("txpower",
                                         ConfigurationProperties::DEFAULT,
                                         {0},
                                         "Defines the transmit power in dBm.");

  configRegistrar.registerNonNumeric<std::string>("transmitter",
                                                  ConfigurationProperties::NONE,
                                                  {},
                                                  "Defines a list of additional collaborative transmitters to be"
                                                  " included.  Each item in the list will include NEM ID and"
                                                  " transmit power in dBm. <NEM Id>:<Tx Power>.",
                                                  0,
                                                  255,
                                                  "^\\d+:\\d+(\\.\\d+)?$");


}


void EMANE::Models::PHYAPITest::ShimLayer::configure(const ConfigurationUpdate & update)
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                         "SHIMI %03d %s::%s", 
                          id_,
                          pzLayerName,
                          __func__);

  for(const auto & item : update)
    {
      if(item.first == "packetsize")
        {
          u16PacketSize_ = item.second[0].asUINT16();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "SHIMI %03d %s::%s %s = %hu bytes", 
                                  id_,
                                  pzLayerName,
                                  __func__, 
                                  item.first.c_str(),
                                  u16PacketSize_);
        }
      else if(item.first == "packetrate")
        {
          float fPacketRate = item.second[0].asFloat();
          
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "SHIMI %03d %s::%s %s = %3.2f pps", 
                                  id_,
                                  pzLayerName,
                                  __func__, 
                                  item.first.c_str(),
                                  fPacketRate);
          
          if(fPacketRate > 0.0)
            {
              // set the pkt interval
              txInterval_ = DoubleSeconds{1.0 / fPacketRate};
            }
        }
      else if(item.first == "destination")
        {
          dst_ =  item.second[0].asUINT16();
          
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "SHIMI %03d %s::%s %s = %hu", 
                                  id_,
                                  pzLayerName,
                                  __func__, 
                                  item.first.c_str(),
                                  dst_);
        }
      else if(item.first == "bandwidth")
        {
          u64BandwidthHz_ = item.second[0].asUINT64();
          
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "SHIMI %03d %s::%s %s = %ju",
                                  id_,
                                  pzLayerName,
                                  __func__, 
                                  item.first.c_str(),
                                  u64BandwidthHz_);
        }
      else if(item.first == "antennaprofileid")
        {
          antennaProfileId_ = item.second[0].asUINT16();
          
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "SHIMI %03d %s::%s %s = %hu", 
                                  id_,
                                  pzLayerName,
                                  __func__, 
                                  item.first.c_str(),
                                  antennaProfileId_);
        }
      else if(item.first == "antennaazimuth")
        {
          dAntennaAzimuthDegrees_ = item.second[0].asDouble();
          
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "SHIMI %03d %s::%s %s = %lf deg", 
                                  id_,
                                  pzLayerName,
                                  __func__, 
                                  item.first.c_str(),
                                  dAntennaAzimuthDegrees_);
        }
      else if(item.first == "antennaelevation")
        {
          dAntennaElevationDegrees_ = item.second[0].asDouble();
          
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "SHIMI %03d %s::%s %s = %lf deg", 
                                  id_,
                                  pzLayerName,
                                  __func__, 
                                  item.first.c_str(),
                                  dAntennaElevationDegrees_);
        }
      else if(item.first == "txpower")
        {
          fTxPowerdBm_ = item.second[0].asFloat();
          
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "SHIMI %03d %s::%s %s = %f dBm", 
                                  id_,
                                  pzLayerName,
                                  __func__, 
                                  item.first.c_str(),
                                  fTxPowerdBm_);
        }
      else if(item.first == "transmitter")
        {
          for(const auto & any : item.second)
            {
              std::string sValue = any.asString();
          
              // scan ctrl values    
              std::vector<std::string> tokens = Utils::getTokens(sValue, ":");
              
              // check num tokens nem:pwr
              if(tokens.size() == 2)
                {
                  std::uint16_t u16TxNEM = Utils::ParameterConvert(tokens[0]).toUINT16();
                  float fTxPower = Utils::ParameterConvert(tokens[1]).toFloat();
                  
                  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                          INFO_LEVEL,
                                          "SHIMI %03d %s::%s %s NEM = %hu, txpwr = %f dBm", 
                                          id_,
                                          pzLayerName,
                                          __func__, 
                                          item.first.c_str(),
                                          u16TxNEM,
                                          fTxPower);
                  
                  // add to the ati
                  transmitters_.push_back({u16TxNEM, fTxPower});
                }
              else
                {
                  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                          ERROR_LEVEL,
                                          "SHIMI %03d %s::%s invalid format %s (%s), expected (NEM:txpwr)", 
                                          id_,
                                          pzLayerName,
                                          __func__, 
                                          item.first.c_str(),
                                          sValue.c_str());
                  
                  throw makeException<ConfigurationException>("PHYAPITestShimLayer: Invalid format %s = %s",
                                                              item.first.c_str(),
                                                              sValue.c_str());
                }
            }
        }
      else if(item.first == "frequency")
        {
          for(const auto & any : item.second)
            {
              std::string sValue = any.asString();

              // scan ctrl values    
              std::vector<std::string> tokens = Utils::getTokens(sValue, ":");
              
              // check num tokens freq:duration:offset
              if(tokens.size() == 3)
                {
                  uint64_t u64TxFrequency{Utils::ParameterConvert(tokens[0]).toUINT64()};
                  DoubleSeconds duration{Utils::ParameterConvert(tokens[1]).toFloat()};
                  DoubleSeconds offset{Utils::ParameterConvert(tokens[2]).toFloat()};
                  
                  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                          INFO_LEVEL,
                                          "SHIMI %03d %s::%s %s freq %ju Hz, duration %lf, offset %lf",
                                          id_,
                                          pzLayerName,
                                          __func__, 
                                          item.first.c_str(), 
                                          u64TxFrequency,
                                          duration.count(),
                                          offset.count());
                  
                  // add to the fi
                  frequencySegments_.push_back({u64TxFrequency,
                        std::chrono::duration_cast<Microseconds>(duration),
                        std::chrono::duration_cast<Microseconds>(offset)});
                }
              else
                {
                  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                          ERROR_LEVEL,
                                          "SHIMI %03d %s::%s invalid format %s (%s), expected (freq:duration:offset)", 
                                          id_,
                                          pzLayerName,
                                          __func__, 
                                          item.first.c_str(), 
                                          sValue.c_str());
                  
                  throw makeException<ConfigurationException>("PHYAPITestShimLayer: Invalid format %s = %s",
                                                              item.first.c_str(),
                                                              sValue.c_str());
                }
            }
        }
      else
        {
          throw makeException<ConfigureException>("%s: Unexpected configuration item %s",
                                                  pzLayerName,
                                                  item.first.c_str());

        }
    }
}


void EMANE::Models::PHYAPITest::ShimLayer::start()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "SHIMI %03d %s::%s", 
                          id_,
                          pzLayerName,
                          __func__);
}


void EMANE::Models::PHYAPITest::ShimLayer::stop()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "SHIMI %03d %s::%s", 
                          id_,
                          pzLayerName,
                          __func__);

  if(txTimedEventId_ != 0)
   {
     pPlatformService_->timerService().cancelTimedEvent(txTimedEventId_);

     txTimedEventId_ = 0;
   }
}



void EMANE::Models::PHYAPITest::ShimLayer::postStart()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "SHIMI %03d %s::%s", 
                          id_,
                          pzLayerName,
                          __func__);

  if(txInterval_ > DoubleSeconds::zero())
   {
     // schedule the event id, pkt data, timeout (absolute time), interval time
     txTimedEventId_ = 
       pPlatformService_->timerService().
       scheduleTimedEvent(Clock::now() + std::chrono::duration_cast<Microseconds>(txInterval_),
                          nullptr,
                          std::chrono::duration_cast<Microseconds>(txInterval_));
   }
}



void EMANE::Models::PHYAPITest::ShimLayer::destroy()
  throw()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "SHIMI %03d %s::%s", 
                          id_,
                          pzLayerName,
                          __func__);
}



void EMANE::Models::PHYAPITest::ShimLayer::processUpstreamControl(const ControlMessages &)
{
  LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                         DEBUG_LEVEL,
                         "SHIMI %03d %s::%s, unexpected control message, drop", 
                         id_,
                         pzLayerName,
                         __func__);
}



void EMANE::Models::PHYAPITest::ShimLayer::processDownstreamControl(const ControlMessages &)
{
  LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                         DEBUG_LEVEL,
                         "SHIMI %03d %s::%s, unexpected control message, drop", 
                         id_,
                         pzLayerName,
                         __func__);
}



void EMANE::Models::PHYAPITest::ShimLayer::processUpstreamPacket(UpstreamPacket & pkt,
                                                       const ControlMessages & msgs)
{
   // get pkt info
   const PacketInfo & pktInfo = pkt.getPacketInfo();

   LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                           DEBUG_LEVEL,
                           "SHIMI %03d %s::%s src %hu dst %hu size %zu controls %zu",
                           id_,
                           pzLayerName,
                           __func__,
                           pktInfo.getSource(),
                           pktInfo.getDestination(),
                           pkt.length(),
                           msgs.size());
   
   for(const auto & pControlMessage : msgs)
     {
       switch(pControlMessage->getId())
         {
         case Controls::FrequencyControlMessage::IDENTIFIER:
           {
             const auto pFrequencyControlMessage =
               static_cast<const Controls::FrequencyControlMessage *>(pControlMessage); 
             
             LOGGER_STANDARD_LOGGING_FN_VARGS(pPlatformService_->logService(),
                                              DEBUG_LEVEL,
                                              Controls::FrequencyControlMessageFormatter{pFrequencyControlMessage},
                                              "SHIMI %03d %s::%s Frequency Control Message",
                                              id_,
                                              pzLayerName,
                                              __func__);

           }

           break;

         case Controls::ReceivePropertiesControlMessage::IDENTIFIER:
           {
             const auto pReceivePropertiesControlMessage =
               static_cast<const Controls::ReceivePropertiesControlMessage *>(pControlMessage); 
             
             LOGGER_STANDARD_LOGGING_FN_VARGS(pPlatformService_->logService(),
                                              DEBUG_LEVEL,
                                              Controls::ReceivePropertiesControlMessageFormatter{pReceivePropertiesControlMessage},
                                              "SHIMI %03d %s::%s Receive Properties Control Message",
                                              id_,
                                              pzLayerName,
                                              __func__);

           }

           break;
           
         default:
           LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                   ERROR_LEVEL,
                                   "SHIMI %03d %s::%s Unknown control message id %hu",
                                   id_,
                                   pzLayerName,
                                   __func__,
                                   pControlMessage->getId());
           break;
         }
     }
}



void EMANE::Models::PHYAPITest::ShimLayer::processDownstreamPacket(DownstreamPacket &, const ControlMessages &)
{
  LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                         DEBUG_LEVEL,
                         "SHIMI %03d %s::%s, unexpected packet, drop", 
                         id_,
                         pzLayerName,
                         __func__);
}



void EMANE::Models::PHYAPITest::ShimLayer::processTimedEvent(TimerEventId,
                                                             const TimePoint &,
                                                             const TimePoint &,
                                                             const TimePoint &,
                                                             const void *)
{
  /** [physicallayer-transmittercontrolmessage-snippet] */
  
  // the pkt
  DownstreamPacket pkt{PacketInfo{id_, dst_, 0, Clock::now()}, PAYLOAD_BUFFER, u16PacketSize_};

  ControlMessages msgs = {Controls::TransmitterControlMessage::create(transmitters_),
                          Controls::FrequencyControlMessage::create(u64BandwidthHz_,
                                                                    frequencySegments_)};

  if(antennaProfileId_)
    {
      msgs.push_back(Controls::AntennaProfileControlMessage::create(antennaProfileId_,
                                                                    dAntennaAzimuthDegrees_,
                                                                    dAntennaElevationDegrees_));
    }
  

  // send pkt to phy 
  sendDownstreamPacket(pkt, msgs);
  
  /** [physicallayer-transmittercontrolmessage-snippet] */
}



DECLARE_SHIM_LAYER(EMANE::Models::PHYAPITest::ShimLayer);
