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

#include "frameworkphy.h"

#include "emane/commonphyheader.h"
#include "emane/configureexception.h"
#include "emane/spectrumserviceexception.h"

#include "emane/utils/conversionutils.h"

#include "emane/events/antennaprofileevent.h"
#include "emane/events/locationevent.h"
#include "emane/events/pathlossevent.h"

#include "emane/controls/frequencycontrolmessage.h"
#include "emane/controls/otatransmittercontrolmessage.h"
#include "emane/controls/transmittercontrolmessage.h"
#include "emane/controls/receivepropertiescontrolmessage.h"
#include "emane/controls/antennaprofilecontrolmessage.h"
#include "emane/controls/timestampcontrolmessage.h"

#include "emane/controls/frequencycontrolmessageformatter.h"
#include "emane/controls/transmittercontrolmessageformatter.h"

#include "freespacepropagationmodelalgorithm.h"
#include "tworaypropagationmodelalgorithm.h"
#include "precomputedpropagationmodelalgorithm.h"

namespace
{
  const std::uint16_t DROP_CODE_OUT_OF_BAND                 = 1;
  const std::uint16_t DROP_CODE_RX_SENSITIVITY              = 2;
  const std::uint16_t DROP_CODE_PROPAGATIONMODEL            = 3;
  const std::uint16_t DROP_CODE_GAINMANAGER_LOCATION        = 4;
  const std::uint16_t DROP_CODE_GAINMANAGER_HORIZON         = 5;
  const std::uint16_t DROP_CODE_GAINMANAGER_ANTENNAPROFILE  = 6;
  const std::uint16_t DROP_CODE_NOT_FOI                     = 7;
  const std::uint16_t DROP_CODE_SPECTRUM_CLAMP              = 8;

  EMANE::StatisticTableLabels STATISTIC_TABLE_LABELS{"Out-of-Band",
      "Rx Sensitivity",
      "Propagation Model",
      "Gain Location",
      "Gain Horizon",
      "Gain Profile",
      "Not FOI",
      "Spectrum Clamp"};
}

EMANE::FrameworkPHY::FrameworkPHY(NEMId id,
                                  PlatformServiceProvider * pPlatformService,
                                  SpectrumMonitor * pSpectrumMonitor):
  PHYLayerImplementor{id, pPlatformService},
  pSpectrumMonitor_{pSpectrumMonitor},
  gainManager_{id},
  locationManager_{id},
  u64BandwidthHz_{},
  dTxPowerdBm_{},
  u64TxFrequencyHz_{},
  dReceiverSensitivitydBm_{},
  noiseMode_{},
  u16SubId_{},
  u16TxSequenceNumber_{},
  commonLayerStatistics_{STATISTIC_TABLE_LABELS,{},"0"},
  eventTablePublisher_{id}{}

EMANE::FrameworkPHY::~FrameworkPHY(){}

void EMANE::FrameworkPHY::initialize(Registrar & registrar)
{
  auto & configRegistrar = registrar.configurationRegistrar();
  
  configRegistrar.registerNumeric<double>("fixedantennagain",
                                          EMANE::ConfigurationProperties::DEFAULT |
                                          EMANE::ConfigurationProperties::MODIFIABLE,
                                         {0},
                                         "Defines the antenna gain in dBi.");
  
  configRegistrar.registerNumeric<bool>("fixedantennagainenable",
                                        EMANE::ConfigurationProperties::DEFAULT,
                                        {true},
                                        "Defines whether fixed antenna gain is used or whether"
                                        " antenna profiles are in use.");

  configRegistrar.registerNumeric<std::uint64_t>("bandwidth",
                                                 EMANE::ConfigurationProperties::DEFAULT,
                                                 {1000000},
                                                 "Defines the center frequency bandwidth in Hz",
                                                 1);  

  configRegistrar.registerNumeric<std::uint64_t>("frequency",
                                                 EMANE::ConfigurationProperties::DEFAULT,
                                                 {2347000000},
                                                 "Defines the transmit center frequency in Hz. This"
                                                 " value is included in the Common PHY Header of all"
                                                 " transmitted OTA packets.",
                                                 1);
  
  configRegistrar.registerNumeric<std::uint64_t>("frequencyofinterest",
                                                 EMANE::ConfigurationProperties::DEFAULT,
                                                 {2347000000},
                                                 "Defines a set of frequencies in Hz that the"
                                                 " will be monitor.",
                                                 std::numeric_limits<std::uint64_t>::min(),
                                                 std::numeric_limits<std::uint64_t>::max(),
                                                 1,
                                                 std::numeric_limits<std::size_t>::max());

  configRegistrar.registerNonNumeric<std::string>("noisemode",
                                                  EMANE::ConfigurationProperties::DEFAULT,
                                                  {"all"},
                                                  "Defines the noise processing mode of operation:"
                                                  " none, all or outofband.",
                                                  1,
                                                  1,
                                                  "^(none|all|outofband)$");

  
  configRegistrar.registerNumeric<std::uint64_t>("noisebinsize",
                                                 EMANE::ConfigurationProperties::DEFAULT,
                                                 {20},
                                                 "Noise bin size in microseconds.",
                                                 1);

  configRegistrar.registerNumeric<bool>("noisemaxclampenable",
                                        EMANE::ConfigurationProperties::DEFAULT,
                                        {false},
                                        "Clamp any above max segment offset, segment duration or message"
                                        " propagation to the maximums defined by noisemaxsegmentoffset,"
                                        " noisemaxsegmentduration and noisemaxmessagepropagation. When"
                                        " disabled any packet with an above max value will be dropped.",
                                        1);


  configRegistrar.registerNumeric<std::uint64_t>("noisemaxsegmentoffset",
                                                 EMANE::ConfigurationProperties::DEFAULT,
                                                 {300000},
                                                 "Noise maximum segment offset microseconds.",
                                                 1);

  configRegistrar.registerNumeric<std::uint64_t>("noisemaxmessagepropagation",
                                                 EMANE::ConfigurationProperties::DEFAULT,
                                                 {200000},
                                                 "Noise maximum message propagation in microseconds.",
                                                 1);

  configRegistrar.registerNumeric<std::uint64_t>("noisemaxsegmentduration",
                                                 EMANE::ConfigurationProperties::DEFAULT,
                                                 {1000000},
                                                 "Noise maximum segment duration in microseconds.",
                                                 1);

  configRegistrar.registerNumeric<std::uint64_t>("timesyncthreshold",
                                                 EMANE::ConfigurationProperties::DEFAULT,
                                                 {10000},
                                                 "Time sync detection threshold in microseconds. If a received"
                                                 " OTA message is more than this threshold, the message reception"
                                                 " time will be used as the source transmission time instead of"
                                                 " the time contained in the Common PHY Header. This allows the"
                                                 " emulator to be used across distributed nodes without time sync.",
                                                 1);
 
  configRegistrar.registerNonNumeric<std::string>("propagationmodel",
                                                  EMANE::ConfigurationProperties::DEFAULT,
                                                  {"precomputed"},
                                                  "Defines the pathloss mode of operation:"
                                                  " precomputed, 2ray or freespace.",
                                                  1,
                                                  1,
                                                  "^(precomputed|2ray|freespace)$");
  
  configRegistrar.registerNumeric<double>("systemnoisefigure",
                                          EMANE::ConfigurationProperties::DEFAULT,
                                          {4.0},
                                          "Defines the system noise figure in dB.");

  configRegistrar.registerNumeric<std::uint16_t>("subid",
                                                 EMANE::ConfigurationProperties::REQUIRED,
                                                 {},
                                                 "Defines the Framework PHY subid used by multiple NEM"
                                                 " definitions. Once instantiated, these NEMs may be using the"
                                                 " same frequency. In order to differentiate between Framework"
                                                 " PHY instances for different waveforms, the subid is used as"
                                                 " part of the unique waveform identifying tuple: PHY Layer"
                                                 " Registration Id, Framework PHY subid and packet center"
                                                 " frequency.",
                                                 1);
 
  configRegistrar.registerNumeric<double>("txpower",
                                         EMANE::ConfigurationProperties::DEFAULT |
                                         EMANE::ConfigurationProperties::MODIFIABLE,
                                         {0.0},
                                          "Defines the transmit power in dBm.");

  auto & eventRegistrar = registrar.eventRegistrar();
  
  eventRegistrar.registerEvent(Events::PathlossEvent::IDENTIFIER);

  eventRegistrar.registerEvent(Events::LocationEvent::IDENTIFIER);
  
  eventRegistrar.registerEvent(Events::AntennaProfileEvent::IDENTIFIER);

  auto & statisticRegistrar = registrar.statisticRegistrar();

  commonLayerStatistics_.registerStatistics(statisticRegistrar);

  eventTablePublisher_.registerStatistics(statisticRegistrar);
}

void EMANE::FrameworkPHY::configure(const ConfigurationUpdate & update)
{
  Microseconds noiseBinSize{};
  double dSystemNoiseFiguredB{};
  Microseconds maxSegmentOffset{};
  Microseconds maxMessagePropagation{};
  Microseconds maxSegmentDuration{};
  Microseconds timeSyncThreshold{};
  bool bNoiseMaxClamp{};

  for(const auto & item : update)
    {
      if(item.first == "bandwidth")
        {
          u64BandwidthHz_ = item.second[0].asUINT64(); 
          
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::%s: %s = %ju Hz",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  u64BandwidthHz_);
        }
      else if(item.first == "fixedantennagain")
        {
          optionalFixedAntennaGaindBi_.first = item.second[0].asDouble();
          
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::%s: %s = %3.2f dBi",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                   optionalFixedAntennaGaindBi_.first);
          
        }
      else if(item.first == "fixedantennagainenable")
        {
          optionalFixedAntennaGaindBi_.second = item.second[0].asBool();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::%s: %s = %s",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  optionalFixedAntennaGaindBi_.second ? "on" : "off");
        }
      else if(item.first == "propagationmodel")
        {
          std::string sPropagationModel{item.second[0].asString()};

          // regex has already validated values
          if(sPropagationModel == "precomputed")
            {
              pPropagationModelAlgorithm_.reset(new PrecomputedPropagationModelAlgorithm{id_});
            }
          else if(sPropagationModel == "2ray")
            {
              pPropagationModelAlgorithm_.reset(new TwoRayPropagationModelAlgorithm{id_});
            }
          else
            {
              pPropagationModelAlgorithm_.reset(new FreeSpacePropagationModelAlgorithm{id_});
            }
          
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::%s: %s = %s",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  sPropagationModel.c_str());
        }
      else if(item.first == "noisemode")
        {
          std::string sNoiseMode{item.second[0].asString()};
          
          // regex has already validated values
          if(sNoiseMode == "all")
            {
              noiseMode_ = SpectrumMonitor::NoiseMode::ALL;
            }
          else if(sNoiseMode == "none")
            {
              noiseMode_ = SpectrumMonitor::NoiseMode::NONE;
            }
          else
            {
              noiseMode_ = SpectrumMonitor::NoiseMode::OUTOFBAND;
            }

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::%s: %s = %s",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  sNoiseMode.c_str());
        }
      else if(item.first == "noisebinsize")
        {
          noiseBinSize = Microseconds{item.second[0].asUINT64()};

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::%s: %s = %ju usec",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  noiseBinSize.count());
        }
      else if(item.first == "noisemaxclampenable")
        {
          bNoiseMaxClamp= item.second[0].asBool();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::%s: %s = %s",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  bNoiseMaxClamp ? "on" : "off");
        }
      else if(item.first == "noisemaxsegmentoffset")
        {
          maxSegmentOffset = Microseconds{item.second[0].asUINT64()};

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::%s: %s = %ju usec",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  maxSegmentOffset.count());
        }
      else if(item.first == "noisemaxmessagepropagation")
        {
          maxMessagePropagation = Microseconds{item.second[0].asUINT64()};

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::%s: %s = %ju usec",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  maxMessagePropagation.count());
        }
      else if(item.first == "noisemaxsegmentduration")
        {
          maxSegmentDuration = Microseconds{item.second[0].asUINT64()};
          
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::%s: %s = %ju usec",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  maxSegmentDuration.count());
        }
      else if(item.first == "timesyncthreshold")
        {
          timeSyncThreshold = Microseconds{item.second[0].asUINT64()};
          
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::%s: %s = %ju usec",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  timeSyncThreshold.count());
        }
      else if(item.first == "systemnoisefigure")
        {
          dSystemNoiseFiguredB = item.second[0].asDouble();
          
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::%s: %s = %3.2f dB",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  dSystemNoiseFiguredB);
        }
      /** [configurationregistrar-processmultiplicity-snippet] */
      else if(item.first == "frequencyofinterest")
        {
          for(const auto & any : item.second)
            {
              std::uint64_t u64Value{any.asUINT64()};

              // try to add value to foi set
              if(foi_.insert(u64Value).second)
                {
                  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                          INFO_LEVEL,
                                          "PHYI %03hu FrameworkPHY::%s: %s = %ju Hz",
                                          id_, 
                                          __func__,
                                          item.first.c_str(),
                                          u64Value);
                }
              else
                {
                  throw makeException<ConfigureException>("FrameworkPHY duplicate frequency of interest found: %ju",
                                                          u64Value);
                  
                }
            }
        }
      /** [configurationregistrar-processmultiplicity-snippet] */
      /** [configurationregistrar-processsingle-snippet] */
      else if(item.first == "txpower")
        {
          dTxPowerdBm_ = item.second[0].asDouble();
   
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::%s: %s = %3.2f dBm",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  dTxPowerdBm_);
        }
      /** [configurationregistrar-processsingle-snippet] */
      else if(item.first == "frequency")
        {
          u64TxFrequencyHz_ = item.second[0].asUINT64();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::%s: %s = %ju Hz",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  u64TxFrequencyHz_);
        }
      else if(item.first == "subid")
        {
          u16SubId_ = item.second[0].asUINT16();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::%s: %s = %hu",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  u16SubId_);
        }
      else
        {
          throw makeException<ConfigureException>("FrameworkPHY: Unexpected configuration item %s",
                                                  item.first.c_str());
        }
    }

  dReceiverSensitivitydBm_ = THERMAL_NOISE_DB + dSystemNoiseFiguredB + 10.0 * log10(u64BandwidthHz_);

  if((maxSegmentOffset + maxMessagePropagation + 2 * maxSegmentDuration) % noiseBinSize !=
     Microseconds::zero())
    {
      throw makeException<ConfigureException>("noisemaxsegmentoffset + noisemaxmessagepropagation"
                                              " + 2 * noisemaxsegmentduration not evenly divisible by the"
                                              " noisebinsize");
    }

  pSpectrumMonitor_->initialize(foi_,
                                u64BandwidthHz_,
                                Utils::DB_TO_MILLIWATT(dReceiverSensitivitydBm_),
                                noiseMode_,
                                noiseBinSize,
                                maxSegmentOffset,
                                maxMessagePropagation,
                                maxSegmentDuration,
                                timeSyncThreshold,
                                bNoiseMaxClamp);
}

void EMANE::FrameworkPHY::start()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "PHYI %03hu FrameworkPHY::%s",
                          id_,
                          __func__);
}

void EMANE::FrameworkPHY::stop()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "PHYI %03hu FrameworkPHY::%s",
                          id_,
                          __func__);
}

void EMANE::FrameworkPHY::destroy() throw()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "PHYI %03hu FrameworkPHY::%s",
                          id_,
                          __func__);

}

void EMANE::FrameworkPHY::processDownstreamControl(const ControlMessages & msgs)
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "PHYI %03hu FrameworkPHY::%s",
                          id_, 
                          __func__);

  for(const auto & pMessage : msgs)
    {
      if(pMessage->getId() == Controls::AntennaProfileControlMessage::IDENTIFIER)
        {
          const auto pAntennaProfileControlMessage =
            reinterpret_cast<const Controls::AntennaProfileControlMessage *>(pMessage);
          
          AntennaProfiles profiles{{id_,
                pAntennaProfileControlMessage->getAntennaProfileId(),
                pAntennaProfileControlMessage->getAntennaAzimuthDegrees(),
                pAntennaProfileControlMessage->getAntennaElevationDegrees()}};
          
          gainManager_.update(profiles);
          
          pPlatformService_->eventService().sendEvent(0,Events::AntennaProfileEvent{profiles});
        }
      else
        {
          LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                 DEBUG_LEVEL,
                                 "PHYI %03hu FrameworkPHY::%s, not in antenna profile mode, ignore", 
                                 id_,
                                 __func__);
        }
    }
}

void EMANE::FrameworkPHY::processDownstreamPacket(DownstreamPacket & pkt,
                                                  const ControlMessages & msgs)
{
  auto now = Clock::now();

  const PacketInfo & pktInfo{pkt.getPacketInfo()};

  LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                         DEBUG_LEVEL,
                         "PHYI %03hu FrameworkPHY::%s src %hu, dst %hu",
                         id_,
                         __func__,
                         pktInfo.getSource(),
                         pktInfo.getDestination());

  commonLayerStatistics_.processInbound(pkt);

  // use the default unless provided below
  std::uint64_t u64BandwidthHz{u64BandwidthHz_};

  // use the default unless provided below
  Transmitters transmitters{{id_, dTxPowerdBm_}};

  // use the default unless provided below
  FrequencySegments frequencySegments{{u64TxFrequencyHz_,Microseconds::zero()}};

  Controls::OTATransmitters otaTransmitters;

  TimePoint txTimeStamp{now};

  for(const auto & pMessage : msgs)
    {
      switch(pMessage->getId())
        {
        case Controls::TransmitterControlMessage::IDENTIFIER:
          {
            const auto pTransmitterControlMessage =
              static_cast<const Controls::TransmitterControlMessage *>(pMessage);


            LOGGER_VERBOSE_LOGGING_FN_VARGS(pPlatformService_->logService(),
                                            DEBUG_LEVEL,
                                            Controls::TransmitterControlMessageFormatter(pTransmitterControlMessage),
                                            "PHYI %03hu FrameworkPHY::%s Transmitter Control Message",
                                            id_,
                                            __func__);

            transmitters = pTransmitterControlMessage->getTransmitters();
             
            // build the list of OTA transmitters
            std::for_each(transmitters.begin(),
                          transmitters.end(), 
                          [&otaTransmitters](const Transmitter & transmitter)
                          {
                            otaTransmitters.insert(transmitter.getNEMId());
                          });
          }

          break;
           
        case Controls::FrequencyControlMessage::IDENTIFIER:
          {
            const auto pFrequencyControlMessage =
              static_cast<const Controls::FrequencyControlMessage *>(pMessage); 
        
            // the mac will be supplying frequency segment info so lets clear the defaults
            frequencySegments.clear();

            for(const auto & segment : pFrequencyControlMessage->getFrequencySegments())
              {
                // for convience a mac can set duration and offset without specifying frequency
                // by using 0 Hz.
                if(segment.getFrequencyHz() == 0)
                  {
                    frequencySegments.push_back({u64TxFrequencyHz_,       // use our frequency
                          segment.getDuration(), // duration
                          segment.getOffset()}); // offset
                  }
                else
                  {
                    frequencySegments.push_back(segment);
                  }
              }

            // if bandwidth provided is not 0, use the provided value, 
            // otherwise keep our default value from above
            if(pFrequencyControlMessage->getBandwidthHz() != 0)
              {
                u64BandwidthHz = pFrequencyControlMessage->getBandwidthHz();
              }

            LOGGER_VERBOSE_LOGGING_FN_VARGS(pPlatformService_->logService(),
                                            DEBUG_LEVEL,
                                            Controls::FrequencyControlMessageFormatter(pFrequencyControlMessage),
                                            "PHYI %03hu FrameworkPHY::%s Frequency Control Message",
                                            id_,
                                            __func__);
          }
           
          break;

        case Controls::AntennaProfileControlMessage::IDENTIFIER:
          {
            const auto pAntennaProfileControlMessage =
              static_cast<const Controls::AntennaProfileControlMessage *>(pMessage);
            
            LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                   DEBUG_LEVEL,
                                   "PHYI %03hu FrameworkPHY::%s Antenna Profile Control Message "
                                   "profile %hu azimuth %lf elevation %lf",
                                   id_,
                                   __func__,
                                   pAntennaProfileControlMessage->getAntennaProfileId(),
                                   pAntennaProfileControlMessage->getAntennaAzimuthDegrees(),
                                   pAntennaProfileControlMessage->getAntennaElevationDegrees());
         
            AntennaProfiles profiles{{id_,
                  pAntennaProfileControlMessage->getAntennaProfileId(),
                  pAntennaProfileControlMessage->getAntennaAzimuthDegrees(),
                  pAntennaProfileControlMessage->getAntennaElevationDegrees()}};

            gainManager_.update(profiles);
            
            pkt.attachEvent(0,Events::AntennaProfileEvent{profiles});
          }
          
          break;

        case Controls::TimeStampControlMessage::IDENTIFIER:
          {
            const auto pTimestampControlMessage =
              static_cast<const Controls::TimeStampControlMessage *>(pMessage);


            txTimeStamp = pTimestampControlMessage->getTimeStamp();

            LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                   DEBUG_LEVEL,
                                   "PHYI %03hu FrameworkPHY::%s Time Stamp Control Message "
                                   "timestamp %.6f",
                                   id_,
                                   __func__,
                                   std::chrono::duration_cast<DoubleSeconds>(txTimeStamp.time_since_epoch()).count());
          }

          break;
        }
    }


  // verify the transitters list include this nem
  if(std::find_if(transmitters.begin(),
                  transmitters.end(),
                  [this](const Transmitter & transmitter)
                  {
                    return transmitter.getNEMId() == this->id_;
                  }) == transmitters.end())
    {
      transmitters.push_back({id_,dTxPowerdBm_});
    }


  CommonPHYHeader phyHeader{REGISTERED_EMANE_PHY_FRAMEWORK, // phy registration id
      u16SubId_,
      u16TxSequenceNumber_++,
      u64BandwidthHz,   
      txTimeStamp,
      frequencySegments,
      transmitters,
      optionalFixedAntennaGaindBi_};

  LOGGER_VERBOSE_LOGGING_FN_VARGS(pPlatformService_->logService(),
                                  DEBUG_LEVEL,
                                  std::bind(&CommonPHYHeader::format, std::ref(phyHeader)),
                                  "PHYI %03hu FrameworkPHY::%s Common PHY Header",
                                  id_,
                                  __func__);
              
  ControlMessages downstreamControlMessages{};
  
  if(!otaTransmitters.empty())
    {
      downstreamControlMessages.
        push_back(Controls::OTATransmitterControlMessage::create(otaTransmitters));
    }

  commonLayerStatistics_.processOutbound(pkt,
                                         std::chrono::duration_cast<Microseconds>(Clock::now() - now));


  sendDownstreamPacket(std::move(phyHeader),pkt,std::move(downstreamControlMessages));
}


void EMANE::FrameworkPHY::processUpstreamPacket(const CommonPHYHeader & commonPHYHeader,
                                                UpstreamPacket & pkt,
                                                const ControlMessages &)
{
  processUpstreamPacket_i(Clock::now(),commonPHYHeader,pkt,{});
}

void EMANE::FrameworkPHY::processUpstreamPacket_i(const TimePoint & now,
                                                  const CommonPHYHeader & commonPHYHeader,
                                                  UpstreamPacket & pkt,
                                                  const ControlMessages &)
{
  LOGGER_VERBOSE_LOGGING_FN_VARGS(pPlatformService_->logService(),
                                  DEBUG_LEVEL,
                                  std::bind(&CommonPHYHeader::format, std::ref(commonPHYHeader)),
                                  "PHYI %03hu FrameworkPHY::%s Common PHY Header",
                                  id_,
                                  __func__);

  commonLayerStatistics_.processInbound(pkt);
 
  const  auto & pktInfo = pkt.getPacketInfo();

  bool bInBand{commonPHYHeader.getRegistrationId() == REGISTERED_EMANE_PHY_FRAMEWORK &&
      u16SubId_ == commonPHYHeader.getSubId()};

  // unless this is an in-band packet, it will only be processed if
  // noise proceccing is on
  if(bInBand || noiseMode_ != SpectrumMonitor::NoiseMode::NONE)
    {
      bool bDrop{};
      
      const auto & frequencySegments = commonPHYHeader.getFrequencySegments();

      std::vector<double> rxPowerSegments(frequencySegments.size(),0);

      Microseconds propagation{};
  
      bool bHavePropagationDelay{};

      std::vector<NEMId> transmitters{};

      for(const auto & transmitter :  commonPHYHeader.getTransmitters())
        {
          transmitters.push_back(transmitter.getNEMId());
          
          // get the location info for a pair of nodes
          auto locationPairInfoRet = locationManager_.getLocationInfo(transmitter.getNEMId());

          // get the propagation model pathloss between a pair of nodes for *each* segment
          auto pathlossInfo = (*pPropagationModelAlgorithm_)(transmitter.getNEMId(),
                                                             locationPairInfoRet.first,
                                                             frequencySegments);

          // if pathloss is available
          if(pathlossInfo.second)
            {
              // calculate the combined gain (Tx + Rx antenna gain) dBi
              // note: gain manager accesses antenna profiles, knows self node profile info
              //       if available, and is updated with all nodes profile info
              auto gainInfodBi = gainManager_.determineGain(transmitter.getNEMId(),
                                                            locationPairInfoRet.first,
                                                            optionalFixedAntennaGaindBi_,
                                                            commonPHYHeader.getOptionalFixedAntennaGaindBi());

              // if gain is available
              if(gainInfodBi.second == EMANE::GainManager::GainStatus::SUCCESS)
                {
                  std::size_t i{};
              
                  // sum up the rx power for each segment
                  for(const auto & dPathlossdB : pathlossInfo.first)
                    {
                      rxPowerSegments[i++] += Utils::DB_TO_MILLIWATT(transmitter.getPowerdBm() +
                                                                     gainInfodBi.first -
                                                                     dPathlossdB);
                    }

                  // calculate propagation delay from 1 of the transmitters
                  //  note: these are collaborative (constructive) transmissions, all 
                  //        the messages are arriving at or near the same time. Destructive
                  //        transmission should be sent as multiple messages
                  if(locationPairInfoRet.second && !bHavePropagationDelay)
                    {
                      if(locationPairInfoRet.first.getDistanceMeters() > 0.0)
                        {
                          propagation =
                            Microseconds{static_cast<std::uint64_t>(std::round(locationPairInfoRet.first.getDistanceMeters() / SOL_MPS * 1000000))};
                        }
                      
                      bHavePropagationDelay = true;
                    }
                }
              else
                {
                  // drop due to GainManager not enough info
                  bDrop = true;

                  std::uint16_t u16Code{};

                  const char * pzReason{};

                  switch(gainInfodBi.second)
                    {
                    case GainManager::GainStatus::ERROR_LOCATIONINFO:
                      pzReason = "missing location info";
                      u16Code = DROP_CODE_GAINMANAGER_LOCATION;
                      break;
                    case GainManager::GainStatus::ERROR_PROFILEINFO:
                      pzReason = "missing profile info";
                      u16Code = DROP_CODE_GAINMANAGER_ANTENNAPROFILE;
                      break;
                    case GainManager::GainStatus::ERROR_HORIZON:
                      pzReason = "below the horizon";
                      u16Code = DROP_CODE_GAINMANAGER_HORIZON;
                      break;
                    default:
                      pzReason = "unknown";
                      break;
                    }

                  commonLayerStatistics_.processOutbound(pkt, 
                                                         std::chrono::duration_cast<Microseconds>(Clock::now() - now), 
                                                         u16Code);

                  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                          DEBUG_LEVEL,
                                          "PHYI %03hu FrameworkPHY::%s transmitter %hu, src %hu, dst %hu, drop %s",
                                          id_,
                                          __func__,
                                          transmitter.getNEMId(),
                                          pktInfo.getSource(),
                                          pktInfo.getDestination(),
                                          pzReason);
                  

                  break;
                }
            }
          else
            {
              // drop due to PropagationModelAlgorithm not enough info
              bDrop = true;

              commonLayerStatistics_.processOutbound(pkt, 
                                                     std::chrono::duration_cast<Microseconds>(Clock::now() - now), 
                                                      DROP_CODE_PROPAGATIONMODEL);

              LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                      DEBUG_LEVEL,
                                      "PHYI %03hu FrameworkPHY::%s transmitter %hu, src %hu, dst %hu,"
                                      " drop propagation model missing info",
                                      id_,
                                      __func__,
                                      transmitter.getNEMId(),
                                      pktInfo.getSource(),
                                      pktInfo.getDestination());
              break;
            }
        }
      
      if(!bDrop)
        {
          
          // update the spectrum monitor with the signal information
          // note: spectrum monitor will remove any frequency segments that are
          //       below the receiver sensitivity. Any frequency segment not in
          //       the foi will cause the entire message to be treated as out-of-band
          //       regardless of the subid. Spectrum monitor will adjust SoT, propagation
          //       delay, offset and duration if out of acceptable value range.
          try
            {
              auto spectrumInfo = pSpectrumMonitor_->update(now,
                                                            commonPHYHeader.getTxTime(),
                                                            propagation,
                                                            frequencySegments,
                                                            commonPHYHeader.getBandwidthHz(),
                                                            rxPowerSegments,
                                                            bInBand,
                                                            transmitters);
              
              TimePoint sot{};
              Microseconds propagationDelay{};
              Microseconds span{};
              FrequencySegments resultingFrequencySegments{};
              bool bTreatAsInBand{};
          
              std::tie(sot,
                       propagationDelay,
                       span,
                       resultingFrequencySegments,
                       bTreatAsInBand) = spectrumInfo;
              
              if(bTreatAsInBand)
                {
                  if(!resultingFrequencySegments.empty())
                    {
                      commonLayerStatistics_.processOutbound(pkt, 
                                                             std::chrono::duration_cast<Microseconds>(Clock::now() - now)); 
                      
                      
                      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                              DEBUG_LEVEL,
                                              "PHYI %03hu FrameworkPHY::%s src %hu, dst %hu,"
                                              " pass",
                                              id_,
                                              __func__,
                                              pktInfo.getSource(),
                                              pktInfo.getDestination());
                      
                      // send to mac with associated control messages
                      sendUpstreamPacket(pkt,
                                         {Controls::FrequencyControlMessage::create(commonPHYHeader.getBandwidthHz(),
                                                                                    resultingFrequencySegments),
                                             Controls::ReceivePropertiesControlMessage::create(sot,
                                                                                               propagationDelay,
                                                                                               span,
                                                                                               dReceiverSensitivitydBm_)});
                    }
                  else
                    {
                      commonLayerStatistics_.processOutbound(pkt, 
                                                             std::chrono::duration_cast<Microseconds>(Clock::now() - now), 
                                                          DROP_CODE_RX_SENSITIVITY);
                      
                      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                              DEBUG_LEVEL,
                                              "PHYI %03hu FrameworkPHY::%s src %hu, dst %hu,"
                                              " drop below receiver sensitivity",
                                              id_,
                                              __func__,
                                              pktInfo.getSource(),
                                              pktInfo.getDestination());
                      
                    }
                }
              else
                {
                  // was this packet actuall in-band, if so at least 1 freq was not in the foi
                  if(bInBand)
                    {
                      commonLayerStatistics_.processOutbound(pkt, 
                                                             std::chrono::duration_cast<Microseconds>(Clock::now() - now), 
                                                             DROP_CODE_NOT_FOI);
                      
                      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                              DEBUG_LEVEL,
                                              "PHYI %03hu FrameworkPHY::%s src %hu, dst %hu,"
                                              " drop one or more frequencies not in frequency of interest list",
                                              id_,
                                              __func__,
                                              pktInfo.getSource(),
                                              pktInfo.getDestination());
                    }
                  else
                    {
                      // must be subid
                      commonLayerStatistics_.processOutbound(pkt, 
                                                             std::chrono::duration_cast<Microseconds>(Clock::now() - now), 
                                                             DROP_CODE_OUT_OF_BAND);
                      
                      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                              DEBUG_LEVEL,
                                              "PHYI %03hu FrameworkPHY::%s src %hu, dst %hu,"
                                              " drop out of band",
                                              id_,
                                              __func__,
                                              pktInfo.getSource(),
                                              pktInfo.getDestination());
                    }
                  
                }
            }
          catch(SpectrumServiceException & exp)
            {
              commonLayerStatistics_.processOutbound(pkt, 
                                                     std::chrono::duration_cast<Microseconds>(Clock::now() - now), 
                                                     DROP_CODE_SPECTRUM_CLAMP);
              
              LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                      DEBUG_LEVEL,
                                      "PHYI %03hu FrameworkPHY::%s src %hu, dst %hu,"
                                      " drop spectrum out of bound %s",
                                      id_,
                                      __func__,
                                      pktInfo.getSource(),
                                      pktInfo.getDestination(),
                                      exp.what());
            }
        }
      else
        {
          commonLayerStatistics_.processOutbound(pkt, 
                                                 std::chrono::duration_cast<Microseconds>(Clock::now() - now), 
                                                 DROP_CODE_OUT_OF_BAND);
          
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  DEBUG_LEVEL,
                                  "PHYI %03hu FrameworkPHY::%s src %hu, dst %hu,"
                                  " drop out of band",
                                  id_,
                                  __func__,
                                  pktInfo.getSource(),
                                  pktInfo.getDestination());
        }
    }
}

void EMANE::FrameworkPHY::processEvent(const EventId & eventId,
                                       const Serialization & serialization)
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "PHYI %03hu FrameworkPHY::%s event id: %hu",
                          id_,
                          __func__,
                          eventId);

  switch(eventId)
    {
    case Events::AntennaProfileEvent::IDENTIFIER:
      {
        Events::AntennaProfileEvent antennaProfile{serialization};
        gainManager_.update(antennaProfile.getAntennaProfiles());
        eventTablePublisher_.update(antennaProfile.getAntennaProfiles());
      }
      break;

    case Events::LocationEvent::IDENTIFIER:
      {
        Events::LocationEvent locationEvent{serialization};
        locationManager_.update(locationEvent.getLocations());
        eventTablePublisher_.update(locationEvent.getLocations());
      }
      break;

    case Events::PathlossEvent::IDENTIFIER:
      {
        Events::PathlossEvent pathlossEvent{serialization};
        pPropagationModelAlgorithm_->update(pathlossEvent.getPathlosses());
        eventTablePublisher_.update(pathlossEvent.getPathlosses());
      }
      break;
    }
}
