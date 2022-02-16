/*
 * Copyright (c) 2013-2014,2016-2017,2019-2021 - Adjacent Link LLC,
 * Bridgewater, New Jersey
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
#include "emane/events/antennaprofileeventformatter.h"
#include "emane/events/locationevent.h"
#include "emane/events/locationeventformatter.h"
#include "emane/events/pathlossevent.h"
#include "emane/events/pathlosseventformatter.h"
#include "emane/events/fadingselectionevent.h"
#include "emane/events/fadingselectioneventformatter.h"

#include "emane/controls/frequencycontrolmessage.h"
#include "emane/controls/frequencyofinterestcontrolmessage.h"
#include "emane/controls/otatransmittercontrolmessage.h"
#include "emane/controls/transmittercontrolmessage.h"
#include "emane/controls/receivepropertiescontrolmessage.h"
#include "emane/controls/antennaprofilecontrolmessage.h"
#include "emane/controls/timestampcontrolmessage.h"
#include "emane/controls/txwhilerxinterferencecontrolmessage.h"
#include "emane/controls/spectrumfilteraddcontrolmessage.h"
#include "emane/controls/spectrumfilterremovecontrolmessage.h"
#include "emane/controls/spectrumfilterdatacontrolmessage.h"
#include "emane/controls/rxantennaupdatecontrolmessage.h"
#include "emane/controls/rxantennaremovecontrolmessage.h"
#include "emane/controls/rxantennaaddcontrolmessage.h"
#include "emane/controls/mimoreceivepropertiescontrolmessage.h"
#include "emane/controls/mimotransmitpropertiescontrolmessage.h"
#include "emane/controls/mimotxwhilerxinterferencecontrolmessage.h"

#include "emane/controls/frequencycontrolmessageformatter.h"
#include "emane/controls/transmittercontrolmessageformatter.h"
#include "emane/controls/frequencyofinterestcontrolmessageformatter.h"
#include "emane/controls/spectrumfilteraddcontrolmessageformatter.h"
#include "emane/controls/spectrumfilterremovecontrolmessageformatter.h"
#include "emane/controls/rxantennaupdatecontrolmessageformatter.h"
#include "emane/controls/rxantennaremovecontrolmessageformatter.h"
#include "emane/controls/rxantennaaddcontrolmessageformatter.h"

#include "freespacepropagationmodelalgorithm.h"
#include "tworaypropagationmodelalgorithm.h"
#include "precomputedpropagationmodelalgorithm.h"

#include <iterator>

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
  const std::uint16_t DROP_CODE_FADINGMANAGER_LOCATION      = 9;
  const std::uint16_t DROP_CODE_FADINGMANAGER_ALGORITHM     = 10;
  const std::uint16_t DROP_CODE_FADINGMANAGER_SELECTION     = 11;
  const std::uint16_t DROP_CODE_ANTENNA_FREQ_INDEX          = 12;
  const std::uint16_t DROP_CODE_GAINMANAGER_ANTENNA_INDEX   = 13;
  const std::uint16_t DROP_CODE_MISSING_CONTROL             = 14;

  EMANE::StatisticTableLabels STATISTIC_TABLE_LABELS{"Out-of-Band",
    "Rx Sensitivity",
    "Propagation Model",
    "Gain Location",
    "Gain Horizon",
    "Gain Profile",
    "Not FOI",
    "Spectrum Clamp",
    "Fade Location",
    "Fade Algorithm",
    "Fade Select",
    "Antenna Freq",
    "Gain Antenna",
    "Missing Control"};

  const std::string FADINGMANAGER_PREFIX{"fading."};
}

EMANE::FrameworkPHY::FrameworkPHY(NEMId id,
                                  PlatformServiceProvider * pPlatformService,
                                  SpectrumService * pSpectrumService):
  PHYLayerImplementor{id, pPlatformService},
  pSpectrumService_{pSpectrumService},
  antennaManager_{},
  locationManager_{id},
  u64BandwidthHz_{},
  dTxPowerdBm_{},
  u64TxFrequencyHz_{},
  dReceiverSensitivitydBm_{},
  noiseMode_{},
  u16SubId_{},
  u16TxSequenceNumber_{},
  commonLayerStatistics_{STATISTIC_TABLE_LABELS,{},"0"},
  eventTablePublisher_{id},
  noiseBinSize_{},
  maxSegmentOffset_{},
  maxMessagePropagation_{},
  maxSegmentDuration_{},
  timeSyncThreshold_{},
  bNoiseMaxClamp_{},
  dSystemNoiseFiguredB_{},
  pTimeSyncThresholdRewrite_{},
  pGainCacheHit_{},
  pGainCacheMiss_{},
  fadingManager_{id, pPlatformService,FADINGMANAGER_PREFIX},
  bExcludeSameSubIdFromFilter_{},
  compatibilityMode_{CompatibilityMode::MODE_1},
  processingPool_{},
  u16ProccssingPoolSize_{},
  bStatsReceivePowerTableEnable_{},
  bStatsObservedPowerTableEnable_{},
  bRxSensitivityPromiscuousModeEnable_{},
  bDopplerShiftEnable_{},
  spectralMaskIndex_{DEFAULT_SPECTRAL_MASK_INDEX}{}

EMANE::FrameworkPHY::~FrameworkPHY(){}

void EMANE::FrameworkPHY::initialize(Registrar & registrar)
{
  auto & configRegistrar = registrar.configurationRegistrar();

  configRegistrar.registerNumeric<double>("fixedantennagain",
                                          EMANE::ConfigurationProperties::DEFAULT |
                                          EMANE::ConfigurationProperties::MODIFIABLE,
                                          {0},
                                          "Defines the antenna gain in dBi and is valid only when"
                                          " fixedantennagainenable is enabled.");

  configRegistrar.registerNumeric<bool>("fixedantennagainenable",
                                        EMANE::ConfigurationProperties::DEFAULT,
                                        {true},
                                        "Defines whether fixed antenna gain is used or whether"
                                        " antenna profiles are in use.");

  configRegistrar.registerNumeric<std::uint64_t>("bandwidth",
                                                 EMANE::ConfigurationProperties::DEFAULT,
                                                 {1000000},
                                                 "Defines receiver bandwidth in Hz and also serves as the"
                                                 " default bandwidth for OTA transmissions when not provided"
                                                 " by the MAC.",
                                                 1);

  configRegistrar.registerNumeric<std::uint64_t>("frequency",
                                                 EMANE::ConfigurationProperties::DEFAULT,
                                                 {2347000000},
                                                 "Defines the default transmit center frequency in Hz when not"
                                                 " provided by the MAC. This value is included in the Common PHY"
                                                 " Header of all transmitted OTA packets.",
                                                 1);

  configRegistrar.registerNumeric<std::uint64_t>("frequencyofinterest",
                                                 EMANE::ConfigurationProperties::DEFAULT,
                                                 {2347000000},
                                                 "Defines a set of center frequencies in Hz that are monitored"
                                                 " for reception as either in-band or out-of-band.",
                                                 std::numeric_limits<std::uint64_t>::min(),
                                                 std::numeric_limits<std::uint64_t>::max(),
                                                 1,
                                                 std::numeric_limits<std::size_t>::max());

  configRegistrar.registerNonNumeric<std::string>("noisemode",
                                                  EMANE::ConfigurationProperties::DEFAULT,
                                                  {"all"},
                                                  "Defines the noise processing mode of operation:"
                                                  " none, all, outofband or passthrough.",
                                                  1,
                                                  1,
                                                  "^(none|all|outofband|passthrough)$");


  configRegistrar.registerNumeric<std::uint64_t>("noisebinsize",
                                                 EMANE::ConfigurationProperties::DEFAULT,
                                                 {20},
                                                 "Defines the noise bin size in microseconds and translates"
                                                 " into timing accuracy associated with aligning the start and"
                                                 " end of reception times of multiple packets for modeling of"
                                                 " interference effects.",
                                                 1);

  configRegistrar.registerNumeric<bool>("noisemaxclampenable",
                                        EMANE::ConfigurationProperties::DEFAULT,
                                        {false},
                                        "Defines whether segment offset, segment duration and message"
                                        " propagation associated with a received packet will be clamped"
                                        " to their respective maximums defined by noisemaxsegmentoffset,"
                                        " noisemaxsegmentduration and noisemaxmessagepropagation. When"
                                        " disabled, any packet with an above max value will be dropped.");


  configRegistrar.registerNumeric<std::uint64_t>("noisemaxsegmentoffset",
                                                 EMANE::ConfigurationProperties::DEFAULT,
                                                 {300000},
                                                 "Noise maximum segment offset in microseconds.",
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
                                                 "Defines the time sync detection threshold in microseconds."
                                                 " If a received OTA message is more than this threshold, the"
                                                 " message reception time will be used as the source transmission"
                                                 " time instead of the time contained in the Common PHY Header."
                                                 " This allows the emulator to be used across distributed nodes"
                                                 " without time sync.",
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
                                          "Defines the system noise figure in dB and is used to determine the"
                                          " receiver sensitivity.");

  configRegistrar.registerNumeric<std::uint16_t>("subid",
                                                 EMANE::ConfigurationProperties::REQUIRED,
                                                 {},
                                                 "Defines the emulator PHY subid used by multiple NEM"
                                                 " definitions. Once instantiated, these NEMs may be using the"
                                                 " same frequency. In order to differentiate between emulator"
                                                 " PHY instances for different waveforms, the subid is used as"
                                                 " part of the unique waveform identifying tuple: PHY Layer"
                                                 " Registration Id, emulator PHY subid and packet center"
                                                 " frequency.",
                                                 1);

  configRegistrar.registerNumeric<double>("txpower",
                                          EMANE::ConfigurationProperties::DEFAULT |
                                          EMANE::ConfigurationProperties::MODIFIABLE,
                                          {0.0},
                                          "Defines the transmit power in dBm.");

  configRegistrar.registerNumeric<bool>("excludesamesubidfromfilterenable",
                                        EMANE::ConfigurationProperties::DEFAULT,
                                        {true},
                                        "Defines whether over-the-air (downstream) messages with a subid"
                                        " matching the emulator PHY subid (inband) should be processed"
                                        " for filter inclusion.");

  configRegistrar.registerNumeric<std::uint16_t>("compatibilitymode",
                                                 EMANE::ConfigurationProperties::DEFAULT,
                                                 {1},
                                                 "Defines the physical layer compatibility mode.",
                                                 1,
                                                 2,
                                                 1,
                                                 1);

  configRegistrar.registerNumeric<std::uint16_t>("processingpoolsize",
                                                 EMANE::ConfigurationProperties::DEFAULT,
                                                 {0},
                                                 "Defines the number of processing pool threads. If > 2, pool"
                                                 " threads are used to process receive paths per receive"
                                                 " antenna. Using a processing pool does not guarantee increased"
                                                 " performance. The processing pool can reduce the amount of"
                                                 " processing time for an upstream message that contains a large"
                                                 " number of frequency segments and/or a large number of transmit"
                                                 " antenna (MIMO). Without a processing pool receive paths are"
                                                 " calculated serially in a loop. There is a threshold where"
                                                 " serial processing is faster than the context switching of the"
                                                 " thread pool.  Additionally, if the number of cores available to"
                                                 " a running emane process is less than the processing pool size"
                                                 " worse performance may be encountered.",
                                                 std::numeric_limits<std::uint16_t>::min(),
                                                 std::numeric_limits<std::uint16_t>::max(), 1, 1,
                                                 "^(0|[2-9]|[1-9]\\d+)$");

  configRegistrar.registerNumeric<bool>("stats.receivepowertableenable",
                                        EMANE::ConfigurationProperties::DEFAULT,
                                        {true},
                                        "Defines whether the receive power table will be populated. Large number"
                                        " of antenna (MIMO) and/or frequency segments will increases processing"
                                        " load when populating.");

  configRegistrar.registerNumeric<bool>("stats.observedpowertableenable",
                                        EMANE::ConfigurationProperties::DEFAULT,
                                        {true},
                                        "Defines whether the observed power table will be populated. Large number"
                                        " of antenna (MIMO) and/or frequency segments will increases processing"
                                        " load when populating.");

  configRegistrar.registerNumeric<bool>("rxsensitivitypromiscuousmodeenable",
                                        EMANE::ConfigurationProperties::DEFAULT,
                                        {false},
                                        "Defines whether over-the-air messages are sent upstream if below receiver"
                                        " sensitivity. Compatibility mode  > 1 only. Messages sent upstream"
                                        " without a MIMOReceivePropertiesControlMessage.");

  configRegistrar.registerNumeric<bool>("dopplershiftenable",
                                        EMANE::ConfigurationProperties::DEFAULT,
                                        {true},
                                        "Defines whether to perform Doppler shift processing when location and"
                                        " velocity information is known for both the transmitter and receiver.");

  configRegistrar.registerNumeric<SpectralMaskIndex>("spectralmaskindex",
                                                     EMANE::ConfigurationProperties::DEFAULT,
                                                     {DEFAULT_SPECTRAL_MASK_INDEX},
                                                     "Defines the spectral mask index used for all transmissions."
                                                     " Set to 0 to use the emulator default square spectral mask.");

  /** [eventservice-registerevent-snippet] */
  auto & eventRegistrar = registrar.eventRegistrar();

  eventRegistrar.registerEvent(Events::PathlossEvent::IDENTIFIER);

  eventRegistrar.registerEvent(Events::LocationEvent::IDENTIFIER);

  eventRegistrar.registerEvent(Events::AntennaProfileEvent::IDENTIFIER);

  eventRegistrar.registerEvent(Events::FadingSelectionEvent::IDENTIFIER);

  /** [eventservice-registerevent-snippet] */

  auto & statisticRegistrar = registrar.statisticRegistrar();

  commonLayerStatistics_.registerStatistics(statisticRegistrar);

  eventTablePublisher_.registerStatistics(statisticRegistrar);

  receivePowerTablePublisher_.registerStatistics(statisticRegistrar);

  observedPowerTablePublisher_.registerStatistics(statisticRegistrar);

  pTimeSyncThresholdRewrite_ =
    statisticRegistrar.registerNumeric<std::uint64_t>("numTimeSyncThresholdRewrite",
                                                      StatisticProperties::CLEARABLE);
  pGainCacheHit_ =
    statisticRegistrar.registerNumeric<std::uint64_t>("numGainCacheHit",
                                                      StatisticProperties::CLEARABLE);

  pGainCacheMiss_ =
    statisticRegistrar.registerNumeric<std::uint64_t>("numGainCacheMiss",
                                                      StatisticProperties::CLEARABLE);

  fadingManager_.initialize(registrar);
}

void EMANE::FrameworkPHY::configure(const ConfigurationUpdate & update)
{
  ConfigurationUpdate fadingManagerConfiguration{};

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
              noiseMode_ = NoiseMode::ALL;
            }
          else if(sNoiseMode == "none")
            {
              noiseMode_ = NoiseMode::NONE;
            }
          else if(sNoiseMode == "passthrough")
            {
              noiseMode_ = NoiseMode::PASSTHROUGH;
            }
          else
            {
              noiseMode_ = NoiseMode::OUTOFBAND;
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
          noiseBinSize_ = Microseconds{item.second[0].asUINT64()};

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::%s: %s = %ju usec",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  noiseBinSize_.count());
        }
      else if(item.first == "noisemaxclampenable")
        {
          bNoiseMaxClamp_ = item.second[0].asBool();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::%s: %s = %s",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  bNoiseMaxClamp_ ? "on" : "off");
        }
      else if(item.first == "noisemaxsegmentoffset")
        {
          maxSegmentOffset_ = Microseconds{item.second[0].asUINT64()};

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::%s: %s = %ju usec",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  maxSegmentOffset_.count());
        }
      else if(item.first == "noisemaxmessagepropagation")
        {
          maxMessagePropagation_ = Microseconds{item.second[0].asUINT64()};

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::%s: %s = %ju usec",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  maxMessagePropagation_.count());
        }
      else if(item.first == "noisemaxsegmentduration")
        {
          maxSegmentDuration_ = Microseconds{item.second[0].asUINT64()};

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::%s: %s = %ju usec",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  maxSegmentDuration_.count());
        }
      else if(item.first == "timesyncthreshold")
        {
          timeSyncThreshold_ = Microseconds{item.second[0].asUINT64()};

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::%s: %s = %ju usec",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  timeSyncThreshold_.count());
        }
      else if(item.first == "systemnoisefigure")
        {
          dSystemNoiseFiguredB_ = item.second[0].asDouble();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::%s: %s = %3.2f dB",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  dSystemNoiseFiguredB_);
        }
      /** [configurationregistrar-processmultiplicity-snippet] */
      else if(item.first == "frequencyofinterest")
        {
          foi_.clear();

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
      else if(item.first == "excludesamesubidfromfilterenable")
        {
          bExcludeSameSubIdFromFilter_ = item.second[0].asBool();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::%s: %s = %s",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  bExcludeSameSubIdFromFilter_ ? "on" : "off");
        }
      else if(item.first == "compatibilitymode")
        {
          std::uint16_t u16CompatibilityMode = item.second[0].asUINT16();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::%s: %s = %hu",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  u16CompatibilityMode);

          switch(u16CompatibilityMode)
            {
            case 1:
              compatibilityMode_ = CompatibilityMode::MODE_1;
              break;
            default:
              compatibilityMode_ = CompatibilityMode::MODE_2;
              break;
            };
        }
      else if(item.first == "processingpoolsize")
        {
          u16ProccssingPoolSize_ = item.second[0].asUINT16();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::%s: %s = %hu",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  u16ProccssingPoolSize_);
        }
      else if(item.first == "stats.receivepowertableenable")
        {
          bStatsReceivePowerTableEnable_ = item.second[0].asBool();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::%s: %s = %s",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  bStatsReceivePowerTableEnable_ ? "on" : "off");
        }
      else if(item.first == "stats.observedpowertableenable")
        {
          bStatsObservedPowerTableEnable_ = item.second[0].asBool();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::%s: %s = %s",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  bStatsObservedPowerTableEnable_ ? "on" : "off");
        }
      else if(item.first == "rxsensitivitypromiscuousmodeenable")
        {
          bRxSensitivityPromiscuousModeEnable_ = item.second[0].asBool();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::%s: %s = %s",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  bRxSensitivityPromiscuousModeEnable_ ? "on" : "off");
        }
      else if(item.first == "dopplershiftenable")
        {
          bDopplerShiftEnable_ = item.second[0].asBool();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::%s: %s = %s",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  bDopplerShiftEnable_ ? "on" : "off");
        }
      else if(item.first == "spectralmaskindex")
        {
          spectralMaskIndex_ = item.second[0].asUINT16();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::%s: %s = %hu",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  spectralMaskIndex_);
        }
      else
        {
          if(!item.first.compare(0,FADINGMANAGER_PREFIX.size(),FADINGMANAGER_PREFIX))
            {
              fadingManagerConfiguration.push_back(item);
            }
          else
            {
              throw makeException<ConfigureException>("FrameworkPHY: Unexpected configuration item %s",
                                                      item.first.c_str());
            }
        }
    }

  fadingManager_.configure(fadingManagerConfiguration);

  dReceiverSensitivitydBm_ = THERMAL_NOISE_DB + dSystemNoiseFiguredB_ + 10.0 * log10(u64BandwidthHz_);

  if((maxSegmentOffset_ + maxMessagePropagation_ + 2 * maxSegmentDuration_) % noiseBinSize_ !=
     Microseconds::zero())
    {
      throw makeException<ConfigureException>("noisemaxsegmentoffset + noisemaxmessagepropagation"
                                              " + 2 * noisemaxsegmentduration not evenly divisible by the"
                                              " noisebinsize");
    }

  pSpectrumService_->initialize(u16SubId_,
                                noiseMode_,
                                noiseBinSize_,
                                maxSegmentOffset_,
                                maxMessagePropagation_,
                                maxSegmentDuration_,
                                timeSyncThreshold_,
                                bNoiseMaxClamp_,
                                bExcludeSameSubIdFromFilter_);

}

void EMANE::FrameworkPHY::start()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "PHYI %03hu FrameworkPHY::%s",
                          id_,
                          __func__);

  if(u16ProccssingPoolSize_)
    {
      processingPool_.start(u16ProccssingPoolSize_);
    }
}

void EMANE::FrameworkPHY::stop()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "PHYI %03hu FrameworkPHY::%s",
                          id_,
                          __func__);

  if(processingPool_.isRunning())
    {
      processingPool_.stop();
    }
}

void EMANE::FrameworkPHY::destroy() throw()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "PHYI %03hu FrameworkPHY::%s",
                          id_,
                          __func__);

}

void EMANE::FrameworkPHY::processConfiguration(const ConfigurationUpdate & update)
{
  ConfigurationUpdate fadingManagerConfiguration{};

  for(const auto & item : update)
    {
      if(item.first == "fixedantennagain")
        {
          optionalFixedAntennaGaindBi_.first = item.second[0].asDouble();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "PHYI %03hu FrameworkPHY::%s: %s = %3.2f dBi",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  optionalFixedAntennaGaindBi_.first);

          if(optionalFixedAntennaGaindBi_.second)
            {
              auto rxAntenna = Antenna::createIdealOmni(DEFAULT_ANTENNA_INDEX,
                                                        optionalFixedAntennaGaindBi_.first);

              antennaManager_.update(id_,rxAntenna);
            }
        }
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
      else
        {
          if(!item.first.compare(0,FADINGMANAGER_PREFIX.size(),FADINGMANAGER_PREFIX))
            {
              fadingManagerConfiguration.push_back(item);
            }
        }
    }

  fadingManager_.modify(fadingManagerConfiguration);
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
      switch(pMessage->getId())
        {
        case Controls::AntennaProfileControlMessage::IDENTIFIER:
          if(compatibilityMode_ == CompatibilityMode::MODE_1)
            {
              const auto pAntennaProfileControlMessage =
                reinterpret_cast<const Controls::AntennaProfileControlMessage *>(pMessage);

              /** [eventservice-sendevent-snippet] */
              Events::AntennaProfiles profiles{{id_,
                  pAntennaProfileControlMessage->getAntennaProfileId(),
                  pAntennaProfileControlMessage->getAntennaAzimuthDegrees(),
                  pAntennaProfileControlMessage->getAntennaElevationDegrees()}};

              antennaManager_.update(profiles);

              pPlatformService_->eventService().sendEvent(0,Events::AntennaProfileEvent{profiles});
              /** [eventservice-sendevent-snippet] */
            }
          else
            {
              LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                     ERROR_LEVEL,
                                     "PHYI %03hu FrameworkPHY::%s, ignoring compatibility"
                                     " mode 1 AntennaProfileControlMessage. Use"
                                     " RxAntennaUpdateControlMessage for compatibility mode 2.",
                                     id_,
                                     __func__);
            }
          break;

        case Controls::FrequencyOfInterestControlMessage::IDENTIFIER:
          if(compatibilityMode_ == CompatibilityMode::MODE_1)
            {
              const auto pFrequencyOfInterestControlMessage =
                reinterpret_cast<const Controls::FrequencyOfInterestControlMessage *>(pMessage);

              LOGGER_VERBOSE_LOGGING_FN_VARGS(pPlatformService_->logService(),
                                              DEBUG_LEVEL,
                                              Controls::FrequencyOfInterestControlMessageFormatter(pFrequencyOfInterestControlMessage),
                                              "PHYI %03hu FrameworkPHY::%s Frequency of Interest Control Message",
                                              id_,
                                              __func__);

              if(pFrequencyOfInterestControlMessage->getBandwidthHz())
                {
                  u64BandwidthHz_ = pFrequencyOfInterestControlMessage->getBandwidthHz();
                }

              dReceiverSensitivitydBm_ =
                THERMAL_NOISE_DB + dSystemNoiseFiguredB_ + 10.0 * log10(u64BandwidthHz_);

              createDefaultAntennaIfNeeded();

              pSpectrumService_->resetSpectrumMonitor(DEFAULT_ANTENNA_INDEX,
                                                      pFrequencyOfInterestControlMessage->getFrequencySet(),
                                                      u64BandwidthHz_,
                                                      Utils::DB_TO_MILLIWATT(dReceiverSensitivitydBm_));
            }
          else
            {
              LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                     ERROR_LEVEL,
                                     "PHYI %03hu FrameworkPHY::%s, ignoring compatibility"
                                     " mode 1 FrequencyOfInterestControlMessage. Use"
                                     " RxAntennaRemoveControlMessage/RxAntennaAddControlMessage"
                                     " for compatibility mode 2.",
                                     id_,
                                     __func__);

            }
          break;

        case Controls::SpectrumFilterAddControlMessage::IDENTIFIER:
          {
            const auto pSpectrumFilterAddControlMessage =
              reinterpret_cast<const Controls::SpectrumFilterAddControlMessage *>(pMessage);

            LOGGER_VERBOSE_LOGGING_FN_VARGS(pPlatformService_->logService(),
                                            DEBUG_LEVEL,
                                            Controls::SpectrumFilterAddControlMessageFormatter(pSpectrumFilterAddControlMessage),
                                            "PHYI %03hu FrameworkPHY::%s Spectrum Filter Add Control Message",
                                            id_,
                                            __func__);

            if(compatibilityMode_ == CompatibilityMode::MODE_1)
              {
                createDefaultAntennaIfNeeded();
              }

            pSpectrumService_->initializeFilter(pSpectrumFilterAddControlMessage->getAntennaIndex(),
                                                pSpectrumFilterAddControlMessage->getFilterIndex(),
                                                pSpectrumFilterAddControlMessage->getFrequencyHz(),
                                                pSpectrumFilterAddControlMessage->getBandwidthHz(),
                                                pSpectrumFilterAddControlMessage->getSubBandBinSizeHz(),
                                                pSpectrumFilterAddControlMessage->getFilterMatchCriterion()->clone());
          }
          break;

        case Controls::SpectrumFilterRemoveControlMessage::IDENTIFIER:
          {
            const auto pSpectrumFilterRemoveControlMessage =
              reinterpret_cast<const Controls::SpectrumFilterRemoveControlMessage *>(pMessage);

            LOGGER_VERBOSE_LOGGING_FN_VARGS(pPlatformService_->logService(),
                                            DEBUG_LEVEL,
                                            Controls::SpectrumFilterRemoveControlMessageFormatter(pSpectrumFilterRemoveControlMessage),
                                            "PHYI %03hu FrameworkPHY::%s Spectrum Filter Remove Control Message",
                                            id_,
                                            __func__);

            pSpectrumService_->removeFilter(pSpectrumFilterRemoveControlMessage->getAntennaIndex(),
                                            pSpectrumFilterRemoveControlMessage->getFilterIndex());
          }
          break;

        case Controls::RxAntennaUpdateControlMessage::IDENTIFIER:
          if(compatibilityMode_ == CompatibilityMode::MODE_2)
            {
              const auto pRxAntennaUpdateControlMessage =
                reinterpret_cast<const Controls::RxAntennaUpdateControlMessage *>(pMessage);

              LOGGER_VERBOSE_LOGGING_FN_VARGS(pPlatformService_->logService(),
                                              DEBUG_LEVEL,
                                              Controls::RxAntennaUpdateControlMessageFormatter(pRxAntennaUpdateControlMessage),
                                              "PHYI %03hu FrameworkPHY::%s Rx Antenna Update Control Message",
                                              id_,
                                              __func__);

              auto rxAntenna = pRxAntennaUpdateControlMessage->getAntenna();

              // for convenience, if a radio model adds a default
              // antenna, the appropriate antenna will be created
              // based on whether fixed gain (ideal omni)
              // configuration is present.
              if(rxAntenna.isDefault())
                {
                  rxAntenna = optionalFixedAntennaGaindBi_.second ?
                    Antenna::createIdealOmni(DEFAULT_ANTENNA_INDEX,
                                             optionalFixedAntennaGaindBi_.first) :
                    Antenna::createProfileDefined(DEFAULT_ANTENNA_INDEX);
                }

              antennaManager_.update(id_,rxAntenna);
            }
          else
            {
              LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                     ERROR_LEVEL,
                                     "PHYI %03hu FrameworkPHY::%s, ignoring compatibility"
                                     " mode > 1 RxAntennaUpdateControlMessage. Use"
                                     " AntennaProfileControlMessage for compatibility mode 1.",
                                     id_,
                                     __func__);
            }
          break;
        case Controls::RxAntennaAddControlMessage::IDENTIFIER:
          if(compatibilityMode_ == CompatibilityMode::MODE_2)
            {
              const auto pRxAntennaAddControlMessage =
                reinterpret_cast<const Controls::RxAntennaAddControlMessage *>(pMessage);

              LOGGER_VERBOSE_LOGGING_FN_VARGS(pPlatformService_->logService(),
                                              DEBUG_LEVEL,
                                              Controls::RxAntennaAddControlMessageFormatter(pRxAntennaAddControlMessage),
                                              "PHYI %03hu FrameworkPHY::%s Rx Antenna Add Control Message",
                                              id_,
                                              __func__);

              auto rxAntenna = pRxAntennaAddControlMessage->getAntenna();

              // for convenience, if a radio model adds a default
              // antenna, the appropriate antenna will be created
              // based on whether fixed gain (ideal omni)
              // configuration is present.
              if(rxAntenna.isDefault())
                {
                  rxAntenna = optionalFixedAntennaGaindBi_.second ?
                    Antenna::createIdealOmni(DEFAULT_ANTENNA_INDEX,
                                             optionalFixedAntennaGaindBi_.first) :
                    Antenna::createProfileDefined(DEFAULT_ANTENNA_INDEX);
                }

              std::uint64_t u64BandwidthHz{u64BandwidthHz_};

              double dReceiverSensitivitydBm{dReceiverSensitivitydBm_};

              // for convenience, if a radio model adds an antenna
              // with a bandwidth of 0Hz, the bandwidth set via
              // configure will be used
              if(rxAntenna.getBandwidthHz())
                {
                  u64BandwidthHz = rxAntenna.getBandwidthHz();
                  dReceiverSensitivitydBm =  THERMAL_NOISE_DB + dSystemNoiseFiguredB_ + 10.0 * log10(u64BandwidthHz);
                }

              auto & foi = foi_;

              // for convenience, if a radio model adds an antenna
              // with an empty frequency set, the foi set via
              // configure will be used
              if(!pRxAntennaAddControlMessage->getFrequencyOfInterestSet().empty())
                {
                  foi = pRxAntennaAddControlMessage->getFrequencyOfInterestSet();
                }

              antennaManager_.update(id_,rxAntenna);

              auto pSpectrumMonitor = pSpectrumService_->addSpectrumMonitor(rxAntenna.getIndex(),
                                                                            foi,
                                                                            u64BandwidthHz,
                                                                            Utils::DB_TO_MILLIWATT(dReceiverSensitivitydBm));
              receiveProcessors_.emplace(rxAntenna.getIndex(),
                                         std::unique_ptr<ReceiveProcessor>(new ReceiveProcessor{id_,
                                                                                                u16SubId_,
                                                                                                rxAntenna.getIndex(),
                                                                                                antennaManager_,
                                                                                                pSpectrumMonitor,
                                                                                                pPropagationModelAlgorithm_.get(),
                                                                                                fadingManager_.createFadingAlgorithmStore(),
                                                                                                bStatsReceivePowerTableEnable_,
                                                                                                bStatsObservedPowerTableEnable_,
                                                                                                bDopplerShiftEnable_}));
            }
          else
            {
              LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                     ERROR_LEVEL,
                                     "PHYI %03hu FrameworkPHY::%s, ignoring compatibility"
                                     " mode 2 RxAntennaAddControlMessage.",
                                     id_,
                                     __func__);
            }
          break;

        case Controls::RxAntennaRemoveControlMessage::IDENTIFIER:
          if(compatibilityMode_ == CompatibilityMode::MODE_2)
            {
              const auto pRxAntennaRemoveControlMessage =
                reinterpret_cast<const Controls::RxAntennaRemoveControlMessage *>(pMessage);

              LOGGER_VERBOSE_LOGGING_FN_VARGS(pPlatformService_->logService(),
                                              DEBUG_LEVEL,
                                              Controls::RxAntennaRemoveControlMessageFormatter(pRxAntennaRemoveControlMessage),
                                              "PHYI %03hu FrameworkPHY::%s Rx Antenna Remove Control Message",
                                              id_,
                                              __func__);

              receiveProcessors_.erase(pRxAntennaRemoveControlMessage->getAntennaIndex());

              antennaManager_.remove(id_,pRxAntennaRemoveControlMessage->getAntennaIndex());

              pSpectrumService_->removeSpectrumMonitor(pRxAntennaRemoveControlMessage->getAntennaIndex());
            }
          else
            {
              LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                     ERROR_LEVEL,
                                     "PHYI %03hu FrameworkPHY::%s, ignoring compatibility"
                                     " mode 2 RxAntennaUpdateControlMessage. Use"
                                     " AntennaProfileControlMessage for compatibility mode 1.",
                                     id_,
                                     __func__);
            }
          break;

        default:
          LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                 DEBUG_LEVEL,
                                 "PHYI %03hu FrameworkPHY::%s, unexpected control message, ignore",
                                 id_,
                                 __func__);
        }
    }
}

void EMANE::FrameworkPHY::processDownstreamPacket(DownstreamPacket & pkt,
                                                  const ControlMessages & msgs)
{
  processDownstreamPacket_i(Clock::now(),pkt,msgs);
}


void EMANE::FrameworkPHY::processDownstreamPacket_i(const TimePoint & now,
                                                    DownstreamPacket & pkt,
                                                    const ControlMessages & msgs)
{
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

  // use the default unless provided via control message
  Transmitters transmitters{{id_, dTxPowerdBm_}};

  // use the default unless provided below
  FrequencyGroups frequencyGroups{{{u64TxFrequencyHz_,Microseconds::zero()}}};

  //  use the default unless provided via control message
  Antennas transmitAntennas{};

  Controls::OTATransmitters otaTransmitters;

  TimePoint txTimeStamp{now};

  double dTxWhileRxInterferenceRxPowerMilliWatt{};

  std::pair<FilterData,bool> optionalSpectrumFilterData{};

  const Controls::MIMOTxWhileRxInterferenceControlMessage * pMIMOTxWhileRxInterferenceControlMessage{};

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
          if(compatibilityMode_ == CompatibilityMode::MODE_1)
            {
              const auto pFrequencyControlMessage =
                static_cast<const Controls::FrequencyControlMessage *>(pMessage);

              // the radio model will be supplying frequency segment
              // info so lets clear the defaults
              frequencyGroups.clear();

              FrequencySegments frequencySegments{};

              for(const auto & segment : pFrequencyControlMessage->getFrequencySegments())
                {
                  // for convenience a radio model can set duration
                  // and offset without specifying frequency by using
                  // 0 Hz.
                  std::uint64_t u64TxFrequencyHz{u64TxFrequencyHz_};

                  if(segment.getFrequencyHz())
                    {
                      u64TxFrequencyHz = segment.getFrequencyHz();
                    }

                  if(segment.getPowerdBm().second)
                    {
                      frequencySegments.emplace_back(u64TxFrequencyHz, // use our frequency
                                                     segment.getPowerdBm().first,
                                                     segment.getDuration(), // duration
                                                     segment.getOffset()); // offset
                    }
                  else
                    {
                      frequencySegments.emplace_back(u64TxFrequencyHz, // use our frequency
                                                     segment.getDuration(), // duration
                                                     segment.getOffset()); // offset
                    }
                }

              frequencyGroups.push_back(std::move(frequencySegments));

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
          else
            {
              LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                     ERROR_LEVEL,
                                     "PHYI %03hu FrameworkPHY::%s, ignoring compatibility"
                                     " mode 1 FrequencyControlMessage. Use"
                                     " MIMOTransmitPropertiesControlMessage for compatibility mode 2.",
                                     id_,
                                     __func__);

            }
          break;

        case Controls::AntennaProfileControlMessage::IDENTIFIER:
          if(compatibilityMode_ == CompatibilityMode::MODE_1)
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

              /** [physicallayer-attachevent-snippet] */
              Events::AntennaProfiles profiles{{id_,
                  pAntennaProfileControlMessage->getAntennaProfileId(),
                  pAntennaProfileControlMessage->getAntennaAzimuthDegrees(),
                  pAntennaProfileControlMessage->getAntennaElevationDegrees()}};

              antennaManager_.update(profiles);

              pkt.attachEvent(0,Events::AntennaProfileEvent{profiles});
              /** [physicallayer-attachevent-snippet] */
            }
          else
            {
              LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                     ERROR_LEVEL,
                                     "PHYI %03hu FrameworkPHY::%s, ignoring compatibility"
                                     " mode 1 AntennaProfileControlMessage. Use"
                                     " RxAntennaUpdateControlMessage for compatibility mode 2.",
                                     id_,
                                     __func__);
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

        case Controls::TxWhileRxInterferenceControlMessage::IDENTIFIER:
          if(compatibilityMode_ == CompatibilityMode::MODE_1)
            {
              const auto pTxWhileRxInterferenceControlMessage =
                static_cast<const Controls::TxWhileRxInterferenceControlMessage *>(pMessage);

              dTxWhileRxInterferenceRxPowerMilliWatt =
                Utils::DB_TO_MILLIWATT(pTxWhileRxInterferenceControlMessage->getRxPowerdBm());

              LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                     DEBUG_LEVEL,
                                     "PHYI %03hu FrameworkPHY::%s Tx While Rx Interference Control Message "
                                     "rx power %.6f dBm",
                                     id_,
                                     __func__,
                                     pTxWhileRxInterferenceControlMessage->getRxPowerdBm());
            }
          else
            {
              LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                     ERROR_LEVEL,
                                     "PHYI %03hu FrameworkPHY::%s, ignoring compatibility"
                                     " mode 1 TxWhileRxInterferenceControlMessage. Use"
                                     " MIMOTxWhileRxInterferenceControlMessage for"
                                     " compatibility mode 2.",
                                     id_,
                                     __func__);
            }

          break;

        case Controls::MIMOTransmitPropertiesControlMessage::IDENTIFIER:
          if(compatibilityMode_ == CompatibilityMode::MODE_2)
            {
              const auto pMIMOTransmitPropertiesControlMessage =
                static_cast<const Controls::MIMOTransmitPropertiesControlMessage *>(pMessage);

              LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                     DEBUG_LEVEL,
                                     "PHYI %03hu FrameworkPHY::%s MIMO Transmit Properties Control Message",
                                     id_,
                                     __func__);

              frequencyGroups.clear();

              for(const auto & group : pMIMOTransmitPropertiesControlMessage->getFrequencyGroups())
                {
                  FrequencySegments frequencySegments{};

                  for(const auto & segment : group)
                    {
                      // for convenience a radio model can set duration
                      // and offset without specifying frequency by using
                      // 0 Hz.
                      std::uint64_t u64TxFrequencyHz{u64TxFrequencyHz_};

                      if(segment.getFrequencyHz())
                        {
                          u64TxFrequencyHz = segment.getFrequencyHz();
                        }

                      if(segment.getPowerdBm().second)
                        {
                          frequencySegments.emplace_back(u64TxFrequencyHz, // use our frequency
                                                         segment.getPowerdBm().first,
                                                         segment.getDuration(), // duration
                                                         segment.getOffset()); // offset
                        }
                      else
                        {
                          frequencySegments.emplace_back(u64TxFrequencyHz, // use our frequency
                                                         segment.getDuration(), // duration
                                                         segment.getOffset()); // offset
                        }
                    }

                  frequencyGroups.push_back(std::move(frequencySegments));
                }

              transmitAntennas = pMIMOTransmitPropertiesControlMessage->getTransmitAntennas();
            }
          else
            {
              LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                     ERROR_LEVEL,
                                     "PHYI %03hu FrameworkPHY::%s, ignoring compatibility"
                                     " mode 2 MIMOTransmitPropertiesControlMessage."
                                     " Use FrequencyControlMessage and/or AntennaProfileControlMessage"
                                     " for compatibility mode 1",
                                     id_,
                                     __func__);
            }

          break;

        case Controls::RxAntennaUpdateControlMessage::IDENTIFIER:
          if(compatibilityMode_ == CompatibilityMode::MODE_2)
            {
              const auto pRxAntennaUpdateControlMessage =
                reinterpret_cast<const Controls::RxAntennaUpdateControlMessage *>(pMessage);

              LOGGER_VERBOSE_LOGGING_FN_VARGS(pPlatformService_->logService(),
                                              DEBUG_LEVEL,
                                              Controls::RxAntennaUpdateControlMessageFormatter(pRxAntennaUpdateControlMessage),
                                              "PHYI %03hu FrameworkPHY::%s Rx Antenna Update Control Message",
                                              id_,
                                              __func__);

              auto rxAntenna = pRxAntennaUpdateControlMessage->getAntenna();

              // for convenience, if a radio model adds a default
              // antenna, the appropriate antenna will be created
              // based on whether fixed gain (ideal omni)
              // configuration is present.
              if(rxAntenna.isDefault())
                {
                  rxAntenna = optionalFixedAntennaGaindBi_.second ?
                    Antenna::createIdealOmni(DEFAULT_ANTENNA_INDEX,
                                             optionalFixedAntennaGaindBi_.first) :
                    Antenna::createProfileDefined(DEFAULT_ANTENNA_INDEX);
                }

              antennaManager_.update(id_,rxAntenna);
            }
          else
            {
              LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                     ERROR_LEVEL,
                                     "PHYI %03hu FrameworkPHY::%s, ignoring compatibility"
                                     " mode 2 RxAntennaUpdateControlMessage. Use"
                                     " AntennaProfileControlMessagefor compatibility mode 1.",
                                     id_,
                                     __func__);
            }
          break;

        case Controls::SpectrumFilterDataControlMessage::IDENTIFIER:
          {
            const auto pSpectrumFilterDataControlMessage =
              static_cast<const Controls::SpectrumFilterDataControlMessage *>(pMessage);

            optionalSpectrumFilterData = {pSpectrumFilterDataControlMessage->getFilterData(),true};

            LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                   DEBUG_LEVEL,
                                   "PHYI %03hu FrameworkPHY::%s Spectrum Filter Data Control Message"
                                   " filter data length: %zu",
                                   id_,
                                   __func__,
                                   optionalSpectrumFilterData.first.size());
          }

          break;

        case Controls::MIMOTxWhileRxInterferenceControlMessage::IDENTIFIER:
          if(compatibilityMode_ == CompatibilityMode::MODE_2)
            {
              pMIMOTxWhileRxInterferenceControlMessage =
                static_cast<const Controls::MIMOTxWhileRxInterferenceControlMessage *>(pMessage);

              LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                     DEBUG_LEVEL,
                                     "PHYI %03hu FrameworkPHY::%s MIMO Tx While Rx Interference Control Message",
                                     id_,
                                     __func__);
            }
          else
            {
              LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                     ERROR_LEVEL,
                                     "PHYI %03hu FrameworkPHY::%s, ignoring compatibility"
                                     " mode 2 MIMOTxWhileRxInterferenceControlMessage."
                                     " Use TxWhileRxInterferenceControlMessage for"
                                     " compatibility mode 1.",
                                     id_,
                                     __func__);
            }

          break;

        default:
          LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                 DEBUG_LEVEL,
                                 "PHYI %03hu FrameworkPHY::%s, unexpected control message %hu, ignore",
                                 id_,
                                 __func__,
                                 pMessage->getId());
        }
    }

  // for compat mode 1 and for compat mode 2 convenience, specifying no
  // transmit antennas results in a transmission using the default
  // antenna which is created based on whether fixed gain (ideal omni)
  // configuration is present
  if(transmitAntennas.empty())
    {
      auto txAntenna = optionalFixedAntennaGaindBi_.second ?
        Antenna::createIdealOmni(DEFAULT_ANTENNA_INDEX,
                                 optionalFixedAntennaGaindBi_.first) :
        Antenna::createProfileDefined(DEFAULT_ANTENNA_INDEX);

      txAntenna.setFrequencyGroupIndex(0);

      txAntenna.setBandwidthHz(u64BandwidthHz_);

      if(spectralMaskIndex_)
        {
          txAntenna.setSpectralMaskIndex(spectralMaskIndex_);
        }

      transmitAntennas.push_back(std::move(txAntenna));
    }
  else
    {
      // for convenience, if you specify a non-zero spectral mask in the
      // phy config vi spectralmaskindex, that value will be used for
      // all transmit antenna without a non-zero spectral mask.
      if(spectralMaskIndex_)
        {
          for(auto & transmitAntenna : transmitAntennas)
            {
              if(!transmitAntenna.getSpectralMaskIndex())
                {
                  transmitAntenna.setSpectralMaskIndex(spectralMaskIndex_);
                }
            }
        }
    }

  // verify the transmitters list include this nem
  if(std::find_if(transmitters.begin(),
                  transmitters.end(),
                  [this](const Transmitter & transmitter)
                  {
                    return transmitter.getNEMId() == this->id_;
                  }) == transmitters.end())
    {
      transmitters.emplace_back(id_,dTxPowerdBm_);
    }

  CommonPHYHeader phyHeader{REGISTERED_EMANE_PHY_FRAMEWORK, // phy registration id
    u16SubId_,
    u16TxSequenceNumber_++,
    txTimeStamp,
    frequencyGroups,
    transmitAntennas,
    transmitters,
    optionalSpectrumFilterData};

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

  if(compatibilityMode_ == CompatibilityMode::MODE_1 &&
     dTxWhileRxInterferenceRxPowerMilliWatt)
    {
      try
        {
          receiveProcessors_[DEFAULT_ANTENNA_INDEX]->processSelfInterference(now,
                                                                             txTimeStamp,
                                                                             frequencyGroups,
                                                                             u64BandwidthHz,
                                                                             {Controls::AntennaSelfInterference{0,dTxWhileRxInterferenceRxPowerMilliWatt}},
                                                                             optionalSpectrumFilterData);

        }
      catch(SpectrumServiceException & exp)
        {
          LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                 ERROR_LEVEL,
                                 "PHYI %03hu FrameworkPHY::%s, spectrum service error: %s",
                                 id_,
                                 __func__,
                                 exp.what());
        }
    }
  else if(compatibilityMode_ == CompatibilityMode::MODE_2 &&
          pMIMOTxWhileRxInterferenceControlMessage)
    {
      // See "Note: the processing pool..." below.
      std::vector<std::future<ReceiveProcessor::ProcessSelfInterferenceResult>> futures{};

      std::list<ReceiveProcessor::ProcessSelfInterferenceResult> results{};

      for(const auto & entry :
            pMIMOTxWhileRxInterferenceControlMessage->getRxAntennaInterferenceMap())
        {
          auto iter = receiveProcessors_.find(entry.first);

          if(iter != receiveProcessors_.end())
            {
              if(processingPool_.isRunning())
                {
                  futures.push_back(processingPool_.submit(std::bind(&ReceiveProcessor::processSelfInterference,
                                                                     iter->second.get(),
                                                                     std::cref(now),
                                                                     std::cref(txTimeStamp),
                                                                     std::cref(frequencyGroups),
                                                                     u64BandwidthHz,
                                                                     std::cref(entry.second),
                                                                     std::cref(optionalSpectrumFilterData))));
                }
              else
                {
                  results.push_back(iter->second->processSelfInterference(now,
                                                                          txTimeStamp,
                                                                          frequencyGroups,
                                                                          u64BandwidthHz,
                                                                          entry.second,
                                                                          optionalSpectrumFilterData));
                }
            }
        }

      // wait for all results
      if(processingPool_.isRunning())
        {
          for(auto & future : futures)
            {
              results.push_back(std::move(future.get()));
            }
        }

      for(const auto & result : results)
        {
          switch(result.status_)
            {
            case ReceiveProcessor::ProcessSelfInterferenceResult::Status::ERROR_MISSING_POWER_VALUES:
              LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                     ERROR_LEVEL,
                                     "PHYI %03hu FrameworkPHY::%s, MIMO TXWhileRxInterference"
                                     " specified power values do not match the number of frequency"
                                     " segments in use",
                                     id_,
                                     __func__);
              break;

            case ReceiveProcessor::ProcessSelfInterferenceResult::Status::ERROR_ANTENNA_FREQ_INDEX:
              LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                     ERROR_LEVEL,
                                     "PHYI %03hu FrameworkPHY::%s, MIMO TXWhileRxInterference"
                                     " specified antenna frequency index outside frequency group range",
                                     id_,
                                     __func__);
              break;

            default:
              break;
            }
        }
    }
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

  if(compatibilityMode_ == CompatibilityMode::MODE_1)
    {
      createDefaultAntennaIfNeeded();
    }
  else
    {
      if(receiveProcessors_.empty())
        {
          commonLayerStatistics_.processOutbound(pkt,
                                                 std::chrono::duration_cast<Microseconds>(Clock::now() - now),
                                                 DROP_CODE_MISSING_CONTROL);

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  ERROR_LEVEL,
                                  "PHYI %03hu FrameworkPHY::%s "
                                  " src %hu, dst %hu, drop no rx antennas in use",
                                  id_,
                                  __func__,
                                  pktInfo.getSource(),
                                  pktInfo.getDestination());

          //drop
          return;
        }
    }

  // unless this is an in-band packet, it will only be processed if
  // noise processing is on
  if(bInBand || noiseMode_ != NoiseMode::NONE)
    {
      Controls::AntennaReceiveInfos antennaReceiveInfos{};
      Controls::DopplerShifts dopplerShifts{};
      std::set<TimePoint> mimoSoT{};
      std::set<Microseconds> mimoPropagationDelay{};
      std::vector<std::pair<LocationInfo,bool>> locationInfos{};
      std::vector<std::pair<FadingInfo,bool>> fadingSelections{};

      for(const auto & transmitter : commonPHYHeader.getTransmitters())
        {
          locationInfos.push_back(locationManager_.getLocationInfo(transmitter.getNEMId()));

          fadingSelections.push_back(fadingManager_.getFadingSelection(transmitter.getNEMId()));

          for(const auto & txAntenna : commonPHYHeader.getTransmitAntennas())
            {
              antennaManager_.update(transmitter.getNEMId(),txAntenna);
            }
        }

      // Note: the processing pool, if enabled, is used to process
      // receive paths per receive antenna. The emane thread/messaging
      // model uses a functor queue to guarantee the processing of
      // framework messages (event, upstream packet and/or control,
      // downstream packet and/or control, running-state
      // configuration, and timers. No new message processing occurs
      // until the current message processing is complete. This allows
      // the processing pool to access internal state without
      // additional locks. Care must me taken when extending
      // functionality so no inter-thread communication occurs between
      // pool threads, otherwise additional locks will be necessary.
      //
      // The processing pool can reduce the amount of processing time
      // for an upstream message that contains a large number of
      // frequency segments and/or a large number of transmit antenna
      // (MIMO). Without a processing pool receive paths are
      // calculated serially in a loop. There is a threshold where
      // serial processing is faster than the context switching of the
      // thread pool. Server and scenario benchmarking should be used
      // to determine whether processing pool usage is appropriate.
      // Additionally, if the number of cores available to a running
      // emane process is less than the processing pool size worse
      // performance may be encountered.

      std::vector<std::future<ReceiveProcessor::ProcessResult>> futures{};

      std::list<ReceiveProcessor::ProcessResult> results{};

      for(auto & receiveProcessorEntry : receiveProcessors_)
        {
          auto & pReceiveProcessor = receiveProcessorEntry.second;

          if(processingPool_.isRunning())
            {
              futures.push_back(processingPool_.submit(std::bind(&ReceiveProcessor::process,pReceiveProcessor.get(),
                                                                 std::cref(now),
                                                                 std::cref(commonPHYHeader),
                                                                 std::cref(locationInfos),
                                                                 std::cref(fadingSelections),
                                                                 bInBand)));
            }
          else
            {
              results.push_back(pReceiveProcessor->process(now,
                                                           commonPHYHeader,
                                                           locationInfos,
                                                           fadingSelections,
                                                           bInBand));
            }
        }

      // wait for all results
      if(processingPool_.isRunning())
        {
          for(auto & future : futures)
            {
              results.push_back(std::move(future.get()));
            }
        }

      bool bNotFOI{};

      for(auto & result : results)
        {
          if(result.status_ == ReceiveProcessor::ProcessResult::Status::SUCCESS)
            {
              if(result.mimoSoT_.time_since_epoch().count() != 0)
                {
                  mimoSoT.insert(result.mimoSoT_);
                  mimoPropagationDelay.insert(result.mimoPropagationDelay_);
                }

              antennaReceiveInfos.insert(antennaReceiveInfos.end(),
                                         std::make_move_iterator(result.antennaReceiveInfos_.begin()),
                                         std::make_move_iterator(result.antennaReceiveInfos_.end()));

              dopplerShifts.insert(std::make_move_iterator(result.dopplerShifts_.begin()),
                                   std::make_move_iterator(result.dopplerShifts_.end()));

              if(result.bGainCacheHit_)
                {
                  ++*pGainCacheHit_;
                }
              else
                {
                  ++*pGainCacheMiss_;
                }

              for(const auto & entry : result.receivePowerMap_)
                {
                  receivePowerTablePublisher_.update(std::get<0>(entry.first),
                                                     std::get<1>(entry.first),
                                                     std::get<2>(entry.first),
                                                     std::get<3>(entry.first),
                                                     std::get<0>(entry.second),
                                                     std::get<1>(entry.second),
                                                     std::get<2>(entry.second),
                                                     std::get<3>(entry.second),
                                                     std::get<4>(entry.second),
                                                     std::get<5>(entry.second),
                                                     commonPHYHeader.getTxTime());
                }

              for(const auto & entry : result.observedPowerMap_)
                {
                  observedPowerTablePublisher_.update(std::get<0>(entry.first),
                                                      std::get<1>(entry.first),
                                                      std::get<2>(entry.first),
                                                      std::get<3>(entry.first),
                                                      std::get<0>(entry.second),
                                                      std::get<1>(entry.second),
                                                      commonPHYHeader.getTxTime());
                }
            }
          else
            {
              std::string sReason{"unknown"};
              LogLevel logLevel{DEBUG_LEVEL};

              Microseconds processingDuration{std::chrono::duration_cast<Microseconds>(Clock::now() - now)};

              switch(result.status_)
                {
                case ReceiveProcessor::ProcessResult::Status::DROP_CODE_ANTENNA_FREQ_INDEX:
                  commonLayerStatistics_.processOutbound(pkt,
                                                         processingDuration,
                                                         DROP_CODE_ANTENNA_FREQ_INDEX);
                  sReason = "transmit antenna frequency index invalid";
                  logLevel = ERROR_LEVEL;
                  break;

                case ReceiveProcessor::ProcessResult::Status::DROP_CODE_FADINGMANAGER_LOCATION:
                  commonLayerStatistics_.processOutbound(pkt,
                                                         processingDuration,
                                                         DROP_CODE_FADINGMANAGER_LOCATION);
                  sReason = "FadingManager missing location information";
                  break;

                case ReceiveProcessor::ProcessResult::Status::DROP_CODE_FADINGMANAGER_ALGORITHM:
                  commonLayerStatistics_.processOutbound(pkt,
                                                         processingDuration,
                                                         DROP_CODE_FADINGMANAGER_ALGORITHM);

                  sReason = "FadingManager unknown algorithm";
                  logLevel = ERROR_LEVEL;
                  break;

                case ReceiveProcessor::ProcessResult::Status::DROP_CODE_FADINGMANAGER_SELECTION:
                  commonLayerStatistics_.processOutbound(pkt,
                                                         processingDuration,
                                                         DROP_CODE_FADINGMANAGER_SELECTION);

                  sReason = "FadingManager unknown fading selection for transmitter";

                  break;

                case ReceiveProcessor::ProcessResult::Status::DROP_CODE_GAINMANAGER_LOCATION:
                  commonLayerStatistics_.processOutbound(pkt,
                                                         processingDuration,
                                                         DROP_CODE_GAINMANAGER_LOCATION);

                  sReason = "GainManager missing location information";
                  break;

                case ReceiveProcessor::ProcessResult::Status::DROP_CODE_GAINMANAGER_ANTENNAPROFILE:
                  commonLayerStatistics_.processOutbound(pkt,
                                                         processingDuration,
                                                         DROP_CODE_GAINMANAGER_ANTENNAPROFILE);
                  sReason = "GainManager unknown antenna profile";
                  logLevel = ERROR_LEVEL;
                  break;

                case ReceiveProcessor::ProcessResult::Status::DROP_CODE_GAINMANAGER_HORIZON:
                  commonLayerStatistics_.processOutbound(pkt,
                                                         processingDuration,
                                                         DROP_CODE_GAINMANAGER_HORIZON);
                  sReason = "GainManager below horizon";
                  break;

                case ReceiveProcessor::ProcessResult::Status::DROP_CODE_GAINMANAGER_ANTENNA_INDEX:
                  commonLayerStatistics_.processOutbound(pkt,
                                                         processingDuration,
                                                         DROP_CODE_GAINMANAGER_ANTENNA_INDEX);
                  sReason = "GainManager unknown antenna index";
                  logLevel = ERROR_LEVEL;
                  break;

                case ReceiveProcessor::ProcessResult::Status::DROP_CODE_PROPAGATIONMODEL:
                  commonLayerStatistics_.processOutbound(pkt,
                                                         processingDuration,
                                                         DROP_CODE_PROPAGATIONMODEL);
                  sReason = "propagation model missing information";
                  break;

                case ReceiveProcessor::ProcessResult::Status::DROP_CODE_SPECTRUM_CLAMP:
                  commonLayerStatistics_.processOutbound(pkt,
                                                         processingDuration,
                                                         DROP_CODE_SPECTRUM_CLAMP);
                  sReason = "SpectrumManager detected request range error";
                  break;

                case ReceiveProcessor::ProcessResult::Status::DROP_CODE_NOT_FOI:
                  bNotFOI = true;
                  break;

                case ReceiveProcessor::ProcessResult::Status::DROP_CODE_OUT_OF_BAND:
                  commonLayerStatistics_.processOutbound(pkt,
                                                         processingDuration,
                                                         DROP_CODE_OUT_OF_BAND);
                  sReason = "out-of-band message, not for this waveform";
                  break;

                default:
                  break;
                }

              if(!bNotFOI)
                {
                  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                          logLevel,
                                          "PHYI %03hu FrameworkPHY::%s "
                                          " src %hu, dst %hu, drop %s",
                                          id_,
                                          __func__,
                                          pktInfo.getSource(),
                                          pktInfo.getDestination(),
                                          sReason.c_str());

                  // drop
                  return;
                }
            }
        }

      if(!antennaReceiveInfos.empty())
        {
          if(compatibilityMode_ == CompatibilityMode::MODE_1)
            {
              const auto & defaultAntennaReceiveInfo = antennaReceiveInfos[0];
              const auto & firstTxAntenna = commonPHYHeader.getTransmitAntennas()[0];

              /** [physicallayer-sendupstreampacket-snippet] */
              sendUpstreamPacket(pkt,
                                 {Controls::FrequencyControlMessage::create(firstTxAntenna.getBandwidthHz(),
                                                                            defaultAntennaReceiveInfo.getFrequencySegments()),
                                  Controls::ReceivePropertiesControlMessage::create(*mimoSoT.begin(),
                                                                                    *mimoPropagationDelay.begin(),
                                                                                    defaultAntennaReceiveInfo.getSpan(),
                                                                                    defaultAntennaReceiveInfo.getReceiverSensitivitydBm())});
              /** [physicallayer-sendupstreampacket-snippet] */
            }
          else
            {
              sendUpstreamPacket(pkt,
                                 {Controls::MIMOReceivePropertiesControlMessage::create(*mimoSoT.begin(),
                                                                                        *mimoPropagationDelay.begin(),
                                                                                        std::move(antennaReceiveInfos),
                                                                                        std::move(dopplerShifts))});
            }

          if(*mimoSoT.begin() != commonPHYHeader.getTxTime())
            {
              ++*pTimeSyncThresholdRewrite_;
            }
        }
      else
        {
          if(bNotFOI)
            {
              commonLayerStatistics_.processOutbound(pkt,
                                                     std::chrono::duration_cast<Microseconds>(Clock::now() - now),
                                                     DROP_CODE_NOT_FOI);

              LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                      DEBUG_LEVEL,
                                      "PHYI %03hu FrameworkPHY::%s"
                                      " src %hu, dst %hu, drop message"
                                      " frequency not in frequency of interest list.",
                                      id_,
                                      __func__,
                                      pktInfo.getSource(),
                                      pktInfo.getDestination());
            }
          else
            {
              if(bRxSensitivityPromiscuousModeEnable_ && compatibilityMode_ == CompatibilityMode::MODE_2)
                {
                  sendUpstreamPacket(pkt);
                }

              if(*mimoSoT.begin() != commonPHYHeader.getTxTime())
                {
                  ++*pTimeSyncThresholdRewrite_;
                }

              // below rx sensitivy on all antenna
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

          // drop
          return;
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

      // drop
      return;
    }
}


/** [eventservice-processevent-snippet] */
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
        antennaManager_.update(antennaProfile.getAntennaProfiles());
        eventTablePublisher_.update(antennaProfile.getAntennaProfiles());

        LOGGER_STANDARD_LOGGING_FN_VARGS(pPlatformService_->logService(),
                                         DEBUG_LEVEL,
                                         Events::AntennaProfileEventFormatter(antennaProfile),
                                         "PHYI %03hu FrameworkPHY::%s antenna profile event: ",
                                         id_,
                                         __func__);

      }
      break;

    case Events::FadingSelectionEvent::IDENTIFIER:
      {
        Events::FadingSelectionEvent fadingSelection{serialization};
        fadingManager_.update(fadingSelection.getFadingSelections());
        eventTablePublisher_.update(fadingSelection.getFadingSelections());

        LOGGER_STANDARD_LOGGING_FN_VARGS(pPlatformService_->logService(),
                                         DEBUG_LEVEL,
                                         Events::FadingSelectionEventFormatter(fadingSelection),
                                         "PHYI %03hu FrameworkPHY::%s fading selection event: ",
                                         id_,
                                         __func__);
      }
      break;

    case Events::LocationEvent::IDENTIFIER:
      {
        Events::LocationEvent locationEvent{serialization};
        locationManager_.update(locationEvent.getLocations());
        eventTablePublisher_.update(locationEvent.getLocations());

        LOGGER_STANDARD_LOGGING_FN_VARGS(pPlatformService_->logService(),
                                         DEBUG_LEVEL,
                                         Events::LocationEventFormatter(locationEvent),
                                         "PHYI %03hu FrameworkPHY::%s location event: ",
                                         id_,
                                         __func__);
      }
      break;

    case Events::PathlossEvent::IDENTIFIER:
      {
        Events::PathlossEvent pathlossEvent{serialization};
        pPropagationModelAlgorithm_->update(pathlossEvent.getPathlosses());
        eventTablePublisher_.update(pathlossEvent.getPathlosses());

        LOGGER_STANDARD_LOGGING_FN_VARGS(pPlatformService_->logService(),
                                         DEBUG_LEVEL,
                                         Events::PathlossEventFormatter(pathlossEvent),
                                         "PHYI %03hu FrameworkPHY::%s pathloss event: ",
                                         id_,
                                         __func__);
      }
      break;
    }
}
/** [eventservice-processevent-snippet] */

void EMANE::FrameworkPHY::createDefaultAntennaIfNeeded()
{
  if(receiveProcessors_.empty())
    {
      antennaManager_.update(id_,
                             optionalFixedAntennaGaindBi_.second ?
                             Antenna::createIdealOmni(DEFAULT_ANTENNA_INDEX,
                                                      optionalFixedAntennaGaindBi_.first) :
                             Antenna::createProfileDefined(DEFAULT_ANTENNA_INDEX));

      auto pSpectrumMonitor =
        pSpectrumService_->addSpectrumMonitor(DEFAULT_ANTENNA_INDEX,
                                              foi_,
                                              u64BandwidthHz_,
                                              Utils::DB_TO_MILLIWATT(dReceiverSensitivitydBm_));

      receiveProcessors_.emplace(DEFAULT_ANTENNA_INDEX,
                                 std::unique_ptr<ReceiveProcessor>(new ReceiveProcessor{id_,
                                                                                        u16SubId_,
                                                                                        DEFAULT_ANTENNA_INDEX,
                                                                                        antennaManager_,
                                                                                        pSpectrumMonitor,
                                                                                        pPropagationModelAlgorithm_.get(),
                                                                                        fadingManager_.createFadingAlgorithmStore(),
                                                                                        bStatsReceivePowerTableEnable_,
                                                                                        bStatsObservedPowerTableEnable_,
                                                                                        bDopplerShiftEnable_}));
    }
}
