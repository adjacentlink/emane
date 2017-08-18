/*
 * Copyright (c) 2013-2017 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2011 - DRS CenGen, LLC, Columbia, Maryland
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

#include "nemmanagerimpl.h"
#include "logservice.h"
#include "otamanager.h"
#include "timerservice.h"
#include "eventservice.h"
#include "eventserviceexception.h"
#include "emane/configureexception.h"
#include "emane/platformexception.h"
#include "emane/startexception.h"
#include "otaexception.h"
#include "antennaprofilemanifest.h"

EMANE::Application::NEMManagerImpl::NEMManagerImpl(const uuid_t & uuid):
  NEMManager{uuid}{}

EMANE::Application::NEMManagerImpl::~NEMManagerImpl(){}

void EMANE::Application::NEMManagerImpl::add(std::unique_ptr<Application::NEM> & pNEM)
{
  if(!platformNEMMap_.insert(std::make_pair(pNEM->getNEMId(),std::move(pNEM))).second)
    {
      throw makeException<PlatformException>("NEMManagerImpl:  Multiple NEMs with id"
                                             " %hu detected",
                                             pNEM->getNEMId());
    }
}

void EMANE::Application::NEMManagerImpl::initialize(Registrar & registrar)
{
  auto & configRegistrar = registrar.configurationRegistrar();

  configRegistrar.registerNonNumeric<INETAddr>("eventservicegroup",
                                               ConfigurationProperties::REQUIRED,
                                               {},
                                               "IPv4 or IPv6 Event Service channel multicast endpoint.");

  configRegistrar.registerNonNumeric<std::string>("eventservicedevice",
                                                  ConfigurationProperties::NONE,
                                                  {},
                                                  "Device to associate with the Event Service channel multicast endpoint.");

  configRegistrar.registerNumeric<std::uint8_t>("eventservicettl",
                                                ConfigurationProperties::DEFAULT,
                                                {1},
                                                "Device to associate with the Event Service channel multicast endpoint.");

  configRegistrar.registerNonNumeric<INETAddr>("otamanagergroup",
                                               ConfigurationProperties::NONE,
                                               {},
                                               "IPv4 or IPv6 Event Service OTA channel endpoint.");

  configRegistrar.registerNonNumeric<std::string>("otamanagerdevice",
                                                  ConfigurationProperties::NONE,
                                                  {},
                                                  "Device to associate with the OTA channel multicast endpoint.");

  configRegistrar.registerNumeric<std::uint32_t>("otamanagermtu",
                                                 ConfigurationProperties::DEFAULT,
    {0},
                                                 "OTA channel MTU.");

  configRegistrar.registerNumeric<std::uint16_t>("otamanagerpartcheckthreshold",
                                                 ConfigurationProperties::DEFAULT,
                                                 {2},
                                                 "Defines the rate in seconds a check is performed to see if any OTA packet"
                                                 " part reassembly efforts should be abandoned.");

  configRegistrar.registerNumeric<std::uint16_t>("otamanagerparttimeoutthreshold",
                                                 ConfigurationProperties::DEFAULT,
    {5},
                                                 "Defines the threshold in seconds to wait for another OTA packet part"
                                                 " for an existing reassembly effort before abandoning the effort.");

  configRegistrar.registerNumeric<std::uint8_t>("otamanagerttl",
                                                ConfigurationProperties::DEFAULT,
                                                {1},
                                                "OTA channel multicast message TTL.");

  configRegistrar.registerNumeric<bool>("otamanagerloopback",
                                        ConfigurationProperties::DEFAULT,
                                        {false},
                                        "Enable multicast loopback on the OTA channel multicast channel.");

  configRegistrar.registerNumeric<bool>("otamanagerchannelenable",
                                        ConfigurationProperties::DEFAULT,
                                        {true},
                                        "Enable OTA channel multicast communication.");


  configRegistrar.registerNonNumeric<INETAddr>("controlportendpoint",
                                               ConfigurationProperties::REQUIRED,
                                               {INETAddr{"0.0.0.0",47000 }},
                                               "IPv4 or IPv6 control port endpoint.");

  configRegistrar.registerNonNumeric<std::string>("antennaprofilemanifesturi",
                                                  EMANE::ConfigurationProperties::NONE,
                                                  {},
                                                  "URI of the antenna profile manifest to load."
                                                  " The antenna profile manifest contains a list of"
                                                  " antenna profile entries. Each entry contains a unique"
                                                  " profile identifier, an antenna pattern URI and an"
                                                  " antenna blockage URI. This parameter is required when"
                                                  " antennaprofileenable is on or if any other NEM"
                                                  " participating in the emulation has antennaprofileenable"
                                                  " set on, even in the case where antennaprofileenable is"
                                                  " off locally.");

  configRegistrar.registerNumeric<std::uint32_t>("stats.ota.maxpacketcountrows",
                                                 ConfigurationProperties::DEFAULT,
                                                 {0},
                                                 "OTA channel max packet count table rows.");

  configRegistrar.registerNumeric<std::uint32_t>("stats.ota.maxeventcountrows",
                                                 ConfigurationProperties::DEFAULT,
                                                 {0},
                                                 "OTA channel max event count table rows.");

  configRegistrar.registerNumeric<std::uint32_t>("stats.event.maxeventcountrows",
                                                 ConfigurationProperties::DEFAULT,
                                                 {0},
                                                 "Event channel max event count table rows.");

}

void EMANE::Application::NEMManagerImpl::configure(const ConfigurationUpdate & update)
{
  for(const auto & item : update)
    {
      if(item.first == "otamanagergroup")
        {
          OTAManagerGroupAddr_ = item.second[0].asINETAddr();

          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  INFO_LEVEL,
                                  "NEMManagerImpl::configure %s: %s",
                                  item.first.c_str(),
                                  OTAManagerGroupAddr_.str().c_str());

        }
      else if(item.first == "otamanagerdevice")
        {
          sOTAManagerGroupDevice_ = item.second[0].asString();

          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  INFO_LEVEL,
                                  "NEMManagerImpl::configure %s: %s",
                                  item.first.c_str(),
                                  sOTAManagerGroupDevice_.c_str());

        }
      else if(item.first == "otamanagerttl")
        {
          u8OTAManagerTTL_ = item.second[0].asUINT8();

          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  INFO_LEVEL,
                                  "NEMManagerImpl::configure %s: %hhu",
                                  item.first.c_str(),
                                  u8OTAManagerTTL_);
        }
      else if(item.first == "otamanagermtu")
        {
          u32OTAManagerMTU_ = item.second[0].asUINT32();

          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  INFO_LEVEL,
                                  "NEMManagerImpl::configure %s: %u",
                                  item.first.c_str(),
                                  u32OTAManagerMTU_);
        }
      else if(item.first == "otamanagerpartcheckthreshold")
        {
          OTAManagerPartCheckThreshold_= EMANE::Seconds{item.second[0].asUINT16()};

          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  INFO_LEVEL,
                                  "NEMManagerImpl::configure %s = %lu",
                                  item.first.c_str(),
                                  OTAManagerPartCheckThreshold_.count());
        }
      else if(item.first == "otamanagerparttimeoutthreshold")
        {
          OTAManagerPartTimeoutThreshold_ = EMANE::Seconds{item.second[0].asUINT16()};

          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  INFO_LEVEL,
                                  "NEMManagerImpl::configure %s = %lu",
                                  item.first.c_str(),
                                  OTAManagerPartTimeoutThreshold_.count());
        }
      else if(item.first == "otamanagerloopback")
        {
          bOTAManagerChannelLoopback_ = item.second[0].asBool();

          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  INFO_LEVEL,
                                  "NEMManagerImpl::configure %s: %s",
                                  item.first.c_str(),
                                  bOTAManagerChannelLoopback_ ? "on" : "off");
        }
      else if(item.first == "otamanagerchannelenable")
        {
          bOTAManagerChannelEnable_ = item.second[0].asBool();

          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  INFO_LEVEL,
                                  "NEMManagerImpl::configure %s: %s",
                                  item.first.c_str(),
                                  bOTAManagerChannelEnable_ ? "on" : "off");
        }
      else if(item.first == "eventservicegroup")
        {
          eventServiceGroupAddr_ = item.second[0].asINETAddr();

          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  INFO_LEVEL,
                                  "NEMManagerImpl::configure %s: %s",
                                  item.first.c_str(),
                                  eventServiceGroupAddr_.str().c_str());

        }
      else if(item.first == "eventservicedevice")
        {
          sEventServiceDevice_ = item.second[0].asString();

          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  INFO_LEVEL,
                                  "NEMManagerImpl::configure %s: %s",
                                  item.first.c_str(),
                                  sEventServiceDevice_.c_str());

        }
      else if(item.first == "eventservicettl")
        {
          u8EventServiceTTL_ = item.second[0].asUINT8();

          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  INFO_LEVEL,
                                  "NEMManagerImpl::configure %s: %hhu",
                                  item.first.c_str(),
                                  u8EventServiceTTL_);
        }
      else if(item.first == "controlportendpoint")
        {
          controlPortAddr_ = item.second[0].asINETAddr();

          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  INFO_LEVEL,
                                  "NEMManagerImpl::configure %s: %s",
                                  item.first.c_str(),
                                  controlPortAddr_.str().c_str());
        }

      else if(item.first == "antennaprofilemanifesturi")
        {
          sAntennaProfileManifestURI_ = item.second[0].asString();

          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  INFO_LEVEL,
                                  "NEMManagerImpl::configure %s: %s",
                                  item.first.c_str(),
                                  sAntennaProfileManifestURI_.c_str());

        }
      else if(item.first == "stats.ota.maxpacketcountrows")
        {
          std::uint32_t u32OTAMaxPacketCountRows = item.second[0].asUINT32();

          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  INFO_LEVEL,
                                  "NEMManagerImpl::configure %s: %u",
                                  item.first.c_str(),
                                  u32OTAMaxPacketCountRows);

          OTAManagerSingleton::instance()->
            setStatPacketCountRowLimit(u32OTAMaxPacketCountRows);
        }
      else if(item.first == "stats.ota.maxeventcountrows")
        {
          std::uint32_t u32OTAMaxEventCountRows = item.second[0].asUINT32();

          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  INFO_LEVEL,
                                  "NEMManagerImpl::configure %s: %u",
                                  item.first.c_str(),
                                  u32OTAMaxEventCountRows);

          OTAManagerSingleton::instance()->
            setStatEventCountRowLimit(u32OTAMaxEventCountRows);
        }
      else if(item.first == "stats.event.maxeventcountrows")
        {
          std::uint32_t u32EventMaxEventCountRows = item.second[0].asUINT32();

          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  INFO_LEVEL,
                                  "NEMManagerImpl::configure %s: %u",
                                  item.first.c_str(),
                                  u32EventMaxEventCountRows);

          EventServiceSingleton::instance()->
            setStatEventCountRowLimit(u32EventMaxEventCountRows);
        }
      else
        {
          throw makeException<ConfigureException>("NEMManagerImpl: "
                                                  "Unexpected configuration item %s",
                                                  item.first.c_str());
        }
    }


  if(!sAntennaProfileManifestURI_.empty())
    {
      AntennaProfileManifest::instance()->load(sAntennaProfileManifestURI_);
    }

}

void EMANE::Application::NEMManagerImpl::start()
{
  if(bOTAManagerChannelEnable_)
    {
      try
        {
          OTAManagerSingleton::instance()->open(OTAManagerGroupAddr_,
                                                sOTAManagerGroupDevice_,
                                                bOTAManagerChannelLoopback_,
                                                u8OTAManagerTTL_,
                                                uuid_,
                                                u32OTAManagerMTU_,
                                                OTAManagerPartCheckThreshold_,
                                                OTAManagerPartTimeoutThreshold_);
        }
      catch(OTAException & exp)
        {
          throw StartException(exp.what());
        }
    }

  try
    {
      EMANE::EventServiceSingleton::instance()->open(eventServiceGroupAddr_,
                                                     sEventServiceDevice_,
                                                     u8EventServiceTTL_,
                                                     true,
                                                     uuid_);
    }
  catch(EventServiceException & e)
    {
      throw StartException(e.what());
    }

  controlPortService_.open(controlPortAddr_);

  std::for_each(platformNEMMap_.begin(),
                platformNEMMap_.end(),
                std::bind(&Component::start,
                          std::bind(&PlatformNEMMap::value_type::second,
                                    std::placeholders::_1)));
}

void EMANE::Application::NEMManagerImpl::postStart()
{
  std::for_each(platformNEMMap_.begin(),
                platformNEMMap_.end(),
                std::bind(&Component::postStart,
                          std::bind(&PlatformNEMMap::value_type::second,
                                    std::placeholders::_1)));
}

void EMANE::Application::NEMManagerImpl::stop()
{
  controlPortService_.close();

  std::for_each(platformNEMMap_.begin(),
                platformNEMMap_.end(),
                std::bind(&Component::stop,
                          std::bind(&PlatformNEMMap::value_type::second,
                                    std::placeholders::_1)));
}

void EMANE::Application::NEMManagerImpl::destroy()
  throw()
{
  std::for_each(platformNEMMap_.begin(),
                platformNEMMap_.end(),
                std::bind(&Component::destroy,
                          std::bind(&PlatformNEMMap::value_type::second,
                                    std::placeholders::_1)));
}
