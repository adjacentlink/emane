/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey 
 * Copyright (c) 2008-2009 - DRS CenGen, LLC, Columbia, Maryland
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

#include "agent.h"
#include "emane/constants.h"
#include "emane/events/locationevent.h"
#include "emane/configureexception.h"
#include "emane/startexception.h"

#include <sstream>

#include <ace/LSOCK_Connector.h>
#include <ace/SOCK_Connector.h>
#include <ace/DEV_Connector.h>
#include <ace/FILE_Connector.h>
#include <ace/TTY_IO.h>
#include <ace/OS_NS_time.h>
#include <ace/OS_NS_sys_time.h>

#ifdef __APPLE__
#include <util.h>
#else
#include <pty.h> 
#endif

EMANE::Agents::GPSDLocation::Agent::Agent(NEMId nemId, 
                                          PlatformServiceProvider *pPlatformService):
  EventAgent{nemId, pPlatformService},
  nemId_{nemId},
  masterPTY_{},
  slavePTY_{},
  bHaveInitialPosition_{false},
  bHaveInitialVelocity_{false},
  dLatitudeDegrees_{},
  dLongitudeDegrees_{},
  dAltitudeMeters_{},
  dAzimuthDegrees_{},
  dMagnitudeMetersPerSecond_{},
  timerId_{}{}

EMANE::Agents::GPSDLocation::Agent::~Agent(){}

void EMANE::Agents::GPSDLocation::Agent::initialize(Registrar & registrar)
{
  auto & configRegistrar = registrar.configurationRegistrar();

  configRegistrar.registerNonNumeric<std::string>("gpsdcontrolsocket",
                                                  ConfigurationProperties::DEFAULT,
                                                  {"/tmp/gpsd.control"},
                                                  "GPSd Control Socket.");

  configRegistrar.registerNonNumeric<std::string>("pseudoterminalfile",
                                                  ConfigurationProperties::DEFAULT,
                                                  {"/tmp/gpsdlocation.pty"},
                                                  "File to write the name of the created pseudo term.");

   
  configRegistrar.registerNumeric<bool>("gpsdconnectionenabled",
                                        ConfigurationProperties::DEFAULT,
                                        {false},
                                        "Enable active connection to GPSd.");

  auto & eventRegistrar = registrar.eventRegistrar();
  
  eventRegistrar.registerEvent(Events::LocationEvent::IDENTIFIER);
}

void EMANE::Agents::GPSDLocation::Agent::configure(const ConfigurationUpdate & update)
{
  for(const auto & item : update)
    {
      if(item.first == "gpsdcontrolsocket")
        {
          sGPSDControlSocket_ = item.second[0].asString();
        }
      else if(item.first == "pseudoterminalfile")
        {
          sPseudoTerminalFile_ = item.second[0].asString();
        }
      else if(item.first == "gpsdconnectionenabled")
        {
          bGPSDConnectionEnabled_ = item.second[0].asBool();
        }
      else
        {
          throw makeException<ConfigureException>("GPSDLocation::Agent: "
                                                  "Unexpected configuration item %s",
                                                  item.first.c_str());
        }
    }
}


void EMANE::Agents::GPSDLocation::Agent::start()
{
  char pzName[MAXPATHLEN]{};
  ACE_LSOCK_Stream gpsdControlStream;
  ACE_LSOCK_Connector connect;
      
  if(bGPSDConnectionEnabled_)
    {
      gpsdControlAddr_.set(sGPSDControlSocket_.c_str());

      if(connect.connect(gpsdControlStream,gpsdControlAddr_) == -1)
        {
          std::stringstream ssDescription;
          ssDescription<<"unable to open GPSD control socket"<<std::ends;
          throw StartException(ssDescription.str());
        }
    }

  if(openpty(&masterPTY_, &slavePTY_, nullptr,nullptr,nullptr) == -1)
    {
      throw StartException("Unable to open pseudo terminal for gpsd"); 
    }

  if(ttyname_r(slavePTY_,&pzName[1],sizeof(pzName)-1) == -1)
    {
      throw StartException("Unable to determin pseudo terminal name"); 
    }

  ACE_TTY_IO slaveTTY;
  ACE_DEV_Connector devConnector;
  ACE_TTY_IO::Serial_Params serial_params;

  if(devConnector.connect(slaveTTY,ACE_DEV_Addr(&pzName[1])) == -1)
    {
      throw StartException("Unable to open slave pseudo terminal for gpsd"); 
    }

  slaveTTY.control(ACE_TTY_IO::GETPARAMS,&serial_params);

  serial_params.baudrate = 4800;
  serial_params.xonlim = 0;
  serial_params.xofflim = 0;
  serial_params.readmincharacters = 1;
  serial_params.readtimeoutmsec = -1;
  serial_params.paritymode = "none";
  serial_params.ctsenb = false;
  serial_params.rtsenb = 0;
  serial_params.xinenb = false;
  serial_params.xoutenb = false;
  serial_params.modem = false;
  serial_params.rcvenb = true;
  serial_params.dsrenb = false;
  serial_params.dtrdisable = false;
  serial_params.databits = 8;
  serial_params.stopbits = 1;

  slaveTTY.control(ACE_TTY_IO::SETPARAMS,&serial_params);

  slaveTTY.close();
  
  // open up the file containing the name of the pseudo ternimal
  ACE_FILE_Connector connector;
  if(connector.connect(pseudoTerminalNameFile_,
                       ACE_FILE_Addr(sPseudoTerminalFile_.c_str()),
                       0,
                       ACE_Addr::sap_any,
                       0,
                       O_WRONLY | O_CREAT | O_TRUNC) == -1)
    {
      std::stringstream ssDescription;
      ssDescription<<"unable to open pseudo terminal name file: "<<sPseudoTerminalFile_<<std::ends;
      throw StartException(ssDescription.str());
    }
 
  sClientPTTYName_ = &pzName[1];

  pzName[0] = '+';
  pzName[strlen(pzName)] = '\n';
 
  // store the pseudo terminal name
  pseudoTerminalNameFile_.send(&pzName[1],strlen(pzName)-1);
  pseudoTerminalNameFile_.close();
  
  if(bGPSDConnectionEnabled_)
    {
      gpsdControlStream.send(pzName,strlen(pzName));
      
      gpsdControlStream.close();

      ACE_INET_Addr addr("localhost:2947");
      ACE_SOCK_Connector connect2;

      if(connect2.connect(gpsdClientStream_,addr) == -1)
        {
          std::stringstream ssDescription;
          ssDescription<<"unable to open GPSD stream socket"<<std::ends;
          throw StartException(ssDescription.str());
        }

      gpsdClientStream_.send("w+r+\n", 5);
    }

  
  std::chrono::seconds interval{1};

  timerId_ = pPlatformService_->timerService().scheduleTimedEvent(Clock::now() + interval,
                                                                  nullptr,
                                                                  interval);

  LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(), DEBUG_LEVEL, "Timer Scheduled");             
}

void EMANE::Agents::GPSDLocation::Agent::stop()
{
  if(!sClientPTTYName_.empty())
    {
      if(bGPSDConnectionEnabled_)
        {
          ACE_LSOCK_Stream gpsdControlStream;
          ACE_LSOCK_Connector connect;
          
          if(connect.connect(gpsdControlStream,gpsdControlAddr_) != -1)
            {
              std::string sTmp = sClientPTTYName_;
              sTmp.append(1,'-');
              sTmp.push_back('\n');
              gpsdControlStream.send(sTmp.c_str(),sTmp.length());
            }
        }

      // remove the pseudo terminal filename file
      pseudoTerminalNameFile_.unlink();
    }

  pPlatformService_->timerService().cancelTimedEvent(timerId_);
}

void EMANE::Agents:: GPSDLocation::Agent::destroy()
  throw()
{}

void EMANE::Agents::GPSDLocation::Agent::processEvent(const EventId & eventId, 
                                                       const Serialization & serialization)
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "GPSDLocation::Agent NEM: %hu processEvent Called",
                          nemId_);

  if(eventId == Events::LocationEvent::IDENTIFIER)
    {
      Events::LocationEvent event{serialization};
      
      const auto & locations = event.getLocations();
      
      auto iter =  std::find_if(locations.begin(),
                                locations.end(),
                                [this](const Events::Location & p)
                                {
                                  return p.getNEMId() == nemId_;
                                });
      
      if(iter != locations.end())
        {
          const auto & position = iter->getPosition();
          dLatitudeDegrees_ = position.getLatitudeDegrees();
          dLongitudeDegrees_ = position.getLongitudeDegrees();
          dAltitudeMeters_ = position.getAltitudeMeters();
          bHaveInitialPosition_ = true;

          auto velocity = iter->getVelocity();

          if(velocity.second)
            {
              dAzimuthDegrees_ = velocity.first.getAzimuthDegrees();
              dMagnitudeMetersPerSecond_ = velocity.first.getMagnitudeMetersPerSecond();
              bHaveInitialVelocity_ = true;
            }
        }
    }      
}

void  EMANE::Agents::GPSDLocation::Agent::processTimedEvent(TimerEventId,
                                                            const TimePoint &,
                                                            const TimePoint &,
                                                            const TimePoint &,
                                                            const void *)
{
  if(bHaveInitialPosition_)
    {
      sendSpoofedNMEA(dLatitudeDegrees_,dLongitudeDegrees_,dAltitudeMeters_);
    
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(), 
                              DEBUG_LEVEL,
                              "gpsdlocationaganet NEM: %hu lat: %lf lon: %lf alt:%lf",
                              nemId_,
                              dLatitudeDegrees_,
                              dLongitudeDegrees_,
                              dAltitudeMeters_);

      if(bHaveInitialVelocity_)
        {
          sendSpoofedGPVTG(dAzimuthDegrees_,dMagnitudeMetersPerSecond_);

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  DEBUG_LEVEL,
                                  "gpsdlocationagent NEM: %hu azm: %lf mag: %lf",
                                  nemId_,
                                  dAzimuthDegrees_,
                                  dMagnitudeMetersPerSecond_);
        }
    }
}

void  EMANE::Agents::GPSDLocation::Agent::sendSpoofedNMEA(double dLatitude, double dLongitude, double dAltitude)
{
  time_t t;
  tm tmval;
  char buf[1024];

  const int NUM_GSV_STRINGS   = 2;
  const int NUM_SV_PER_STRING = 4;

  int elv[NUM_GSV_STRINGS * NUM_SV_PER_STRING] = {41,   9,  70,  35,  10,  53,  2, 48 };
  int azm[NUM_GSV_STRINGS * NUM_SV_PER_STRING] = {104, 84,  30, 185, 297, 311, 29, 64 };
  int snr[NUM_GSV_STRINGS * NUM_SV_PER_STRING] = {41,  51,  39,  25,  25,  21, 29, 32 };

  char cLatitudeHemisphere  =  (dLatitude  > 0) ? 'N' : 'S';
  char cLongitudeHemisphere =  (dLongitude > 0) ? 'E' : 'W';
  
  if(dLatitude < 0)
    {
      dLatitude *= -1;
    }
  
  if(dLongitude < 0)
    {
      dLongitude *= -1;
    }
  
  int iLongitudeDegrees = static_cast<int>(dLongitude);
  int iLatitudeDegrees  = static_cast<int>(dLatitude);
  
  double dLongitudeMinutes = (dLongitude - iLongitudeDegrees) * 60.0;
  double dLatitudeMinutes  = (dLatitude - iLatitudeDegrees) * 60.0;
  
  int iLongitudeMinutes = static_cast<int>(dLongitudeMinutes);
  int iLatitudeMinutes  = static_cast<int>(dLatitudeMinutes);

  int iGPSquality = 2;
  int iNumSatellitesUsed = NUM_GSV_STRINGS * NUM_SV_PER_STRING;

  double dDOP = 1.8;
  double dHorizontalDOP = 1.1;
  double dVerticalDOP   = 1.3;

  double dGeoidalHeight = -34.0;
  
  dLongitudeMinutes -= iLongitudeMinutes;
  dLatitudeMinutes  -= iLatitudeMinutes;
  
  dLongitudeMinutes *= 10000;
  dLatitudeMinutes  *= 10000;
  
  ACE_OS::time(&t);
  ACE_OS::gmtime_r(&t,&tmval);

  /* NMEA GGA */
  snprintf(buf,sizeof(buf),"$GPGGA,%02d%02d%02d,%02d%02d.%04d,%c,%03d%02d.%04d,%c,%d,%02d,%.1f,%.1f,M,%.1f,M,,",
           tmval.tm_hour,
           tmval.tm_min,
           tmval.tm_sec,
           iLatitudeDegrees,
           iLatitudeMinutes,
           static_cast<int>(dLatitudeMinutes),
           cLatitudeHemisphere,
           iLongitudeDegrees,
           iLongitudeMinutes,
           static_cast<int>(dLongitudeMinutes),
           cLongitudeHemisphere,
           iGPSquality,
           iNumSatellitesUsed,
           dHorizontalDOP,
           dAltitude,
           dGeoidalHeight
           );
  
  doCheckSumNMEA(buf,sizeof(buf));
  write(masterPTY_,buf,strlen(buf));


  /* NMEA RMC */
  snprintf(buf,sizeof(buf),"$GPRMC,%02d%02d%02d,A,%02d%02d.%04d,%c,%03d%02d.%04d,%c,000.0,000.0,%02d%02d%02d,000.0,%c",
           tmval.tm_hour,
           tmval.tm_min,
           tmval.tm_sec,
           iLatitudeDegrees,
           iLatitudeMinutes,
           static_cast<int>(dLatitudeMinutes),
           cLatitudeHemisphere,
           iLongitudeDegrees,
           iLongitudeMinutes,
           static_cast<int>(dLongitudeMinutes),
           cLongitudeHemisphere,
           tmval.tm_mday,
           tmval.tm_mon + 1,
           (tmval.tm_year + 1900) % 100,
           cLongitudeHemisphere
           );
  
  doCheckSumNMEA(buf,sizeof(buf));
  write(masterPTY_,buf,strlen(buf));


  /* NMEA GSA */
  snprintf(buf,sizeof(buf), "$GPGSA,A,3,01,02,03,04,05,06,07,08,,,,,%2.1f,%2.1f,%2.1f",
           dDOP, dHorizontalDOP, dVerticalDOP);

  doCheckSumNMEA(buf,sizeof(buf));
  write(masterPTY_,buf,strlen(buf));
    
 
  for(int i = 0; i < NUM_GSV_STRINGS; ++i) 
    { 
      const int a = (NUM_SV_PER_STRING * i) + 0;
      const int b = (NUM_SV_PER_STRING * i) + 1;
      const int c = (NUM_SV_PER_STRING * i) + 2;
      const int d = (NUM_SV_PER_STRING * i) + 3;

      /* NMEA GSV */
      snprintf(buf,sizeof(buf), "$GPGSV,%d,%d,%02d,%02d,%02d,%03d,%02d,%02d,%02d,%03d,%02d,%02d,%02d,%03d,%02d,%02d,%02d,%03d,%02d",
               NUM_GSV_STRINGS,
               i + 1,
               iNumSatellitesUsed,
               a + 1,  elv[a], azm[a], snr[a],
               b + 2,  elv[b], azm[b], snr[b],
               c + 3,  elv[c], azm[c], snr[c],
               d + 4,  elv[d], azm[d], snr[d]);
 
      doCheckSumNMEA(buf,sizeof(buf));
      write(masterPTY_,buf,strlen(buf));
    }
}

void  EMANE::Agents::GPSDLocation::Agent::sendSpoofedGPVTG(double dAzimuth, double dMagnitude)
{
  char buf[1024];

  double dMagnitudeKnots = dMagnitude * 1.94384;
  double dMagnitudeKilometerPerHour = (dMagnitude * 3600.0) / 1000.0;

  /* NMEA GPVTG */
  snprintf(buf,sizeof(buf),"$GPVTG,%.1f,%C,%.1f,%C,%.1f,%C,%.1f,%C",
           dAzimuth,
           'T',
           dAzimuth,
           'M',
           dMagnitudeKnots,
           'N',
           dMagnitudeKilometerPerHour,
           'K'
           );

  doCheckSumNMEA(buf,sizeof(buf));
  write(masterPTY_,buf,strlen(buf));
}

void  EMANE::Agents::GPSDLocation::Agent::doCheckSumNMEA(char *buf, size_t len)
{
  char foo[6] = {0};

  char chksum = buf[1];
  
  for(unsigned int i = 2; i < strlen(buf); ++i)
    {
      chksum ^= buf[i];
    }
  
  snprintf(foo, sizeof(foo), "*%02X\r\n", static_cast<unsigned int>(chksum));
  
  strncat(buf, foo, len - strlen(buf) - 1);
}

DECLARE_EVENT_AGENT(EMANE::Agents::GPSDLocation::Agent);
