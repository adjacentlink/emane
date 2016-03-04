/*
 * Copyright (c) 2013-2014,2016 - Adjacent Link LLC, Bridgewater, New
 * Jersey
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

#include <pty.h>
#include <termios.h>
#include <unistd.h>
#include <climits>
#include <fstream>
#include <cstring>

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

  configRegistrar.registerNonNumeric<std::string>("pseudoterminalfile",
                                                  ConfigurationProperties::DEFAULT,
                                                  {"/tmp/gps.pty"},
                                                  "File to write the name of the created pseudo term.");

  auto & eventRegistrar = registrar.eventRegistrar();

  eventRegistrar.registerEvent(Events::LocationEvent::IDENTIFIER);
}

void EMANE::Agents::GPSDLocation::Agent::configure(const ConfigurationUpdate & update)
{
  for(const auto & item : update)
    {
      if(item.first == "pseudoterminalfile")
        {
          sPseudoTerminalFile_ = item.second[0].asString();
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
  char pzName[PATH_MAX]{};

  if(openpty(&masterPTY_, &slavePTY_, nullptr,nullptr,nullptr) == -1)
    {
      throw StartException("Unable to open pseudo terminal for gpsd");
    }

  if(ttyname_r(slavePTY_,&pzName[1],sizeof(pzName)-1) == -1)
    {
      throw StartException("Unable to determin pseudo terminal name");
    }

  struct termios term;

  tcgetattr(slavePTY_,&term);

  term.c_iflag = 0;
  term.c_oflag = 0;
  term.c_lflag = 0;

  term.c_cflag &=~ (PARENB | PARODD | CRTSCTS | CSIZE |
                    CSTOPB | PARENB | PARODD | CS8);

  term.c_cflag |= CREAD | CLOCAL;

  term.c_iflag &=~ (PARMRK | INPCK);

  term.c_cc[VMIN] = 1;

  cfsetispeed(&term, B4800);
  cfsetospeed(&term, B4800);

  tcsetattr(slavePTY_,TCSANOW,&term);

  close(slavePTY_);

  // open up the file containing the name of the pseudo ternimal
  std::ofstream ofs{sPseudoTerminalFile_.c_str(),std::ios::trunc};

  ofs<<&pzName[1]<<std::endl;

  ofs.close();

  std::chrono::seconds interval{1};

  timerId_ = pPlatformService_->timerService().scheduleTimedEvent(Clock::now() + interval,
                                                                  nullptr,
                                                                  interval);

  LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(), DEBUG_LEVEL, "Timer Scheduled");
}

void EMANE::Agents::GPSDLocation::Agent::stop()
{
  // remove the pseudo terminal filename file
  LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(), DEBUG_LEVEL, "unlink %s",
                         sPseudoTerminalFile_.c_str());

  unlink(sPseudoTerminalFile_.c_str());

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

  time(&t);
  gmtime_r(&t,&tmval);

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
