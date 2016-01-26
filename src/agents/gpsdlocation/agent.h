/*
 * Copyright (c) 2013-2014,2016 - Adjacent Link LLC, Bridgewater, New
 * Jersey
 * Copyright (c) 2008 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANEAGENTSGPSDLOCATIONAGENT_HEADER_
#define EMANEAGENTSGPSDLOCATIONAGENT_HEADER_

#include "emane/eventagent.h"
#include "emane/types.h"

#include <string>

namespace EMANE
{
  namespace Agents
  {
    namespace GPSDLocation
    {
      /**
       * @class Agent
       *
       * @brief  Agent translating location events to NMEA gpsd messages
       */
      class Agent : public EventAgent
      {
      public:
        Agent(NEMId nemId,
              PlatformServiceProvider *pPlatformService);

        ~Agent();

        void initialize(Registrar & registrar) override;

        void configure(const ConfigurationUpdate & update) override;

        void start() override;

        void stop() override;

        void destroy() throw() override;

        void processEvent(const EventId&,
                          const Serialization &) override;

        void processTimedEvent(TimerEventId eventId,
                               const TimePoint & expireTime,
                               const TimePoint & scheduleTime,
                               const TimePoint & fireTime,
                               const void * arg) override;

      private:
        NEMId nemId_;
        int masterPTY_;
        int slavePTY_;
        std::string sClientPTTYName_;
        std::string sPseudoTerminalFile_;
        std::string sPseudoTerminalNameFile_;
        bool bHaveInitialPosition_;
        bool bHaveInitialVelocity_;
        double dLatitudeDegrees_;
        double dLongitudeDegrees_;
        double dAltitudeMeters_;
        double dAzimuthDegrees_;
        double dMagnitudeMetersPerSecond_;
        TimerEventId timerId_;

        /**
         * Convert position in to NMEA strings and send to gpsd via pseudo  terminal
         *
         * @param fLatitude   Latitude in degrees
         * @param fLongitude  Longiutde in degrees
         * @param fAltitude Altitude in meters
         *
         */
        void sendSpoofedNMEA(double dLatitude, double dLongitude, double dAltitude);

        /**
         * Convert attitude in to NMEA strings and send to gpsd via pseudo  terminal
         *
         * @param dAzimuth   Azimuth in degrees
         * @param dMagnitude Magnitude in meters per second
         *
         */
        void sendSpoofedGPVTG(double dAzimuth, double dMagnitude);

        void doCheckSumNMEA(char *buf, size_t len);
      };
    }
  }
}

#endif // EMANEAGENTSGPSDLOCATIONAGENT_HEADER_
