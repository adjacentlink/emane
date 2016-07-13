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

#ifndef EMANEEVENTSANTENNAPROFILE_HEADER_
#define EMANEEVENTSANTENNAPROFILE_HEADER_

#include "emane/types.h"

#include <list>

namespace EMANE
{
  namespace Events
  {
    /**
     * @class AntennaProfile
     *
     * @brief Holds NEM Id, antenna profile Id, azimuth and elevation
     *
     * @note Instances are immutable
     */
    class AntennaProfile
    {
    public:
      /**
       * Creates an AntennaProfile instance with a NEM Id, profile id
       * azimuth and elevation of 0.
       */
      AntennaProfile();

      /**
       * Creates an AntennaProfile instance
       *
       * @param nemId Id of NEM
       * @param antennaProfileId Antenna profile id
       * @param dAntennaAzimuthDegrees Antenna azimuth in degrees
       * @param dAntennaElevationDegrees Antenna elevation in degrees
       */
      AntennaProfile(NEMId nemId,
                     AntennaProfileId antennaProfileId,
                     double dAntennaAzimuthDegrees,
                     double dAntennaElevationDegrees);

      /**
       * Gets the NEM Id
       *
       * @return NEM Id
       */
      NEMId getNEMId() const;

      /**
       * Gets the antenna profile Id
       *
       * @return profile Id
       */
      AntennaProfileId getAntennaProfileId() const;

      /**
       * Gets the antenna pointing azimuth in degrees
       *
       * @return azimuth
       */
      double getAntennaAzimuthDegrees() const;

      /**
       * Gets the antenna pointing elevation in degrees
       *
       * @return elevation
       */
      double getAntennaElevationDegrees() const;

      /**
       * Determines if another instance is equal
       *
       * @return @a true if equal, @a false if not
       */
      bool operator==(const AntennaProfile & rhs) const;

    private:
      NEMId nemId_;
      AntennaProfileId antennaProfileId_;
      double dAntennaAzimuthDegrees_;
      double dAntennaElevationDegrees_;
      bool bValid_;
    };

    using AntennaProfiles = std::list<AntennaProfile>;
  }
}

#include "emane/events/antennaprofile.inl"

#endif // EMANEEVENTSANTENNAPROFILE_HEADER_
