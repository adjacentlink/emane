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

#ifndef EMANEPOSITION_HEADER_
#define EMANEPOSITION_HEADER_

#include <cstdint>

namespace EMANE
{
  /**
   * @class Position
   *
   * @brief Holds latitude, longitude and altitude
   * 
   * @note Instances are immutable
   */
  class Position
  {
  public:
    /**
     * Creates a Position instance with a latitude,
     * longitude and altitude of 0.
     */
    Position();

    /**
     * Creates a Position instance
     *
     * @param dLatitudeDegrees Latitude in degrees
     * @param dLongitudeDegrees Longitude in degrees
     * @param dAltitudeMeters Altitude in meters
     */
    Position(double dLatitudeDegrees,
             double dLongitudeDegrees,
             double dAltitudeMeters);
      
    /**
     * Gets the latitude in degrees
     *
     * @return latitude
     */
    double getLatitudeDegrees() const;

    /**
     * Gets the longitude in degrees
     *
     * @return longitude
     */
    double getLongitudeDegrees() const;
      
    /**
     * Gets the altitude in meters
     *
     * @return altitude
     */
    double getAltitudeMeters() const;
     
    /**
     * Gets the latitude in radians
     *
     * @return latitude
     */
    double getLatitudeRadians() const;
    
    /**
     * Gets the longitude in radians
     *
     * @return longitude
     */ 
    double getLongitudeRadians() const;
      
    /**
     * Determines if another instance is equal
     *
     * @return @a true if equal, @a false if not
     */
    bool operator==(const Position & rhs) const;

    /**
     * Determines if another instance is not equal
     *
     * @return @a true if not equal, @a false if equal
     */
    bool operator!=(const Position & rhs) const;
      
  private:
    double dLatitudeDegrees_;
    double dLongitudeDegrees_;
    double dAltitudeMeters_;
    double dLatitudeRadians_;
    double dLongitudeRadians_;
    double dReserved_{};
  };
}

#include "emane/position.inl"

#endif //EMANEPOSITION_HEADER_
