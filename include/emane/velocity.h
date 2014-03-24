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

#ifndef EMANEVELOCITY_HEADER_
#define EMANEVELOCITY_HEADER_

#include <cstdint>

namespace EMANE
{
  /**
   * @class Velocity
   *
   * @brief Holds the velocity elements associated with an
   * NEM's location information
   *
   * @note Instances are immutable
   */
  class Velocity
  {
  public:
    /**
     * Creates a default Velocity
     */
    Velocity();

    /**
     * Creates a Velocity instance
     *
     * @param dAzimuthDegrees Azimuth vector component in degrees
     * @param dElevationDegrees Elevation vector component in degrees
     * @param dMagnitudeMetersPerSecond Total velocity scalar in meters/second
     */
    Velocity(double dAzimuthDegrees,
             double dElevationDegrees,
             double dMagnitudeMetersPerSecond);
    
    /**
     * Gets the azimuth vector component in degrees
     *
     * @return azimuth vector component
     */
    double getAzimuthDegrees() const;

    
    /**
     * Gets the elevation vector component in degrees
     *
     * @return elevation vector component
     */
    double getElevationDegrees() const;

    /**
     * Gets the azimuth vector component in radians
     *
     * @return azimuth vector component
     */
    double getAzimuthRadians() const;
    
    /**
     * Gets the elevation vector component in radians
     *
     * @return elevation vector component
     */
    double getElevationRadians() const;
      
    /**
     * Gets the total veclocity scalar in meters/second
     *
     * @return Total velocity scalar
     */
    double getMagnitudeMetersPerSecond() const;
     
    /**
     * Compares instance for equality
     * 
     * @return @a True if equal @a False otherwise
     */
    bool operator==(const Velocity & rhs) const;

  private:
    double dAzimuthDegrees_;
    double dElevationDegrees_;
    double dMagnitudeMetersPerSecond_;
    double dAzimuthRadians_;
    double dElevationRadians_;
  };
}

#include "emane/velocity.inl"

#endif //EMANEVELOCITY_HEADER_
