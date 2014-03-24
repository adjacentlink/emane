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

#ifndef EMANEORIENTATION_HEADER_
#define EMANEORIENTATION_HEADER_

namespace EMANE
{
  /**
   * @class Orientation
   *
   * @brief Holds pitch, yaw and roll
   * 
   * @note Instances are immutable
   */
  class Orientation
  {
  public:
    /**
     * Creates an Orientation instance with pitch,
     * yaw and roll of 0.
     */
    Orientation();
      
    /**
     * Creates an Orientation instance
     *
     * @param dRollDegrees Roll in degrees
     * @param dPitchDegrees Pitch in degrees
     * @param dYawDegrees Yaw in degrees
     */
    Orientation(double dRollDegrees,
                double dPitchDegrees,
                double dYawDegrees);
      
    /**
     * Gets roll in degrees
     *
     * @return roll
     */
    double getRollDegrees() const;
      
    /**
     * Gets pitch in degrees
     *
     * @return pitch
     */
    double getPitchDegrees() const;

    /**
     * Gets yaw in degrees
     *
     * @return yaw
     */
    double getYawDegrees() const;

    /**
     * Gets roll in radians
     *
     * @return roll
     */
    double getRollRadians() const;
      
    /**
     * Gets pitch in radians
     *
     * @return roll
     */
    double getPitchRadians() const;

    /**
     * Gets yaw in radians
     *
     * @return yaw
     */
    double getYawRadians() const;

    /**
     * Determines if another instance is equal
     *
     * @return @a true if equal, @a false if not
     */
    bool operator==(const Orientation & rhs) const;

  private:
    double dRollDegrees_;
    double dPitchDegrees_;
    double dYawDegrees_;
    double dRollRadians_;
    double dPitchRadians_;
    double dYawRadians_;
  };
}

#include "emane/orientation.inl"

#endif // EMANEORIENTATION_HEADER_
