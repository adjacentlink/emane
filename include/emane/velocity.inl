/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
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

inline
EMANE::Velocity::Velocity():
  dAzimuthDegrees_{},
  dElevationDegrees_{},
  dMagnitudeMetersPerSecond_{},
  dAzimuthRadians_{},
  dElevationRadians_{}{}

inline
EMANE::Velocity::Velocity(double dAzimuthDegrees,
                                  double dElevationDegrees,
                                  double dMagnitudeMetersPerSecond):
  dAzimuthDegrees_{dAzimuthDegrees},
  dElevationDegrees_{dElevationDegrees},
  dMagnitudeMetersPerSecond_{dMagnitudeMetersPerSecond},
  dAzimuthRadians_{Utils::DEGREES_TO_RADIANS(dAzimuthDegrees)},
  dElevationRadians_{Utils::DEGREES_TO_RADIANS(dElevationDegrees)}{}

inline
double EMANE::Velocity::getAzimuthDegrees() const
{
  return dAzimuthDegrees_;
}

inline
double EMANE::Velocity::getElevationDegrees() const
{
  return dElevationDegrees_;
}

inline
double EMANE::Velocity::getMagnitudeMetersPerSecond() const
{
  return dMagnitudeMetersPerSecond_;
}

inline
double EMANE::Velocity::getAzimuthRadians() const
{
  return dAzimuthRadians_;
}

inline
double EMANE::Velocity::getElevationRadians() const
{
  return dElevationRadians_;
}

inline
bool  EMANE::Velocity::operator==(const Velocity & rhs) const
{
  return dAzimuthDegrees_ == rhs.dAzimuthDegrees_ &&
    dElevationDegrees_ == rhs.dElevationDegrees_ &&
    dMagnitudeMetersPerSecond_ == rhs.dMagnitudeMetersPerSecond_;
}
