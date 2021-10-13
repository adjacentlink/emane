/*
 * Copyright (c) 2021 - Adjacent Link LLC, Bridgewater, New Jersey
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
EMANE::VelocityECEF::VelocityECEF():
  dX_{},
  dY_{},
  dZ_{},
  bValid_{false}{}

inline
EMANE::VelocityECEF::VelocityECEF(const VelocityNEU & velocityNEU,
                                  const Position & position):
  dX_{},
  dY_{},
  dZ_{},
  bValid_{true}
{
  const double dSinLongitude{sin(position.getLongitudeRadians())};
  const double dSinLatitude{sin(position.getLatitudeRadians())};
  const double dCosLongitude{cos(position.getLongitudeRadians())};
  const double dCosLatitude{cos(position.getLatitudeRadians())};

  // XvT = -EvT*sin(lonT) -NvT*sin(latT)*cos(lonT) + UvT*cos(latT)*cos(lonT);
  dX_ =
    -velocityNEU.getEastMetersPerSecond() * dSinLongitude -
    velocityNEU.getNorthMetersPerSecond() * dSinLatitude * dCosLongitude +
    velocityNEU.getUpMetersPerSecond() * dCosLongitude * dCosLongitude;

  // YvT = EvT*cos(lonT) -NvT*sin(latT)*sin(lonT) + UvT*cos(latT)*sin(lonT);
  dY_ =
    velocityNEU.getEastMetersPerSecond() * dCosLongitude -
    velocityNEU.getNorthMetersPerSecond() * dSinLatitude * dSinLongitude +
    velocityNEU.getUpMetersPerSecond() * dCosLatitude * dSinLongitude;

  // ZvT = EvT*0 + NvT*cos(latT) + UvT*sin(lonT);
  dZ_ =
    velocityNEU.getNorthMetersPerSecond() * dCosLatitude +
    velocityNEU.getUpMetersPerSecond() * dSinLongitude;
}

inline
double EMANE::VelocityECEF::getX() const
{
  return dX_;
}

inline
double EMANE::VelocityECEF::getY() const
{
  return dY_;
}

inline
double EMANE::VelocityECEF::getZ() const
{
  return dZ_;
}

inline
bool EMANE::VelocityECEF::isValid() const
{
  return bValid_;
}
