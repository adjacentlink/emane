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
EMANE::PositionECEF::PositionECEF():
  dX_{},
  dY_{},
  dZ_{},
  bValid_{false}{}

inline
EMANE::PositionECEF::PositionECEF(const Position & position):
  bValid_{true}
{
 double dLatitudeRadians{position.getLatitudeRadians()};
 
 double dLongitudeRadians{position.getLongitudeRadians()};
 
 double dAltitudeMeters{position.getAltitudeMeters()};
 
 double R{EMANE::SEMI_MAJOR / sqrt(1.0 - (EMANE::ECC2 * (pow(sin(dLatitudeRadians), 2.0))))};
 
 dX_ = (R + dAltitudeMeters)  * cos(dLatitudeRadians) * cos(dLongitudeRadians);
 
 dY_ = (R + dAltitudeMeters)  * cos(dLatitudeRadians) * sin(dLongitudeRadians);
 
 dZ_ = ((1.0 - ECC2) * R + dAltitudeMeters) * sin(dLatitudeRadians);
}

inline
double EMANE::PositionECEF::getX() const
{
  return dX_;
}

inline
double EMANE::PositionECEF::getY() const
{
  return dY_;
}

inline
double EMANE::PositionECEF::getZ() const
{
  return dZ_;
}

inline
bool EMANE::PositionECEF::operator!() const
{
  return bValid_==false;
}
