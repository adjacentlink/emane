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

#include "emane/utils/conversionutils.h"

inline
EMANE::Orientation::Orientation():
  dRollDegrees_{},
  dPitchDegrees_{},
  dYawDegrees_{},
  dRollRadians_{},
  dPitchRadians_{},
  dYawRadians_{}{}

inline
EMANE::Orientation::Orientation(double dRollDegrees,
                                        double dPitchDegrees,
                                        double dYawDegrees):
  dRollDegrees_{dRollDegrees},
  dPitchDegrees_{dPitchDegrees},
  dYawDegrees_{dYawDegrees},
  dRollRadians_{Utils::DEGREES_TO_RADIANS(dRollDegrees)},
  dPitchRadians_{Utils::DEGREES_TO_RADIANS(dPitchDegrees)},
  dYawRadians_{Utils::DEGREES_TO_RADIANS(dYawDegrees)}{}

inline
double EMANE::Orientation::getRollDegrees() const
{
  return dRollDegrees_;
}

inline
double EMANE::Orientation::getPitchDegrees() const
{
  return dPitchDegrees_;
}

inline
double EMANE::Orientation::getYawDegrees() const
{
  return dYawDegrees_;
}

inline
double EMANE::Orientation::getRollRadians() const
{
  return dRollRadians_;
}

inline
double EMANE::Orientation::getPitchRadians() const
{
  return dPitchRadians_;
}

inline
double EMANE::Orientation::getYawRadians() const
{
  return dYawRadians_;
}

inline
bool EMANE::Orientation::operator==(const Orientation & rhs) const
{
  return dRollDegrees_ == rhs.dRollDegrees_ &&
    dPitchDegrees_ == rhs.dPitchDegrees_ &&
    dYawDegrees_ == rhs.dYawDegrees_;
}
