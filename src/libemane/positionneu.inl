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
EMANE::PositionNEU::PositionNEU():
  dNorthMeters_{},
  dEastMeters_{},
  dUpMeters_{}{}

inline
EMANE::PositionNEU::PositionNEU(double dNorthMeters,double dEastMeters, double dUpMeters):
  dNorthMeters_{dNorthMeters},
  dEastMeters_{dEastMeters},
  dUpMeters_{dUpMeters}{}

inline     
double EMANE::PositionNEU::getNorthMeters() const
{
  return dNorthMeters_;
}

inline
double EMANE::PositionNEU::getEastMeters() const
{
  return dEastMeters_;
}

inline     
double EMANE::PositionNEU::getUpMeters() const
{
  return dUpMeters_;
}

inline
void EMANE::PositionNEU::rotate(const Orientation & orientation)
{
  rotate(orientation.getYawRadians(),
         orientation.getPitchRadians(),
         orientation.getRollRadians());
}

inline
void EMANE::PositionNEU::rotate(double dYawRadians, double dPitchRadians, double dRollRadians)
{
  // check if rotation needed
  if((dYawRadians != 0.0) || (dPitchRadians != 0.0) || (dRollRadians != 0.0))
    {
      // order of rotion applied here is yaw, pitch, roll
      double dRotatedNorthMeters = 
        dNorthMeters_ * cos(dYawRadians) * cos(dPitchRadians) + 
        dEastMeters_  * sin(dYawRadians) * cos(dPitchRadians) +
        dUpMeters_    * sin(dPitchRadians);
            
      double dRotatedEastMeters =
        dNorthMeters_ * (cos(dYawRadians) * sin(dPitchRadians) * sin(dRollRadians) -
                         sin(dYawRadians)      * cos(dRollRadians))  +
        dEastMeters_  * (cos(dYawRadians)   * cos(dRollRadians)  +
                         sin(dYawRadians)      * sin(dPitchRadians)  * sin(dRollRadians)) -
        dUpMeters_    * (cos(dPitchRadians) * sin(dRollRadians));
            
      double dRotatedUpMeters =
        -dNorthMeters_ * (cos(dYawRadians)  * sin(dPitchRadians) * cos(dRollRadians) +
                         sin(dYawRadians)   * sin(dRollRadians)) -
        dEastMeters_  * (sin(dYawRadians)   * sin(dPitchRadians) * cos(dRollRadians) -
                         cos(dYawRadians)   * sin(dRollRadians)) +
        dUpMeters_    * (cos(dPitchRadians) * cos(dRollRadians));
                        
      dNorthMeters_ = dRotatedNorthMeters;
      dEastMeters_ = dRotatedEastMeters;
      dUpMeters_ = dRotatedUpMeters;
    }
}
      
inline
void EMANE::PositionNEU::adjust(double dNorthMeters,double dEastMeters, double dUpMeters)
{
  dNorthMeters_ +=  dNorthMeters;
  dEastMeters_ += dEastMeters;
  dUpMeters_ += dUpMeters;
}
