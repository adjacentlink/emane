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

#ifndef EMANEPHYPOSITIONUTILS_HEADER_
#define EMANEPHYPOSITIONUTILS_HEADER_

#include "emane/models/frameworkphy/locationinfo.h"
#include "emane/models/frameworkphy/positionneu.h"

#include "emane/utils/conversionutils.h"

#include <tuple>

namespace EMANE
{
  namespace Utils
  {
    inline
    std::tuple<double,double,double>
    calculateDirection(const PositionOrientationVelocity & localLocationInfo,
                       const PositionNEU & localAntennaPositionNEU,
                       const PositionOrientationVelocity & remoteLocationInfo,
                       const PositionNEU & remoteAntennaPositionNEU)
    {
      // remote placement
      //PositionNEU remoteAntennaPositionNEU{remoteAntennaPositionNEU};
      
      // NEU for the remote node, includes rotation for orientation
      PositionNEU remoteNEU{localLocationInfo.getPositionNEU(remoteLocationInfo)};

      PositionNEU mutableRemoteAntennaPositionNEU{remoteAntennaPositionNEU};

      const auto & localOrientation = localLocationInfo.getAdjustedOrientation().first;
      const auto & remoteOrientation = remoteLocationInfo.getAdjustedOrientation().first;

      // rotate the remote antenna position based on local and remote orientation
      mutableRemoteAntennaPositionNEU.rotate(localOrientation.getYawRadians() - remoteOrientation.getYawRadians(),
                                             localOrientation.getPitchRadians() - remoteOrientation.getPitchRadians(),
                                             localOrientation.getRollRadians() - remoteOrientation.getRollRadians());
      
      // adjust the remote NEM to include antenna placement -  accounts for both local and remote platform
      
      // Update NEU to include antenna location on the platform.  This accounts for both local and remote platforms.
      remoteNEU.adjust(-localAntennaPositionNEU.getNorthMeters() + mutableRemoteAntennaPositionNEU.getNorthMeters(),
                       -localAntennaPositionNEU.getEastMeters()  + mutableRemoteAntennaPositionNEU.getEastMeters(),
                       -localAntennaPositionNEU.getUpMeters()    + mutableRemoteAntennaPositionNEU.getUpMeters());
                       
      // get distance
      double dDistanceMeters{NORMALIZE_VECTOR(remoteNEU.getNorthMeters(), remoteNEU.getEastMeters(), remoteNEU.getUpMeters())};
      
      // get elevation
      double dElevationDegrees{asin(remoteNEU.getUpMeters() / dDistanceMeters) * (180.0 / M_PI)};
      
      // get azimuth
      double dAzimuthDegrees{};
      
      if(remoteNEU.getNorthMeters() == 0.0)
        {
          if( remoteNEU.getEastMeters() > 0.0)
            {
              dAzimuthDegrees = 90.0;
            }
          else
            {
              dAzimuthDegrees = 270.0;
            }
        }
      else
        {
          if(remoteNEU.getEastMeters() == 0.0)
            {
              if(remoteNEU.getNorthMeters() > 0.0)
                {
                  dAzimuthDegrees = 0.0;
                }
              else
                {
                  dAzimuthDegrees = 180.0;
                }
            }
          else                  
            {
              dAzimuthDegrees = atan(remoteNEU.getEastMeters() / remoteNEU.getNorthMeters()) * (180.0 / M_PIl);
            }
          
          if(remoteNEU.getNorthMeters() < 0.0)
            {
              dAzimuthDegrees += 180.0;
            }
          
          if((remoteNEU.getNorthMeters() > 0.0) && ( remoteNEU.getEastMeters() < 0.0))
            {
              dAzimuthDegrees += 360.0;
            }
        }
      
      return std::make_tuple(dAzimuthDegrees, dElevationDegrees, dDistanceMeters);
    }
  
  inline std::pair<double,double>
  calculateLookupAngles(double dAzReference,
                        double dAzPointing,
                        double dElReference,
                        double dElPointing)
  {
    // Az = Az(reference) - Az(pointing)
    double dAzimuth {dAzReference - dAzPointing};
    
    // El = El(reference) - El(pointing)
    double dElevation = {dElReference - dElPointing};
    
    // ensure (0.0 <= Az < 360.0)
    Utils::AI_TO_BE_DEGREES(dAzimuth, 0.0, 360.0);
    
    // ensure (0.0 <= El < 360.0)
    Utils::AI_TO_BE_DEGREES(dElevation, 0.0, 360.0);
    
    // ensure (-90 <= El <= 90)
    Utils::NEG_90_TO_POS_90_DEGREES(dElevation);
    
    return {dAzimuth, dElevation};
  }

  inline bool 
  checkHorizon(const double dHeightMeters1,
               const double dHeightMeters2,
               const double dDistanceMeters)
  {
    const double dDH1{3570.0 * sqrt(dHeightMeters1 < 0.0 ? 0.0 : dHeightMeters1)};
    
    const double dDH2{3570.0 * sqrt(dHeightMeters2 < 0.0 ? 0.0 : dHeightMeters2)};

    return (dDH1 + dDH2) > dDistanceMeters;
  }
  }
}

#endif // EMANEPHYPOSITIONUTILS_HEADER_

