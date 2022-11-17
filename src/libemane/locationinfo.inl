/*
 * Copyright (c) 2013,2020-2021 - Adjacent Link LLC, Bridgewater,
 * New Jersey
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
EMANE::LocationInfo::LocationInfo():
  localPOV_{},
  remotePOV_{},
  dDistanceMeters_{},
  u64SequenceNumber_{},
  dDopplerFactor_{1}{}

inline
EMANE::LocationInfo::LocationInfo(const PositionOrientationVelocity & localPOV,
                                  const PositionOrientationVelocity & remotePOV,
                                  std::uint64_t u64SequenceNumber):
  localPOV_{localPOV},
  remotePOV_{remotePOV},
  dDistanceMeters_{},
  u64SequenceNumber_{u64SequenceNumber},
  dDopplerFactor_{1}
{
  if(localPOV.getPosition() != remotePOV.getPosition())
    {
      const auto & pos1 = localPOV.getPositionECEF();
      const auto & pos2 = remotePOV.getPositionECEF();

      // position vector remote (transmitter) relative
      // to local (receiver)
      double dPosX{pos2.getX() - pos1.getX()};
      double dPosY{pos2.getY() - pos1.getY()};
      double dPosZ{pos2.getZ() - pos1.getZ()};

      // return distance
      dDistanceMeters_ =  Utils::NORMALIZE_VECTOR(dPosX,
                                                  dPosY,
                                                  dPosZ);

      // calculate doppler factor
      const auto & vel1 =  localPOV.getVelocityECEF();
      const auto & vel2 =  remotePOV.getVelocityECEF();

      if(vel1.isValid() && vel2.isValid() && dDistanceMeters_ > 0)
        {
          // velocity vector remote (transmitter) relative
          // to local (receiver)
          double dVelX{vel1.getX() - vel2.getX()};
          double dVelY{vel1.getY() - vel2.getY()};
          double dVelZ{vel1.getZ() - vel2.getZ()};

          double dDotProduct = dPosX * dVelX + dPosY * dVelY + dPosZ * dVelZ;
          double dMagnitudeVelocityVector = Utils::NORMALIZE_VECTOR(dVelX,dVelY,dVelZ);
          double dCosTheta = dDotProduct/(dDistanceMeters_*dMagnitudeVelocityVector);

          if(dMagnitudeVelocityVector > 0)
            {
              dDopplerFactor_ =  SOL_MPS/(SOL_MPS - (dMagnitudeVelocityVector * dCosTheta));
            }
        }
    }
}

inline
const EMANE::PositionOrientationVelocity & EMANE::LocationInfo::getLocalPOV() const
{
  return localPOV_;
}

inline
const EMANE::PositionOrientationVelocity & EMANE::LocationInfo::getRemotePOV() const
{
  return remotePOV_;
}

inline
double EMANE::LocationInfo::getDistanceMeters() const
{
  return dDistanceMeters_;
}

inline
std::uint64_t EMANE::LocationInfo::getSequenceNumber() const
{
  return u64SequenceNumber_;
}

inline
bool EMANE::LocationInfo::isValid() const
{
  return localPOV_.isValid() && remotePOV_.isValid();
}

inline
double EMANE::LocationInfo::getDopplerFactor() const
{
  return dDopplerFactor_;
}
