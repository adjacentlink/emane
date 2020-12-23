/*
 * Copyright (c) 2013,2020 - Adjacent Link LLC, Bridgewater, New Jersey
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

namespace
{
  inline
  EMANE::Orientation adjustOrientation(const EMANE::Orientation & orientation,
                                       const EMANE::Velocity & velocity)
  {
    double dYaw{velocity.getAzimuthDegrees() + orientation.getYawDegrees()};

    // set yaw to [0 to 360)
    EMANE::Utils::AI_TO_BE_DEGREES(dYaw, 0.0, 360.0);

    double dPitch{velocity.getElevationDegrees() + orientation.getPitchDegrees()};

    // set dpitch to [0 to 360)
    EMANE::Utils::AI_TO_BE_DEGREES(dPitch, 0.0, 360.0);

    // set pitch to [-90 to 90]
    EMANE::Utils::NEG_90_TO_POS_90_DEGREES(dPitch);

    return {orientation.getRollDegrees(),dPitch,dYaw};
  }
}

inline
EMANE::PositionOrientationVelocity::PositionOrientationVelocity():
  position_{},
  orientation_{},
  velocity_{},
  bValid_{},
  bHasOrientation_{},
  bHasVelocity_{}{}

inline
EMANE::PositionOrientationVelocity::PositionOrientationVelocity(const Position & position,
                                                                const std::pair<const Orientation &, bool> & orientation,
                                                                const std::pair<const Velocity &, bool> & velocity):
  position_{position},
  orientation_{orientation.first},
  velocity_{velocity.first},
  bValid_{true},
  bHasOrientation_{orientation.second},
  bHasVelocity_{velocity.second},
  positionECEF_{position},
  adjustedOrientation_{adjustOrientation(orientation_,velocity_)}{}

inline
bool EMANE::PositionOrientationVelocity::update(const Position & position,
                                                const std::pair<const Orientation &, bool> & orientation,
                                                const std::pair<const Velocity &, bool> & velocity)
{
  if(position == position_ &&
     (!orientation.second || orientation.first == orientation_) &&
     (!velocity.second || velocity.first == velocity_))
    {
      return false;
    }
  else
    {
      bValid_ = true;

      if(position != position_)
        {
          position_ = position;
          positionECEF_ = PositionECEF(position_);
        }

      bool bCalculateAdjustedOrientation{false};

      if(orientation.second)
        {
          orientation_ = orientation.first;
          bHasOrientation_ = true;
          bCalculateAdjustedOrientation = true;
        }

      if(velocity.second)
        {
          velocity_ = velocity.first;
          bHasVelocity_ = true;
          bCalculateAdjustedOrientation = true;
        }

      if(bCalculateAdjustedOrientation)
        {
          adjustedOrientation_ = adjustOrientation(orientation_,velocity_);
        }

      return true;
    }
}

inline
const EMANE::Position & EMANE::PositionOrientationVelocity::getPosition() const
{
  return position_;
}

inline
std::pair<const EMANE::Orientation &, bool> EMANE::PositionOrientationVelocity::getOrientation() const
{
  return {orientation_,bHasOrientation_};
}

inline
std::pair<const EMANE::Orientation &, bool> EMANE::PositionOrientationVelocity::getAdjustedOrientation() const
{
  return {adjustedOrientation_,bHasOrientation_ || bHasVelocity_};
}

inline
std::pair<const EMANE::Velocity &, bool>  EMANE::PositionOrientationVelocity::getVelocity() const
{
  return {velocity_,bHasVelocity_};
}

inline
const EMANE::PositionECEF &  EMANE::PositionOrientationVelocity::getPositionECEF() const
{
  return positionECEF_;
}

inline
EMANE::PositionNEU  EMANE::PositionOrientationVelocity::getPositionNEU(const PositionOrientationVelocity & other) const
{
  const auto & selfECEF = getPositionECEF();
  const auto & otherECEF = other.getPositionECEF();

  double dX{otherECEF.getX() - selfECEF.getX()};
  double dY{otherECEF.getY() - selfECEF.getY()};
  double dZ{ otherECEF.getZ() - selfECEF.getZ()};

  double dLatitudeRadians{position_.getLatitudeRadians()};
  double dLongitudeRadians{position_.getLongitudeRadians()};

  double dNorthMeters{-dX * sin(dLatitudeRadians) * cos(dLongitudeRadians) -
                      dY * sin(dLatitudeRadians) * sin(dLongitudeRadians) +
                      dZ * cos(dLatitudeRadians)};

  double dEastMeters{-dX * sin(dLongitudeRadians) + dY * cos(dLongitudeRadians)};

  double dUpMeters{dX * cos(dLatitudeRadians) * cos(dLongitudeRadians) +
                   dY * cos(dLatitudeRadians) * sin(dLongitudeRadians) +
                   dZ * sin(dLatitudeRadians)};

  PositionNEU otherNEU{dNorthMeters,dEastMeters,dUpMeters};

  if(bHasOrientation_ || bHasVelocity_)
    {
      otherNEU.rotate(adjustedOrientation_);
    }

  return otherNEU;
};

inline
bool EMANE::PositionOrientationVelocity::isValid() const
{
  return bValid_;
}
