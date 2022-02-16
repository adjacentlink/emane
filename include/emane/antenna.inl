/*
 * Copyright (c) 2020-2021 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "emane/antenna.h"
#include <cmath>

inline
EMANE::Antenna::Pointing::Pointing():
  antennaProfileId_{},
  dAzimuthDegrees_{},
  dElevationDegrees_{},
  bIsValid_{}{}

inline
EMANE::Antenna::Pointing::Pointing(AntennaProfileId antennaProfileId,
                                   double dAzimuthDegrees,
                                   double dElevationDegrees):
  antennaProfileId_{antennaProfileId},
  dAzimuthDegrees_{dAzimuthDegrees},
  dElevationDegrees_{dElevationDegrees},
  bIsValid_{true}{}

inline
EMANE::AntennaProfileId EMANE::Antenna::Pointing::getProfileId() const
{
  return antennaProfileId_;
}

inline
double EMANE::Antenna::Pointing::getAzimuthDegrees() const
{
  return dAzimuthDegrees_;
}

inline
double EMANE::Antenna::Pointing::getElevationDegrees() const
{
  return dElevationDegrees_;
}

inline
bool EMANE::Antenna::Pointing::operator==(const Pointing & rhs) const
{
  return (antennaProfileId_ == rhs.antennaProfileId_ &&
          fabs(dAzimuthDegrees_ - rhs.dAzimuthDegrees_) < .00001 &&
          fabs(dElevationDegrees_ - rhs.dElevationDegrees_) < .00001);
}

inline
bool EMANE::Antenna::Pointing::isValid() const
{
  return bIsValid_;
}

inline
EMANE::Antenna EMANE::Antenna::createDefault()
{
  return {};
}

inline
EMANE::Antenna EMANE::Antenna::createIdealOmni(AntennaIndex antennaIndex,
                                               double dFixedGaindBi)
{
  return {antennaIndex,dFixedGaindBi};
}

inline
EMANE::Antenna EMANE::Antenna::createProfileDefined(AntennaIndex antennaIndex,
                                                    const Pointing & pointing)
{
  return {antennaIndex,pointing};
}

inline
EMANE::Antenna::Antenna():
  antennaIndex_{0},
  dFixedGaindBi_{0},
  pointing_{},
  bIsIdealOmni_{false},
  bIsProfileDefined_{false},
  frequencyGroupIndex_{},
  u64BandwidthHz_{},
  spectralMaskIndex_{DEFAULT_SPECTRAL_MASK_INDEX}{}

inline
bool EMANE::Antenna::isDefault() const
{
  return !(bIsIdealOmni_ || bIsProfileDefined_);
}

inline
bool EMANE::Antenna::isIdealOmni() const
{
  return bIsIdealOmni_;
}

inline
bool EMANE::Antenna::isProfileDefined() const
{
  return bIsProfileDefined_;
}

inline
std::pair<double,bool>
EMANE::Antenna::getFixedGaindBi() const
{
  return {dFixedGaindBi_,bIsIdealOmni_};
}

inline
std::pair<const EMANE::Antenna::Pointing &,bool> EMANE::Antenna::getPointing() const
{
  return {pointing_, bIsProfileDefined_ && pointing_.isValid()};
}

inline
EMANE::AntennaIndex EMANE::Antenna::getIndex() const
{
  return antennaIndex_;
}

inline
EMANE::FrequencyGroupIndex EMANE::Antenna::getFrequencyGroupIndex() const
{
  return frequencyGroupIndex_;
}

inline
void EMANE::Antenna::setPointing(const Pointing & pointing)
{
  pointing_ = pointing;
}

inline
void EMANE::Antenna::setFrequencyGroupIndex(FrequencyGroupIndex index)
{
  frequencyGroupIndex_ = index;
}

inline
std::uint64_t EMANE::Antenna::getBandwidthHz() const
{
  return u64BandwidthHz_;
}

inline
void EMANE::Antenna::setBandwidthHz(std::uint64_t u64BandwidthHz)
{
  u64BandwidthHz_ = u64BandwidthHz;
}

inline
EMANE::SpectralMaskIndex EMANE::Antenna::getSpectralMaskIndex() const
{
  return spectralMaskIndex_;
}

inline
void EMANE::Antenna::setSpectralMaskIndex(SpectralMaskIndex spectralMaskIndex)
{
  spectralMaskIndex_ = spectralMaskIndex;
}

inline
void EMANE::Antenna::setFixedGaindBi(double dFixedGaindBi)
{
  dFixedGaindBi_ = dFixedGaindBi;
}

inline
bool EMANE::Antenna::operator==(const Antenna & rhs) const
{
  if(antennaIndex_ == rhs.antennaIndex_ &&
     u64BandwidthHz_ == rhs.u64BandwidthHz_ &&
     bIsIdealOmni_ == rhs.bIsIdealOmni_ &&
     bIsProfileDefined_ == rhs.bIsProfileDefined_)
    {
      if(bIsIdealOmni_)
        {
          return fabs(dFixedGaindBi_ - rhs.dFixedGaindBi_) < .00001;
        }
      else
        {
          return pointing_ == rhs.pointing_;
        }
    }

  return false;
}

inline
bool EMANE::Antenna::operator!=(const Antenna & rhs) const
{
  return !(*this == rhs);
}

inline
EMANE::Antenna::Antenna(AntennaIndex antennaIndex,
                        double dFixedGaindBi):
  antennaIndex_{antennaIndex},
  dFixedGaindBi_{dFixedGaindBi},
  pointing_{},
  bIsIdealOmni_{true},
  bIsProfileDefined_{},
  frequencyGroupIndex_{},
  u64BandwidthHz_{},
  spectralMaskIndex_{DEFAULT_SPECTRAL_MASK_INDEX}{}

inline
EMANE::Antenna::Antenna(AntennaIndex antennaIndex,
                        const Pointing & pointing):
  antennaIndex_{antennaIndex},
  dFixedGaindBi_{},
  pointing_{pointing},
  bIsIdealOmni_{},
  bIsProfileDefined_{true},
  frequencyGroupIndex_{},
  u64BandwidthHz_{},
  spectralMaskIndex_{DEFAULT_SPECTRAL_MASK_INDEX}{}
