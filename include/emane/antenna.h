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

#ifndef EMANEANTENNA_HEADER_
#define EMANEANTENNA_HEADER_

#include <emane/types.h>
#include <vector>

namespace EMANE
{
  class Antenna
  {
  public:
    class Pointing
    {
    public:
      Pointing();

      Pointing(AntennaProfileId antennaProfileId,
               double dAzimuthDegrees,
               double dElevationDegrees);

      AntennaProfileId getProfileId() const;

      double getAzimuthDegrees() const;

      double getElevationDegrees() const;

      bool operator==(const Pointing & rhs) const;

      bool isValid() const;

    private:
      AntennaProfileId antennaProfileId_;
      double dAzimuthDegrees_;
      double dElevationDegrees_;
      bool bIsValid_;
    };

    static Antenna createDefault();

    static Antenna createIdealOmni(AntennaIndex antennaIndex,
                                   double dFixedGaindBi);

    static Antenna createProfileDefined(AntennaIndex antennaIndex,
                                        const Pointing & pointing = {});

    Antenna();

    bool isIdealOmni() const;

    bool isProfileDefined() const;

    bool isDefault() const;

    std::pair<double,bool>
    getFixedGaindBi() const;

    std::pair<const Pointing &,bool>
    getPointing() const;

    AntennaIndex getIndex() const;

    FrequencyGroupIndex getFrequencyGroupIndex() const;

    void setPointing(const Pointing & pointing);

    void setFrequencyGroupIndex(FrequencyGroupIndex index);

    std::uint64_t getBandwidthHz() const;

    void setBandwidthHz( std::uint64_t u64BandwidthHz);

    void setFixedGaindBi(double dFixedGaindBi);

    SpectralMaskIndex getSpectralMaskIndex() const;

    void setSpectralMaskIndex(SpectralMaskIndex spectralMaskIndex);

    bool operator==(const Antenna & rhs) const;

    bool operator!=(const Antenna & rhs) const;

  private:
    AntennaIndex antennaIndex_;
    double dFixedGaindBi_;
    Pointing pointing_;
    bool bIsIdealOmni_;
    bool bIsProfileDefined_;
    FrequencyGroupIndex frequencyGroupIndex_;
    std::uint64_t u64BandwidthHz_;
    SpectralMaskIndex spectralMaskIndex_;

    // ideal omni
    Antenna(AntennaIndex antennaIndex,
            double dFixedGaindBi);

    Antenna(AntennaIndex antennaIndex,
            const Pointing & pointing = {});
  };

  using Antennas = std::vector<Antenna>;
}

#include "emane/antenna.inl"

#endif // EMANEANTENNA_HEADER
