/*
 * Copyright (c) 2023 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "configurehelpers.h"
#include "pcrmanager.h"
#include "emane/configureexception.h"


void EMANE::Models::BentPipe::configureAntennaValue(Antennas & antennas,
                                                    const std::string & sEntry)
{
  auto pos = sEntry.find_first_of(':');

  AntennaIndex antennaIndex =
    Utils::ParameterConvert(sEntry.substr(0,pos)).toUINT16();

  if(antennas.count(antennaIndex))
    {
      throw makeException<ConfigureException>("BentPipe::RadioModel:"
                                              " antenna.defines contains duplicate"
                                              " antenna index: %hu",
                                              antennaIndex);
    }

  auto semi = sEntry.find_first_of(';',pos+1);

  auto value = sEntry.substr(pos+1,semi - (pos+1));

  pos = semi;

  Antenna antenna{};

  if(value == "omni")
    {
      semi = sEntry.find_first_of(';',pos+1);

      double dFixedGaindBi =
        EMANE::Utils::ParameterConvert(sEntry.substr(pos+1,semi - (pos+1))).toDouble();

      antenna = Antenna::createIdealOmni(antennaIndex,dFixedGaindBi);
    }
  else
    {
      AntennaProfileId profileId =
        EMANE::Utils::ParameterConvert(value).toUINT16();

      semi = sEntry.find_first_of(';',pos+1);

      double dAzimuthDegrees =
        EMANE::Utils::ParameterConvert(sEntry.substr(pos+1,semi - (pos+1))).toDouble();

      pos = semi;

      semi = sEntry.find_first_of(';',pos+1);

      double dElevationDegrees =
        EMANE::Utils::ParameterConvert(sEntry.substr(pos+1,semi - (pos+1))).toDouble();

      pos = semi;

      antenna = Antenna::createProfileDefined(antennaIndex,{profileId,
                                                            dAzimuthDegrees,
                                                            dElevationDegrees});

    }

  SpectralMaskIndex maskIndex =
    EMANE::Utils::ParameterConvert(sEntry.substr(semi+1)).toUINT16();

  antenna.setSpectralMaskIndex(maskIndex);

  antennas.emplace(antennaIndex,antenna);
}
