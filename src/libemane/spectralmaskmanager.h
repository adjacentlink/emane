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

#ifndef EMANESPECTRALMASKMANAGER_HEADER_
#define EMANESPECTRALMASKMANAGER_HEADER_

#include "emane/utils/singleton.h"
#include "emane/types.h"

#include <string>
#include <map>
#include <vector>
#include <tuple>

namespace EMANE
{
  class SpectralMaskManager : public Utils::Singleton<SpectralMaskManager>
  {
  public:
    void load(const std::string & sSpectralMaskURI);


    using SpectralSegment = std::tuple<double, // overlap ratio
                                       double, // mWr
                                       std::uint64_t, // lower seg overlap freq Hz
                                       std::uint64_t>; // upper seg overlap freq Hz


    using SpectralSegments = std::vector<SpectralSegment>;

    using SpectralOverlap = std::tuple<SpectralSegments,
                                       std::uint64_t, // lower spur overlap freq Hz
                                       std::uint64_t>; // upper spur overlap freq Hz

    using SpectralOverlaps = std::vector<SpectralOverlap>;

    using MaskOverlap = std::tuple<SpectralOverlaps,
                                   std::uint64_t, // lower mask overlap freq Hz
                                   std::uint64_t, // upper mask overlap freq Hz
                                   std::uint64_t>;


    MaskOverlap getSpectralOverlap(std::uint64_t u64TxFrequency,
                                   std::uint64_t u64RxFrequency,
                                   std::uint64_t u64RxBandwidth,
                                   std::uint64_t u64TxBandwidth,
                                   std::uint16_t u16SpectalMaskId) const;
  private:
    using MaskShape = std::vector<std::tuple<uint64_t, // width hz
                                             double>>; // mWr

    using SpectralMask =  std::tuple<std::int64_t, // offset
                                     std::uint64_t, // bandwidth
                                     MaskShape>;

    using SpectralMasks = std::vector<SpectralMask>;

    using SpectralMaskStore = std::map<SpectralMaskIndex,
                                       SpectralMasks>;

    SpectralMaskStore spectralMaskStore_;


  protected:
    SpectralMaskManager() = default;
  };
}

#endif // EMANESPECTRALMASKMANAGER_HEADER_
