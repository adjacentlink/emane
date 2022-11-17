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

#ifndef EMANEFREQUENCYOVERLAPRATIO_HEADER_
#define EMANEFREQUENCYOVERLAPRATIO_HEADER_

#include <tuple>
#include <cstdint>

namespace EMANE
{
  inline
  std::tuple<double, // overlap ratio
             std::uint64_t, // lower overlap frequency hz
             std::uint64_t> // upper overlap frequency hz
  frequencyOverlapRatio(std::uint64_t u64FrequencyHz1,
                        std::uint64_t u64BandwidthHz1,
                        std::uint64_t u64FrequencyHz2,
                        std::uint64_t u64BandwidthHz2)
  {
    std::uint64_t u64UpperFrequencyHz1{u64FrequencyHz1 + u64BandwidthHz1 / 2};
    std::uint64_t u64LowerFrequencyHz1{u64FrequencyHz1 - u64BandwidthHz1 / 2};

    std::uint64_t u64UpperFrequencyHz2{u64FrequencyHz2 + u64BandwidthHz2 / 2};
    std::uint64_t u64LowerFrequencyHz2{u64FrequencyHz2 - u64BandwidthHz2 / 2};

    std::uint64_t u64LowerOverlapFrequencyHz{};
    std::uint64_t u64UpperOverlapFrequencyHz{};

    // percent in band, defaults to no coverage
    double dRatio{};

    // signal is somewhere in band
    if((u64LowerFrequencyHz2 < u64UpperFrequencyHz1) && (u64UpperFrequencyHz2 > u64LowerFrequencyHz1))
      {
        // low is within lower bound
        if(u64LowerFrequencyHz2 >= u64LowerFrequencyHz1)
          {
            u64LowerOverlapFrequencyHz = u64LowerFrequencyHz2;

            // high is within upper bound
            if(u64UpperFrequencyHz2 <= u64UpperFrequencyHz1)
              {
                u64UpperOverlapFrequencyHz = u64UpperFrequencyHz2;

                // full coverage
                dRatio = 1.0;
              }
            // exceeded upper bound
            else
              {
                u64UpperOverlapFrequencyHz = u64UpperFrequencyHz1;

                // partial coverage
                dRatio =
                  (u64UpperFrequencyHz1 - u64LowerFrequencyHz2) / static_cast<double>(u64BandwidthHz2);
              }
          }
        // low is below lower bound
        else
          {
            u64LowerOverlapFrequencyHz = u64LowerFrequencyHz1;

            // the signal is at or beyond
            if(u64UpperFrequencyHz2 <= u64UpperFrequencyHz1)
              {
                u64UpperOverlapFrequencyHz = u64UpperFrequencyHz2;

                // partial coverage
                dRatio =
                  (u64UpperFrequencyHz2 - u64LowerFrequencyHz1) / static_cast<double>(u64BandwidthHz2);
              }
            else
              {
                u64UpperOverlapFrequencyHz = u64UpperFrequencyHz1;

                dRatio =
                  (u64UpperFrequencyHz1 - u64LowerFrequencyHz1) / static_cast<double>(u64BandwidthHz2);
              }
          }
      }

    // return ratio
    return std::make_tuple(dRatio,u64LowerOverlapFrequencyHz,u64UpperOverlapFrequencyHz);
  }
}

#endif //EMANEFREQUENCYOVERLAPRATIO_HEADER_
