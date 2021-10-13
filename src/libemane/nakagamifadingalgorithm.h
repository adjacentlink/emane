/*
 * Copyright (c) 2017,2020 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANENAKAGAMIFADINGALGORITHM_HEADER_
#define EMANENAKAGAMIFADINGALGORITHM_HEADER_

#include "fadingalgorithm.h"
#include "emane/utils/conversionutils.h"
#include <random>

namespace EMANE
{
  class NakagamiFadingAlgorithm: public FadingAlgorithm
  {
  public:
    NakagamiFadingAlgorithm(NEMId id,
                            PlatformServiceProvider * pPlatformService);

    ~NakagamiFadingAlgorithm();

    struct Parameters
    {
      double dDistance0Meters_{};
      double dDistance1Meters_{};
      double dm0_{};
      double dm1_{};
      double dm2_{};
    };

    double operator()(double dPowerdBm, double dDistanceMeters, const void * pParams) override
    {
      auto pNakagamiFadingParameters = reinterpret_cast<const Parameters *>(pParams);

      double m{};

      if(dDistanceMeters < pNakagamiFadingParameters->dDistance0Meters_)
        {
          m = pNakagamiFadingParameters->dm0_;
        }
      else if (dDistanceMeters < pNakagamiFadingParameters->dDistance1Meters_)
        {
          m = pNakagamiFadingParameters->dm1_;
        }
      else
        {
          m = pNakagamiFadingParameters->dm2_;
        }

      return distribution_(generator_,
                           Distribution::param_type{m,
                                                      Utils::DB_TO_MILLIWATT(dPowerdBm) / m});
    }

  private:
    std::mt19937 generator_;
    using  Distribution = std::gamma_distribution<>;
    Distribution distribution_;
  };
}

#endif // EMANENAKAGAMIFADINGALGORITHM_HEADER_
