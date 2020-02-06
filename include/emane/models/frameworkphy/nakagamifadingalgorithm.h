/*
 * Copyright (c) 2017 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "emane/models/frameworkphy/fadingalgorithm.h"
#include "emane/platformserviceprovider.h"
#include "emane/utils/conversionutils.h"
#include <random>

namespace EMANE
{
  class NakagamiFadingAlgorithm: public FadingAlgorithm
  {
  public:
    NakagamiFadingAlgorithm(NEMId id,
                            PlatformServiceProvider * pPlatformService,
                            const std::string & sPrefix);

    void initialize(Registrar & registrar) override;

    void configure(const ConfigurationUpdate & update);

    void modify(const ConfigurationUpdate & update) override;

    double operator()(double dPowerdBm, double dDistanceMeters) override
    {
      double m{};

      if(dDistanceMeters < dDistance0Meters_)
        {
          m = dm0_;
        }
      else if (dDistanceMeters < dDistance1Meters_)
        {
          m = dm1_;
        }
      else
        {
          m = dm2_;
        }

      return Utils::MILLIWATT_TO_DB(distribution_(generator_,
                                                  Distribution::param_type{m,
                                                      Utils::DB_TO_MILLIWATT(dPowerdBm) / m}));
    }

  private:
    double dm0_;
    double dm1_;
    double dm2_;
    double dDistance0Meters_;
    double dDistance1Meters_;
    std::mt19937 generator_;
    using  Distribution = std::gamma_distribution<>;
    Distribution distribution_;

    void configure_i(const ConfigurationUpdate & update);
  };
}

#endif // EMANENAKAGAMIFADINGALGORITHM_HEADER_
