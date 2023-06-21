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
 * * Neither the name of Adjacent Link, LLC nor the names of its
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

#ifndef EMANE_UTILS_WEIGHTEDMOVINGAVERAGE_HEADER_
#define EMANE_UTILS_WEIGHTEDMOVINGAVERAGE_HEADER_

#include <vector>

namespace EMANE
{
  namespace Utils
  {
    class WeightedMovingAverage
    {
    public:
      WeightedMovingAverage(size_t size):
        samples_(size,{}),
        size_{size},
        oldestIndex_{},
        totalSamples_{}{}

      void update(double dValue)
      {
        ++totalSamples_;

        samples_[oldestIndex_] = dValue;

        ++oldestIndex_;

        oldestIndex_ %= size_;
      }

      using MeanInfo = std::tuple<double,double,size_t>;


      double value()
      {
        double dTotal{};

        auto count =
          std::min(totalSamples_,samples_.size());

        for(size_t i = 0; i < count; ++i)
          {
            dTotal += samples_[(oldestIndex_ + i) %  count] * (i + 1);
          }

        return dTotal / (((1 + count) *  count) / 2);
      }

    private:
      using Samples = std::vector<double>;
      Samples samples_;
      const size_t size_;
      size_t oldestIndex_;
      size_t totalSamples_;
    };
  }
}

#endif // EMANE_UTILS_WEIGHTEDMOVINGAVERAGE_HEADER_
