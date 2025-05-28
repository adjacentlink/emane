/*
 * Copyright (c) 2013,2025 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANEPHYPRECOMPUTEDPROPAGATIONMODELALGORITHM_HEADER_
#define EMANEPHYPRECOMPUTEDPROPAGATIONMODELALGORITHM_HEADER_

#include "propagationmodelalgorithm.h"

#include <map>

namespace EMANE
{
  class PrecomputedPropagationModelAlgorithm : public PropagationModelAlgorithm
  {
  public:
    PrecomputedPropagationModelAlgorithm(NEMId){}

    void update(const Events::Pathlosses & pathlosses) override
    {
      for(const auto & pathloss : pathlosses)
        {
          PathlossStore::iterator iter =
            pathlossStore_.find(pathloss.getNEMId());

          if(iter == pathlossStore_.end())
            {
              iter = pathlossStore_.emplace(pathloss.getNEMId(),
                                            FrequencyPathlossStore{}).first;
            }

          // 0 Hz is the default when no frequency match is found
          iter->second[0] = pathloss.getForwardPathlossdB();
        }
    }

    void update(const Events::PathlossExs & pathlossExs) override
    {
      for(const auto & pathloss : pathlossExs)
        {
          PathlossStore::iterator iter =
            pathlossStore_.find(pathloss.getNEMId());

          if(iter == pathlossStore_.end())
            {
              iter = pathlossStore_.emplace(pathloss.getNEMId(),
                                            FrequencyPathlossStore{}).first;
            }

          for(const auto & entry : pathloss.getFrequencyPathlossMap())
            {
              iter->second[entry.first] = entry.second;
            }
        }
    }

    std::pair<std::vector<double>, bool> operator()(NEMId src,
                                                    const LocationInfo &,
                                                    const FrequencySegments & segments) override
    {
      auto iter = pathlossStore_.find(src);

      if(iter != pathlossStore_.end())
        {
          std::vector<double> pathloss(segments.size(),0);

          size_t i{};

          for(const auto & segment : segments)
            {
              if(auto fiter = iter->second.find(segment.getFrequencyHz());
                 fiter != iter->second.end())
                {
                  pathloss[i++] = fiter->second;
                }
              else
                {
                  // 0 Hz is the default when no frequency match is found
                  if(auto fiter = iter->second.find(0);
                     fiter != iter->second.end())
                    {
                      pathloss[i++] = fiter->second;
                    }
                  else
                    {
                      return {{},false};
                    }
                }
            }

          return {std::move(pathloss),true};
        }

      return {{},false};
    }

  private:
    using FrequencyPathlossStore = std::map<std::uint64_t,double>;
    using PathlossStore = std::map<NEMId,FrequencyPathlossStore>;
    PathlossStore pathlossStore_;
  };
}

#endif  // EMANEPHYPRECOMPUTEDPROPAGATIONMODELALGORITHM_HEADER_
