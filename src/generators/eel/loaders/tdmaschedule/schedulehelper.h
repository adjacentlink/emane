/*
 * Copyright (c) 2025 - Adjacent Link LLC, Bridgewater, New Jersey
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
 * * Neither the name of DRS CenGen, LLC nor the names of its
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
 *
 */

#ifndef EMANEGENERATORSEELSCHEDULEHELPER_HEADER_
#define EMANEGENERATORSEELSCHEDULEHELPER_HEADER_

#include "emane/utils/parameterconvert.h"

#include <string>
#include <list>

namespace EMANE
{
  namespace Generators
  {
    namespace EEL
    {
      template <typename T>
      std::list<T> expandRange(const std::string & sEntry)
      {
        static_assert(std::numeric_limits<T>::is_integer, "T must be an integer");

        std::list<T> expanded{};
        std::string::size_type start_pos{0};
        std::string::size_type comma{};

        do
          {
            comma = sEntry.find_first_of(',',start_pos);

            auto sValue = sEntry.substr(start_pos,comma - start_pos);

            start_pos = comma + 1;

            auto colon = sValue.find_first_of(':',0);

            if(colon != std::string::npos)
              {
                auto sFirst = sValue.substr(0,colon);
                auto sLast = sValue.substr(colon+1,std::string::npos);

                T first{};
                T last{};

                if(std::numeric_limits<T>::is_signed)
                  {
                    first =
                      Utils::ParameterConvert{sFirst}.toINT64(std::numeric_limits<T>::min(),
                                                              std::numeric_limits<T>::max());

                    last =
                      Utils::ParameterConvert{sLast}.toINT64(std::numeric_limits<T>::min(),
                                                             std::numeric_limits<T>::max());
                  }
                else
                  {
                    first =
                      Utils::ParameterConvert{sFirst}.toUINT64(std::numeric_limits<T>::min(),
                                                               std::numeric_limits<T>::max());

                    last =
                      Utils::ParameterConvert{sLast}.toUINT64(std::numeric_limits<T>::min(),
                                                              std::numeric_limits<T>::max());
                  }

                for(T i = std::min(first,last);
                    i <= std::max(first,last);
                    ++i)
                  {
                    expanded.emplace_back(i);
                  }
              }
            else
              {
                T value{};

                if(std::numeric_limits<T>::is_signed)
                  {
                    value =
                      Utils::ParameterConvert{sValue}.toINT64(std::numeric_limits<T>::min(),
                                                              std::numeric_limits<T>::max());
                  }
                else
                  {
                    value =
                      Utils::ParameterConvert{sValue}.toUINT64(std::numeric_limits<T>::min(),
                                                               std::numeric_limits<T>::max());
                  }

                expanded.emplace_back(value);
              }
          }
        while(comma != std::string::npos);

        return expanded;
      }
    }
  }
}

#endif // EMANEGENERATORSEELSCHEDULEHELPER_HEADER_
