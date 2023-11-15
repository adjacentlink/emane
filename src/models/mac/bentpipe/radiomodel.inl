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

#include "emane/utils/parameterconvert.h"

#include <type_traits>
#include <iostream>

template<typename T>
void loadCommaRangeSet(std::set<T> & contents,
                       const std::string & entry,
                       std::size_t pos)
{
  if(entry.substr(pos+1) != "na")
    {
      std::size_t comma{};
      do
        {
          comma = entry.find_first_of(',',pos+1);

          auto value = entry.substr(pos+1,comma - (pos+1));

          auto dash = value.find_first_of('-');

          if(dash == std::string::npos)
            {
              if constexpr(std::is_same_v<T,std::uint8_t>)
                {
                  contents.emplace(EMANE::Utils::ParameterConvert(value).toUINT8());
                }
              else
                {
                  contents.emplace(EMANE::Utils::ParameterConvert(value).toUINT16());
                }
            }
          else
            {
              T v1{};
              T v2{};

              if constexpr(std::is_same_v<T,std::uint8_t>)
                {
                  v1 = EMANE::Utils::ParameterConvert(value.substr(0,dash)).toUINT8();
                  v2 = EMANE::Utils::ParameterConvert(value.substr(dash+1)).toUINT8();
                }
              else
                {
                  v1 = EMANE::Utils::ParameterConvert(value.substr(0,dash)).toUINT16();
                  v2 = EMANE::Utils::ParameterConvert(value.substr(dash+1)).toUINT16();
                }

              for(auto i = std::min(v1,v2); i <= std::max(v1,v2); ++i)
                {
                  contents.emplace(i);

                  if(i == std::numeric_limits<T>::max())
                    {
                      break;
                    }
                }
            }

          pos = comma;
        }
      while(comma != std::string::npos);
    }
}

template <typename T>
void EMANE::Models::BentPipe::RadioModel::configureTransponderValue(Transponders & transponders,
                                                                    const std::string & entry,
                                                                    void (Transponder::*fn)(T))
{
  auto pos = entry.find_first_of(":");

  auto transponderIndex = Utils::ParameterConvert(entry.substr(0,pos)).toUINT16();

  auto iter = transponders.find(transponderIndex);

  if(iter == transponders.end())
    {
      iter = transponders.emplace(transponderIndex,
                                  Transponder{transponderIndex}).first;
    }

  auto * pTransponder = &iter->second;

  if constexpr(std::is_same_v<T,std::uint16_t>)
    {
      if(entry.substr(pos+1) != "na")
        {
          (pTransponder->*fn)(Utils::ParameterConvert(entry.substr(pos+1)).toUINT16());
        }
    }
  else if constexpr(std::is_same_v<T,std::uint64_t>)
    {
      if(entry.substr(pos+1) != "na")
        {
          (pTransponder->*fn)(Utils::ParameterConvert(entry.substr(pos+1)).toUINT64());
        }
    }
  else if constexpr(std::is_same_v<T,const Microseconds &>)
    {
      if(entry.substr(pos+1) != "na")
        {
          (pTransponder->*fn)(Microseconds(Utils::ParameterConvert(entry.substr(pos+1)).toUINT64()));
        }
    }
  else if constexpr(std::is_same_v<T,Transponder::ReceiveAction>)
    {
      (pTransponder->*fn)(entry.substr(pos+1) == "process" ?
                          Transponder::ReceiveAction::PROCESS :
                          Transponder::ReceiveAction::UBEND);
    }
  else if constexpr(std::is_same_v<T,const std::string &>)
    {
      (pTransponder->*fn)(entry.substr(pos+1));
    }
  else if constexpr(std::is_same_v<T,const Transponder::TransmitSlots &>)
    {
      Transponder::TransmitSlots slots{};
      loadCommaRangeSet(slots,entry,pos);
      (pTransponder->*fn)(slots);
    }
  else if constexpr(std::is_same_v<T,const Transponder::DSCPSet &>)
    {
      Transponder::DSCPSet dscps{};
      loadCommaRangeSet(dscps,entry,pos);
      (pTransponder->*fn)(dscps);
    }
}
