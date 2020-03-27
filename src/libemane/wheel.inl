/*
 * Copyright (c) 2013,2020 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include <cstring>

template<typename T>
EMANE::Wheel<T>::Wheel(std::size_t slots,
                       std::size_t bins):
  slots_{slots},
  bins_{bins},
  store_(slots * bins,0)
{}

template<typename T>
std::size_t EMANE::Wheel<T>:: slots() const
{
  return slots_;
}

template<typename T>
std::size_t EMANE::Wheel<T>::bins() const
{
  return bins_;
}

template<typename T>
const std::vector<T> & EMANE::Wheel<T>::dump() const
{
  return store_;
}

template<typename T>
void EMANE::Wheel<T>::add(std::size_t begin,
                          std::size_t slots,
                          T value,
                          std::size_t binBegin,
                          std::size_t bins)
{
  if(slots > slots_ || begin >= slots_)
    {
      throw makeException<IndexError>("wheel total slots available: %zu"
                                      " attempting to set bins: %zu"
                                      " starting at: %zu",
                                      slots_,
                                      slots,
                                      begin);
    }

  if(!slots)
    return;

  if(bins > bins_ || binBegin >= bins_)
    {
      throw makeException<IndexError>("wheel total bins available: %zu"
                                      " attempting to set bins: %zu"
                                      " starting at: %zu",
                                      bins_,
                                      bins,
                                      binBegin);
    }

  if(!bins)
    return;

  std::size_t remainder{};

  if(begin + slots > slots_)
    {
      remainder = (begin + slots) % slots_;
    }

  for(std::size_t i = begin; i < begin + slots - remainder; ++i)
    {
      for(std::size_t j = binBegin; j < binBegin + bins; ++j)
        {
          store_[i*bins_ + j] += value;
        }
    }

  if(remainder)
    {
      for(std::size_t i = 0; i < remainder; ++i)
        {
          for(std::size_t j = binBegin; j < binBegin + bins; ++j)
            {
              store_[i*bins_ + j] += value;
            }
        }
    }
}

template<typename T>
void EMANE::Wheel<T>::set(std::size_t begin,
                          std::size_t slots,
                          T value,
                          std::size_t binBegin,
                          std::size_t bins)
{
  if(slots > slots_ || begin >= slots_)
    {
      throw makeException<IndexError>("wheel total slots available: %zu"
                                      " attempting to set bins: %zu"
                                      " starting at: %zu",
                                      slots_,
                                      slots,
                                      begin);
    }

  if(!slots)
    return;

  if(bins > bins_ || binBegin >= bins_)
    {
      throw makeException<IndexError>("wheel total bins available: %zu"
                                      " attempting to set bins: %zu"
                                      " starting at: %zu",
                                      bins_,
                                      bins,
                                      binBegin);
    }

  if(!bins)
    return;

  std::size_t remainder{};

  if(begin + slots > slots_)
    {
      remainder = (begin + slots) % slots_;
    }

  // special case clear
  if(value == 0 && binBegin == 0 && bins == bins_)
    {
      std::memset(&store_[begin * bins_],
                  0,
                  (slots - remainder) * sizeof(T) * bins_);

      if(remainder)
        {
          std::memset(&store_[0],
                      0,
                      remainder * sizeof(T) * bins_);
        }
    }
  else
    {
      for(std::size_t i = begin; i < begin + slots - remainder; ++i)
        {
          for(std::size_t j = binBegin; j < binBegin + bins; ++j)
            {
              store_[i*bins_ + j] = value;
            }
        }

      if(remainder)
        {
          for(std::size_t i = 0; i < remainder; ++i)
            {
              for(std::size_t j = binBegin; j < binBegin + bins; ++j)
                {
                  store_[i*bins_ + j] = value;
                }
            }
        }
    }
}

template<typename T>
std::vector<T> EMANE::Wheel<T>::get(std::size_t begin,std::size_t slots)
{
  if(slots > slots_ || begin >= slots_)
    {
      throw makeException<IndexError>("wheel total slots available: %zu"
                                      " attempting to set bins: %zu"
                                      " starting at: %zu",
                                      slots_,
                                      slots,
                                      begin);
    }

  std::vector<T> values(slots * bins_);

  if(begin >= slots - 1)
    {
      std::memcpy(&values[0],&store_[(begin - slots + 1) * bins_],slots * sizeof(T) * bins_);
    }
  else
    {
      std::size_t remainder = slots - begin -1;

      std::memcpy(&values[0],&store_[(slots_ - remainder) * bins_],remainder * sizeof(T) * bins_);

      std::memcpy(&values[remainder * bins_],&store_[0],(begin + 1) * sizeof(T) * bins_);
    }

  return values;
}
