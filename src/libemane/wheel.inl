/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
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

template<typename T>
EMANE::Wheel<T>::Wheel(std::size_t size):
  size_{size},
  store_(size,0){}

template<typename T>
const std::vector<T> & EMANE::Wheel<T>::dump() const
{
  return store_;
}

template<typename T>
void EMANE::Wheel<T>::add(std::size_t begin,std::size_t slots,T value)
{
  if(slots > size_ || begin >= size_)
    {
      throw IndexError{};
    }
  
  if(!slots)
    return;
  
  std::size_t remainder{};
  
  if(begin + slots > size_)
    {
      remainder = (begin + slots) % size_;
    }

  for(std::size_t i = begin; i < begin + slots - remainder; ++i)
    {
      store_[i] += value;
    }

  if(remainder)
    {
      for(std::size_t i = 0; i < remainder; ++i)
        {
          store_[i] += value;
        }
    }
}

template<typename T>
void EMANE::Wheel<T>::set(std::size_t begin,std::size_t slots,T value)
{
  if(slots > size_ || begin >= size_)
    {
      throw IndexError{};
    }
  
  if(!slots)
    return;
  
  std::size_t remainder{};
  
  if(begin + slots > size_)
    {
      remainder = (begin + slots) % size_;
    }


  for(std::size_t i = begin; i < begin + slots - remainder; ++i)
    {
      store_[i] = value;
    }

  if(remainder)
    {
      for(std::size_t i = 0; i < remainder; ++i)
        {
          store_[i] = value;
        }
    }
}


template<typename T>
std::vector<T> EMANE::Wheel<T>::get(std::size_t begin,std::size_t slots)
{
  if(slots > size_ || begin >= size_)
    {
      throw IndexError{};
    }

  std::vector<T> values(slots);

  if(begin >= slots - 1)
    {
      std::memcpy(&values[0],&store_[begin - slots + 1],slots * sizeof(T));
    }
  else
    {
      std::size_t remainder = slots - begin -1;

      std::memcpy(&values[0],&store_[size_ - remainder],remainder * sizeof(T));

      std::memcpy(&values[remainder],&store_[0],(begin + 1) * sizeof(T));
    }

  return values;
}


template<typename T>
void EMANE::Wheel<T>::clear(std::size_t begin,std::size_t slots)
{
  if(slots > size_ || begin >= size_)
    {
      throw IndexError{};
    }

  if(begin >= slots - 1)
    {
      std::memset(&store_[begin - slots + 1],0,slots * sizeof(T));
    }
  else
    {
      std::size_t remainder = slots - begin -1;

      std::memset(&store_[size_ - remainder],0,remainder * sizeof(T));

      std::memset(&store_[0],0,(begin + 1) * sizeof(T));
    }
}
