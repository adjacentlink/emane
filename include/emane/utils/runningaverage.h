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

#include "emane/statisticnumeric.h"

#ifndef EMANEUTILSRUNNINGAVERAGE_HEADER_
#define EMANEUTILSRUNNINGAVERAGE_HEADER_

namespace EMANE
{
  namespace Utils
  {
    template<class T>
    class RunningAverage
    {
    public:
      RunningAverage() :
        pStatistic_{},
        value_{},
        count_{}
      { }

      void update(T value)
      {
        T newValue{value};

        if(count_)
          {
            newValue = ((value_ * count_) + value) / (count_ + 1);
          }
        
        // if pStatistic_ is still set to value_ set it to
        //  newValue otherwise it was cleared, set it to value
        if(pStatistic_->compareExchange(value_,newValue,value))
          {
            value_ = newValue;
            ++count_;
          }
        else
          {
            value_= value;
            count_ = 1;
          }
      }

      void registerStatistic(StatisticNumeric<T> * p)
      {
        pStatistic_ = p;
      }

    private:
      StatisticNumeric<T> * pStatistic_;
      T value_;
      size_t count_;
    };
  }
}

#endif // EMANEUTILSRUNNINGAVERAGE_HEADER_
