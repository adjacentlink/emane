/*
 * Copyright (c) 2013,2016 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2010-2012 - DRS CenGen, LLC, Columbia, Maryland
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
 */


#ifndef EMANEMODELSIEEE802ABGUTILS_HEADER_
#define EMANEMODELSIEEE802ABGUTILS_HEADER_


#include <cmath>

namespace EMANE
 {
  namespace Models
   {
     namespace IEEE80211ABG
       {
         inline std::uint64_t FACTORIAL(int x)
         {
           std::uint64_t val {1};

           while(x > 1)
            {
              val *= x--;
            }

           return val;
         }


        inline Microseconds TimePointToMicroseconds(const TimePoint & tp)
         {
           return Microseconds{std::chrono::duration_cast<Microseconds>(tp.time_since_epoch())};
         }

        inline float getRatio(const EMANE::Microseconds & d1, const EMANE::Microseconds d2)
         {
           return std::chrono::duration_cast<DoubleSeconds>(d1).count() /
                  std::chrono::duration_cast<DoubleSeconds>(d2).count();
         }
      }
   }
}

#endif //EMANEMODELSIEEE802ABGUTILS_HEADER_
