/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2012 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANEUTILSCONVERSIONUTILS_HEADER_
#define EMANEUTILSCONVERSIONUTILS_HEADER_

#include <cmath>
#include "emane/constants.h"

namespace EMANE
{
  namespace Utils
  {
    inline double NORMALIZE_VECTOR(const double x, const double y, const double z)
    {
      return sqrt((x * x) + (y * y) + (z * z));
    }


    inline double DEGREES_TO_RADIANS(const double deg)
    {
      return deg * (M_PIl / 180.0);
    }

 
    inline void AI_TO_BE_DEGREES(double & val, const double A, const double B)
    {
      // less than A
      if (val < A)
        {
          while (val < A)
            {
              val += 360.0;
            }
        }
      // greater or equal to B
      else if(val >= B)
        {
          while(val >= B)
            {
              val -= 360.0;
            }
        }

    }

    inline void AI_TO_BI_DEGREES(double & val, const double A, const double B)
    {
      // less than A
      if (val < A)
        {
          while (val < A)
            {
              val += 360.0;
            }
        }
      // greater than B
      else if(val > B)
        {
          while(val > B)
            {
              val -= 360.0;
            }
        }
    }


    inline void NEG_90_TO_POS_90_DEGREES(double & val)
    {
      // ensure (-90 <= val <= 90)
      while(val > 90.0)
        {
          if(val < 270.0)
            {
              val -= 180.0;
            }
          else
            {
              val -= 360.0;
            }
        }
    }

    inline double MILLIWATT_TO_DB(double dMillWatt)
    {
      return 10.0 * log10(dMillWatt);
    }
    
    inline double DB_TO_MILLIWATT(double ddB)
    {
      return pow(10.0,ddB / 10.0);
    }
  }
}

#endif // EMANEUTILSCONVERSIONUTILS_HEADER_
