/*
 * Copyright (c) 2010 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANE_CONSTANTS_HEADER_
#define EMANE_CONSTANTS_HEADER_

// Need for Win32 and OS X
#ifndef M_PIl
#define M_PIl 3.1415926535897932384626433832795029L
#endif

namespace EMANE
{
  // min/max dBm power levels
  const float DBM_MIN = {-327.0f};
  const float DBM_MAX = {327.0f};

  const double THERMAL_NOISE_DB{-174.0};
  

  // arc seconds per degree of angle
  const double ARC_SECONDS_PER_DEGREE{3600.0};

  // milli arc seconds per degree of angle (3600 arc seconds per deg)
  const double MILLI_ARC_SECONDS_PER_DEGREE{3600.0e3};

  const double SEMI_MAJOR{6378137.0};
  const double SEMI_MINOR{6356752.3142};
  const double SEMI_MAJOR_2{SEMI_MAJOR * SEMI_MAJOR};
  const double SEMI_MINOR_2{SEMI_MINOR * SEMI_MINOR};
  const double ECC2{(SEMI_MAJOR_2 - SEMI_MINOR_2) / SEMI_MAJOR_2}; 

  // speed of light in meters per second
  const double SOL_MPS{299792458.0};

  // usec per second as a float
  const float USEC_PER_SEC_F{1.0e6};
}


#endif // EMANE_CONSTANTS_HEADER_
