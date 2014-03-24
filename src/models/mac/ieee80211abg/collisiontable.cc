/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2011-2012 - DRS CenGen, LLC, Columbia, Maryland
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


#include "collisiontable.h"

namespace {
   const int NUM_NBRS{60};

   const float v1024[NUM_NBRS] = { 0.0,   0.0,   0.4,   0.9,   0.6,   0.9,   2.0,   3.1,   3.5,   3.9,  \
                                   5.1,   5.4,   8.6,   8.1,   9.8,  10.7,  11.4,  12.9,  14.5,  17.6,  \
                                  18.4,  20.0,  23.5,  26.0,  27.2,  28.3,  29.3,  30.9,  31.2,  34.9,  \
                                  38.9,  39.2,  41.1,  42.2,  43.8,  46.2,  48.0,  49.6,  51.4,  53.8,  \
                                  56.4,  58.0,  59.9,  60.3,  63.0,  64.0,  65.5,  66.0,  70.2,  72.0,  \
                                  72.5,  75.0,  77.5,  80.0,  85.0,  87.5,  90.0,  92.5,  95.0,  100.0  };


   const float v512[NUM_NBRS] = { 0.0,    0.0,   0.6,   0.9,   1.9,   3.8,   4.2,   5.6,   7.2,   7.5,  \
                                  9.6,   12.0,  16.0,  18.6,  20.6,  22.4,  23.4,  27.5,  28.2,  30.1,  \
                                 33.9,   36.9,  40.8,  41.2,  44.6,  51.1,  49.6,  54.5,  56.4,  57.9,  \
                                 59.6,   62.7,  64.0,  68.0,  69.0,  71.3,  72.8,  74.2,  75.0,  79.6,  \
                                 82.1,   83.0,  83.4,  85.1,  86.7,  87.9,  89.1,  89.7,  91.5,  92.0,  \
                                 98.0,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0   };
    

  const float v256[NUM_NBRS] =  { 0.0,   0.2,    0.8,   3.1,   4.4,   5.1,   6.7,   9.7,  12.7,  15.4,  \
                                 20.2,  21.6,   26.3,  31.7,  33.2,  39.3,  40.8,  44.5,  47.6,  54.1,  \
                                 56.6,  59.9,   63.7,  67.5,  68.3,  74.2,  76.2,  76.8,  81.1,  83.1,  \
                                 86.3,  86.3,   86.4,  89.7,  91.4,  91.8,  93.3,  94.2,  96.2,  97.1,  \
                                 98.0,  99.3,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,  \
                                100.0, 100.0,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0   };


  const float v128[NUM_NBRS] =  { 0.0,   0.8,    2.8,   4.3,   8.2,  10.6,  16.6,  19.6,  22.9,  28.2,  \
                                 38.4,  39.8,   48.0,  51.0,  57.2,  64.3,  67.1,  71.4,  76.3,  77.8,  \
                                 84.2,  85.6,   89.2,  89.5,  90.9,  94.2,  94.5,  95.7,  96.2,  97.6,  \
                                 98.9,  99.3,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,  \
                                100.0, 100.0,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,  \
                                100.0, 100.0,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0   };


  const float v64[NUM_NBRS] =  {  0.0,   0.9,    5.5,   9.5,  15.2,  21.5,  28.5,  34.9,  44.7,  49.4,  \
                                 60.6,  69.9,   72.1,  80.3,  83.3,  87.7,  90.0,  93.1,  94.6,  95.7,  \
                                 97.5,  98.4,   98.7,  99.0,  99.3,  99.6,  99.7, 100.0, 100.0, 100.0,  \
                                100.0, 100.0,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,  \
                                100.0, 100.0,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,  \
                                100.0, 100.0,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0   };


  const float v32[NUM_NBRS] = {   0.0,   3.5,    9.5,  18.2,  27.5,  39.3,  50.6,  61.2,  71.8,  76.2,  \
                                 84.3,  88.9,   94.3,  96.6,  97.7,  99.0, 100.0, 100.0, 100.0, 100.0,  \
                                100.0, 100.0,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,  \
                                100.0, 100.0,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,  \
                                100.0, 100.0,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,  \
                                100.0, 100.0,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0   };


  const float v16[NUM_NBRS] = {   0.0,   5.8,   18.2,  32.6,  49.8,  64.8,  76.9,  86.8,  93.2,  96.1,  \
                                 99.0,  99.6,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,  \
                                100.0, 100.0,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,  \
                                100.0, 100.0,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,  \
                                100.0, 100.0,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,  \
                                100.0, 100.0,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0   };


  const float v8[NUM_NBRS] = {   0.0,   14.0,   32.0,  59.5,  78.5,  88.8,  95.8,  98.0, 100.0, 100.0,  \
                                100.0, 100.0,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,  \
                                100.0, 100.0,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,  \
                                100.0, 100.0,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,  \
                                100.0, 100.0,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,  \
                                100.0, 100.0,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0   };


  const float v4[NUM_NBRS] = {    0.0,  25.0,   62.5,  90.6, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,  \
                                100.0, 100.0,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,  \
                                100.0, 100.0,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,  \
                                100.0, 100.0,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,  \
                                100.0, 100.0,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,  \
                                100.0, 100.0,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0   };
 

  const float v2[NUM_NBRS] = {    0.0,  50.0,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
                                100.0, 100.0,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,  \
                                100.0, 100.0,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,  \
                                100.0, 100.0,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,  \
                                100.0, 100.0,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,  \
                                100.0, 100.0,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0   };


  const float v1[NUM_NBRS] = {    0.0, 100.0,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
                                100.0, 100.0,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,  \
                                100.0, 100.0,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,  \
                                100.0, 100.0,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,  \
                                100.0, 100.0,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,  \
                                100.0, 100.0,  100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0   };


  const int NUM_CW {11};

  const float *TABLE[NUM_CW] = { v1, v2, v4, v8, v16, v32, v64, v128, v256, v512, v1024 };
}


EMANE::Models::IEEE80211ABG::CollisionTable::CollisionTable()
{ }



float
EMANE::Models::IEEE80211ABG::CollisionTable::getCollisionFactor(int num, int cw)
{
  // check num nbrs
  if(num == 0)
   {
     // no nbrs return 0
     return 0.0f;
   }
  else if(num > NUM_NBRS)
   {
     // too many nbrs return 100
     return 100.0f;
   }

  
  // cap min contention window
  if(cw < 1)
   {
     cw = 1;
   }

  // cap max contention window
  if(cw > 1024)
   {
     cw = 1024;
   }

   // start at table 1
   int tbl{1};

   // check table from lowest to highest
   for(int i = 0; i < NUM_CW; ++i, tbl *= 2)
     {
       // cw is at or below this tbl
       if(cw <= tbl)
         { 
            // exact match, always true for cw 1 and 2
            if(cw == tbl)
             {
               // use this tbl
               return TABLE[i][num - 1];
             }
            // possbile for cw > 2
            else
             {
               // interpolate from previous tbl to this tbl, where i always >= 1
               return interpolate(tbl / 2, tbl, TABLE[i - 1][num - 1], TABLE[i][num - 1], cw);
             }
         }
     }

  // default
  return 0.0;
}



float
EMANE::Models::IEEE80211ABG::CollisionTable::interpolate(float x0, float x1, float y0, float y1, float x)
{
   const float deltaX{x1 - x0};

   const float deltaY{y1 - y0};

   const float m{deltaX == 0.0f ? 0.0f : deltaY / deltaX};

   return y0 + (x - x0) * m;
}
