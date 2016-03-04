/*
 * Copyright (c) 2013-2014,2016 - Adjacent Link LLC, Bridgewater, New
 * Jersey
 * Copyright (c) 2009-2010 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef BITPOOL_HEADER_
#define BITPOOL_HEADER_

#include "emane/types.h"
#include "emane/platformserviceprovider.h"

namespace EMANE
{
  namespace Utils
  {
    /**
     * @class BitPool
     *
     * @brief Implementation of a rate limiting bit pool
     *
     * @note Uses a thread to allow blocking dequeues
     */
    class BitPool
    {
    public:
      /**
       * Creates a BitPool instance
       */
      BitPool(PlatformServiceProvider * pPlatformService, NEMId id);

      /**
       * Destroys an instance
       */
      ~BitPool();

      /**
       * Gets bits from pool
       *
       * @param u64Request request size
       * @param bFullFill Continue until entire request is fulfilled
       *
       * @return Number outstanding or 0 if disabled
       */
      std::uint64_t get(std::uint64_t u64Request, bool bFullFill = true);

      /**
       * Gets current pool size
       *
       * @return the current pool size
       *
       */
      std::uint64_t getCurrentSize();

      /**
       * Set pool size
       *
       * @param u64NewSize pool size request in bits
       *
       */
      void setMaxSize(std::uint64_t u64NewSize);

    private:
      EMANE::PlatformServiceProvider * pPlatformService_;

      NEMId id_;

      std::uint64_t         u64MaxSize_;

      std::uint64_t         u64CurrentSize_;

      TimePoint             lastRequestTime_;

      float                 fFillRemainder_;

      std::mutex            mutex_;

      /**
       * Adds to pool
       *
       * @param requestTime   request time
       */
      void doFillPool(const TimePoint & requestTime);

      /**
       * Drains the pool
       *
       * @param u64Request            request size
       * @param requestTime           request time (abs time)
       * @param intervalMicroseconds  time to wait until request would be fulfilled
       *
       * @return  number available in the pool
       *
       */
      std::uint64_t doDrainPool(std::uint64_t u64Request,
                                const TimePoint & requestTime,
                                Microseconds & intervalMicroseconds);
    };
  }
}

#include "emane/utils/bitpool.inl"

#endif // BITPOOL_HEADER_
