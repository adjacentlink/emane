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

#include "bitpool.h"

#include <cmath>
#include <condition_variable>

namespace
{
  const EMANE::Microseconds FillIntervalMicroseconds{1000000};
}


inline
EMANE::Utils::BitPool::BitPool(PlatformServiceProvider * pPlatformService, NEMId id) :
 pPlatformService_{pPlatformService},
 id_(id),
 u64MaxSize_{},
 u64CurrentSize_{},
 lastRequestTime_{},
 fFillRemainder_{}
{ }


inline
EMANE::Utils::BitPool::~BitPool()
{ }



inline
 void
EMANE::Utils::BitPool::setMaxSize(std::uint64_t u64NewSize)
{
  // lock mutex
  std::lock_guard<std::mutex> m(mutex_);

  // size changed
  if(u64NewSize != u64MaxSize_)
    {
      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                             DEBUG_LEVEL,
                             "NEM %03hu BitPool::%s current pool size %ju, new pool size %ju",
                             id_,
                             __func__,
                             u64MaxSize_,
                             u64NewSize);

      // new size is less than current size
      if(u64CurrentSize_ > u64NewSize)
        {
          // reduce current size
          u64CurrentSize_ = u64NewSize;
        }

      // set new max size
      u64MaxSize_ = u64NewSize;
    }
}



inline
std::uint64_t
EMANE::Utils::BitPool::get(std::uint64_t u64Request, bool bFullFill)
{
  // lock mutex
  std::lock_guard<std::mutex> m(mutex_);

  // disabled
  if(u64MaxSize_ == 0 || u64Request == 0)
    {
      // nothing outstanding
      return 0;
    }

  // total acquired
  std::uint64_t u64Acquired{};

  // try to fulfill request
  while(1)
    {
      TimePoint currentTime = Clock::now();

      // time to wait interval
      Microseconds waitIntervalMicroseconds;

      // make request
      u64Acquired += doDrainPool(u64Request - u64Acquired, currentTime, waitIntervalMicroseconds);

      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                             DEBUG_LEVEL,
                             "NEM %03hu BitPool::%s request %ju, acquired %ju, pool remaining %ju,  %4.2f%%",
                             id_,
                             __func__,
                             u64Request,
                             u64Acquired,
                             u64CurrentSize_,
                             100.0f * (1.0f - (static_cast<float>(u64MaxSize_ - u64CurrentSize_) /
                                               static_cast<float>(u64MaxSize_))));

      // request fulfilled
      if(!bFullFill || u64Acquired == u64Request)
        {
          // done
          break;
       }

     // local mutex
     std::mutex mutex;

     // condition variable
     std::condition_variable cond{};

     std::unique_lock<std::mutex> lock(mutex);

     // wait here
     if(cond.wait_until(lock,currentTime + waitIntervalMicroseconds) != std::cv_status::timeout) {

       // did not time out, spurious wake up
       break;
     }
  }

  // return the result
  return u64Request - u64Acquired;
}


inline
std::uint64_t
EMANE::Utils::BitPool::getCurrentSize()
{
  // lock mutex
  std::lock_guard<std::mutex> m(mutex_);

  return u64CurrentSize_;
}


inline
std::uint64_t
EMANE::Utils::BitPool::doDrainPool(std::uint64_t u64Request,
                                   const TimePoint & requestTime,
                                   Microseconds & intervalMicroseconds)
{
  std::uint64_t u64Acquired{};

  // bring the pool up to date since last request
  doFillPool(requestTime);

  // drain the pool
  if(u64Request > u64CurrentSize_)
    {
      // size outstanding
      std::uint64_t u64Outstanding{u64Request - u64CurrentSize_};

      // result is the available pool
      u64Acquired = u64CurrentSize_;

      // empty the pool
      u64CurrentSize_ = 0;

      // fill ratio assumes 1 sec interval
      float fSeconds{static_cast<float>(u64Outstanding) / static_cast<float>(u64MaxSize_)};

      // set wait interval
      intervalMicroseconds = std::chrono::duration_cast<Microseconds>(DoubleSeconds{fSeconds});

      // cap wait interval to fill interval
      if(intervalMicroseconds > FillIntervalMicroseconds)
        {
          intervalMicroseconds = FillIntervalMicroseconds;
        }
    }
  else
    {
      // result is the request
      u64Acquired = u64Request;

      // drain the pool by the request size
      u64CurrentSize_ -= u64Request;

      // no wait time
      intervalMicroseconds = Microseconds::zero();
    }

  // update last request time
  lastRequestTime_ = requestTime;

  // return result
  return u64Acquired;
}


inline
void
EMANE::Utils::BitPool::doFillPool(const TimePoint & requestTime)
{
  // fill the pool since last request
  if(lastRequestTime_ != TimePoint{})
    {
      // time since last request
      Microseconds deltaTMicroseconds{std::chrono::duration_cast<Microseconds>(requestTime - lastRequestTime_)};

     // fill the pool depending on elapsed time
     if(deltaTMicroseconds >= FillIntervalMicroseconds)
       {
         // fill to full capacity
         u64CurrentSize_ = u64MaxSize_;

         fFillRemainder_ = 0;
       }
     else
       {
         // find the amount of time into the fill interval
         Microseconds diffMicroseconds{FillIntervalMicroseconds - (FillIntervalMicroseconds - deltaTMicroseconds)};

         // accumulate fill amount for this interval
         fFillRemainder_ += (static_cast<float>(diffMicroseconds.count()) /
                             static_cast<float>(FillIntervalMicroseconds.count())) * u64MaxSize_;

         // have at least a whole unit
         if(fFillRemainder_ >= 1.0f)
           {
             std::uint64_t u64Fill{static_cast<std::uint64_t>(fFillRemainder_)};

             // fill pool
             if((u64CurrentSize_ + u64Fill) <= u64MaxSize_)
               {
                 u64CurrentSize_ = u64CurrentSize_ + u64Fill;
               }
             else
               {
                 u64CurrentSize_ = u64MaxSize_;
               }

             // keep fraction remainder
             fFillRemainder_ -= u64Fill;
           }
       }
    }
}
