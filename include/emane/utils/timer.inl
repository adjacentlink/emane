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

template <typename Clock, typename Duration, typename Function>
EMANE::Utils::Timer::TimerId EMANE::Utils::Timer::schedule(Function fn,
                                                           const std::chrono::time_point<Clock,Duration> & absoluteTimePoint)
{
  return schedule(fn,
                  std::chrono::time_point_cast<std::chrono::microseconds>(absoluteTimePoint),
                  std::chrono::microseconds::zero());
}

template <typename Clock, typename Duration, typename Rep, typename Period, typename Function>
EMANE::Utils::Timer::TimerId EMANE::Utils::Timer::scheduleInterval(Function fn,
                                                                   const std::chrono::time_point<Clock,Duration> & absoluteTimePoint,
                                                                   const std::chrono::duration<Rep,Period> & interval)
{
  return schedule(fn,
                  std::chrono::time_point_cast<std::chrono::microseconds>(absoluteTimePoint),
                  std::chrono::duration_cast<std::chrono::microseconds>(interval));
}
