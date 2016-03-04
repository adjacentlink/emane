/*
 * Copyright (c) 2013-2014,2016 - Adjacent Link LLC, Bridgewater, New
 * Jersey
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

#ifndef EMANESTATISTICNONNUMERIC_HEADER_
#define EMANESTATISTICNONNUMERIC_HEADER_

#include "emane/statistic.h"

#include <mutex>

namespace EMANE
{
  class StatisticRegistrar;

  /**
   *
   * @class  StatisticNonNumeric
   *
   * @brief A non-numeric statistic can be a std::string or an INETAddr.
   *
   */
  template<typename T>
  class StatisticNonNumeric : public Statistic
  {
  public:
    /**
     * Destroys a statistic instance
     */
    ~StatisticNonNumeric();

    StatisticNonNumeric<T> & operator=(const T & value);

    /**
     * Gets the current statistic value
     *
     * @return statistic value
     */
    T get() const;

    /**
     * Converts the current statistic value to an Any
     *
     * @return statistic value any
     *
     * @throw AnyExpception on conversion failure
     */
    Any asAny() const override;

    /**
     * Clears the statistic value
     */
    void clear() override;

  private:
    T value_;

    mutable std::mutex mutex_;

    StatisticNonNumeric();

    friend StatisticRegistrar;
  };
}

#include "emane/statisticnonnumeric.inl"

#endif // EMANESTATISTICNONNUMERIC_HEADER_
