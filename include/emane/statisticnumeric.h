/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANESTATISTICNUMERIC_HEADER_
#define EMANESTATISTICNUMERIC_HEADER_

#include "emane/statistic.h"

#include <mutex>

namespace EMANE
{
  class StatisticRegistrar;
  
  /**
   *
   * @class  StatisticNumeric 
   *
   * @brief Defines a numeric statistic and its operations
   *
   */
  template<typename T>
  class StatisticNumeric : public Statistic
  {
  public:
    ~StatisticNumeric();
   
    StatisticNumeric<T> & operator++();
    
    const T operator++(int);

    StatisticNumeric<T> & operator--();
    
    const T operator--(int);

    StatisticNumeric<T> & operator+=(const StatisticNumeric<T> & rhs);

    const T operator+(const StatisticNumeric<T> & rhs) const;

    const T operator-(const StatisticNumeric<T> & rhs) const;

    StatisticNumeric<T> & operator-=(const StatisticNumeric<T> & rhs);

    const T operator*(const StatisticNumeric<T> & rhs) const;

    StatisticNumeric<T> & operator*=(const StatisticNumeric<T> & rhs);

    const T operator/(const StatisticNumeric<T> & rhs) const;

    StatisticNumeric<T> & operator/=(const StatisticNumeric<T> & rhs);

    StatisticNumeric<T> & operator=(const StatisticNumeric<T> & rhs);

    StatisticNumeric<T> & operator+=(const T & rhs);
    
    T operator+(const T & rhs) const;

    T operator-(const T & rhs) const;

    StatisticNumeric<T> & operator-=(const T & rhs);

    T operator*(const T & rhs) const;

    StatisticNumeric<T> & operator*=(const T & rhs);

    T operator/(const T & rhs) const;

    StatisticNumeric<T> & operator/=(const T & rhs);

    StatisticNumeric<T> & operator=(const T & rhs);

    bool operator==(const StatisticNumeric<T> & rhs) const;

    bool operator!=(const StatisticNumeric<T> & rhs) const;

    bool operator<(const StatisticNumeric<T> & rhs) const;

    bool operator<=(const StatisticNumeric<T> & rhs) const;

    bool operator>(const StatisticNumeric<T> & rhs) const;

    bool operator>=(const StatisticNumeric<T> & rhs) const;

    bool operator==(const T & rhs) const;

    bool operator!=(const T & rhs) const;

    bool operator<(const T & rhs) const;

    bool operator<=(const T & rhs) const;

    bool operator>(const T & rhs) const;

    bool operator>=(const T & rhs) const;
    
    /**
     * Gets the underlying statistic value
     *
     * @return value
     */
    T get() const;

    Any asAny() const override;
    
    void clear() override;

    template<typename Compare = std::equal_to<T>>
      bool compareExchange(const T & expected, const T & desired);

    template<typename Compare = std::equal_to<T>>
      bool compareExchange(const T & expected,
                           const T & desired,
                           const T & alternate);

  private:
    T value_;    
    mutable std::mutex mutex_;

    StatisticNumeric();

    friend StatisticRegistrar;
  };
} 

template <typename T>
bool operator<=(const T & val, const EMANE::StatisticNumeric<T> & stat);

template <typename T>
bool operator>=(const T & val, const EMANE::StatisticNumeric<T> & stat);

template <typename T>
bool operator>(const T & val, const EMANE::StatisticNumeric<T> & stat);

template <typename T>
bool operator<(const T & val, const EMANE::StatisticNumeric<T> & stat);

template <typename T>
bool operator==(const T & val, const EMANE::StatisticNumeric<T> & stat);

template <typename T>
bool operator!=(const T & val, const EMANE::StatisticNumeric<T> & stat);

template <typename T>
T operator+(const T & val, const EMANE::StatisticNumeric<T> & stat);
    
template <typename T>
T operator-(const T & val, const EMANE::StatisticNumeric<T> & stat);

template <typename T>
T operator*(const T & val, const EMANE::StatisticNumeric<T> & stat);

template <typename T>
T operator/(const T & val, const EMANE::StatisticNumeric<T> & stat);

template <typename T>
T & operator+=(T & val, const EMANE::StatisticNumeric<T> & stat);

template <typename T>
T & operator-=(T & val, const EMANE::StatisticNumeric<T> & stat);

template <typename T>
T & operator*=(T & val, const EMANE::StatisticNumeric<T> & stat);

template <typename T>
T & operator/=(T & val, const EMANE::StatisticNumeric<T> & stat);
      
#include "emane/statisticnumeric.inl"

#endif // EMANESTATISTICNUMERIC_T_HEADER_
