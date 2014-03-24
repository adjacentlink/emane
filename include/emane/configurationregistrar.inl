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

#include <algorithm>

inline
EMANE::ConfigurationRegistrar::ConfigurationRegistrar(){};

inline
EMANE::ConfigurationRegistrar::~ConfigurationRegistrar(){};

template<typename T>
void EMANE::ConfigurationRegistrar::registerNumeric(const std::string & sName,
                                                    const ConfigurationProperties & properties,
                                                    const std::initializer_list<T> & values,
                                                    const std::string & sUsage,
                                                    T minValue,
                                                    T maxValue,
                                                    std::size_t minOccurs,
                                                    std::size_t maxOccurs,
                                                    const std::string & sRegexPattern)
{
  static_assert(std::is_integral<T>() || std::is_floating_point<T>(),"Type not convertable to numeric Any");
  std::vector<Any> anys;
  std::for_each(values.begin(),values.end(),[&anys](const T & v){anys.push_back(Any(v));});
  registerNumericAny(sName,
                     AnyConvertableType<T>::type(),
                     properties,
                     anys,
                     sUsage,
                     Any{minValue},
                     Any{maxValue},
                     minOccurs,
                     maxOccurs,
                     sRegexPattern);
}

template<typename T>
void EMANE::ConfigurationRegistrar::registerNonNumeric(const std::string & sName,
                                                       const ConfigurationProperties & properties,
                                                       const std::initializer_list<T> & values,
                                                       const std::string & sUsage,
                                                       std::size_t minOccurs,
                                                       std::size_t maxOccurs,
                                                       const std::string & sRegexPattern)
{
  static_assert(is_any_convertable_nonnumeric<T>(),"Type not convertable to non-numeric Any");
  std::vector<Any> anys;
  std::for_each(values.begin(),values.end(),[&anys](const T & v){anys.push_back(Any(v));});
  registerNonNumericAny(sName,
                        AnyConvertableType<T>::type(),
                        properties,
                        anys,
                        sUsage,
                        minOccurs,
                        maxOccurs,
                        sRegexPattern);
}
