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
EMANE::ConfigurationInfo::ConfigurationInfo(const std::string & sName,
                                            Any::Type type,
                                            const ConfigurationProperties & properties,
                                            const std::vector<Any> & values,
                                            const std::string & sUsage,
                                            const Any & minValue,
                                            const Any & maxValue,
                                            std::size_t minOccurs,
                                            std::size_t maxOccurs,
                                            const std::string & sRegexPattern):
sName_{sName},
  properties_{properties},
  sUsage_{sUsage},
  minOccurs_{minOccurs},
  maxOccurs_{maxOccurs},
  values_{values},
  minValue_{minValue},
  maxValue_{maxValue},
  bIsNumberic_{true},
  type_{type},
  sRegexPattern_{sRegexPattern}
{}

inline
EMANE::ConfigurationInfo::ConfigurationInfo(const std::string & sName,
                                            Any::Type type,
                                            const ConfigurationProperties & properties,
                                            const std::vector<Any> & values,
                                            const std::string & sUsage,
                                            std::size_t minOccurs,
                                            std::size_t maxOccurs,
                                            const std::string & sRegexPattern):
sName_{sName},
  properties_{properties},
  sUsage_{sUsage},
  minOccurs_{minOccurs},
  maxOccurs_{maxOccurs},
  values_{values},
  minValue_{0},
  maxValue_{0},
  bIsNumberic_{false},
  type_{type},
  sRegexPattern_{sRegexPattern}
{}

inline
const std::string & EMANE::ConfigurationInfo::getName() const
{
  return sName_;
}

inline
EMANE::Any::Type EMANE::ConfigurationInfo::getType() const
{
  return type_;
}

inline
bool EMANE::ConfigurationInfo::hasDefault() const
{
  return static_cast<bool>(properties_ & ConfigurationProperties::DEFAULT);
}

inline
bool EMANE::ConfigurationInfo::isRequired() const
{
  return static_cast<bool>(properties_ & ConfigurationProperties::REQUIRED);
}

inline
bool EMANE::ConfigurationInfo::isModifiable() const
{
  return static_cast<bool>(properties_ & ConfigurationProperties::MODIFIABLE);
}

inline
std::size_t EMANE::ConfigurationInfo::getMinOccurs() const
{
  return minOccurs_;
}

inline
std::size_t EMANE::ConfigurationInfo::getMaxOccurs() const
{
  return maxOccurs_;
}

inline
const std::vector<EMANE::Any> & EMANE::ConfigurationInfo::getValues() const
{
  return values_;
}

inline
const EMANE::Any & EMANE::ConfigurationInfo::getMinValue() const
{
  return minValue_;
}

inline
const EMANE::Any & EMANE::ConfigurationInfo::getMaxValue() const
{
  return maxValue_;
}

inline
const std::string & EMANE::ConfigurationInfo::getUsage() const
{
  return sUsage_;
}

inline
const std::string & EMANE::ConfigurationInfo::getRegexPattern() const
{
  return sRegexPattern_;
}

inline
bool EMANE::ConfigurationInfo::isNumberic() const
{
  return bIsNumberic_;
}

inline
void EMANE::ConfigurationInfo::setValues(const std::vector<Any> & values)
{
  values_ = values;
}

