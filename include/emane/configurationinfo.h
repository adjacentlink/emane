/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANECONFIGURATIONINFO_HEADER_
#define EMANECONFIGURATIONINFO_HEADER_

#include "emane/any.h"
#include "emane/configurationproperties.h"

#include <string>
#include <vector>

namespace EMANE
{
  /**
   * @class ConfigurationInfo
   *
   * @brief Holds configuration item meta information
   */
  class ConfigurationInfo
  {
  public:
    /**
     * Creates a ConfiguratonInfo instance for numeric
     * configuration items (ones with min and max values)
     *
     * @param sName Name of the configuration item
     * @param properties Configuration properties mask
     * @param values Default configuration values
     * @param sUsage Parameter usage description
     * @param minValue Minimum acceptable value
     * @param maxValue Maximum acceptable value
     * @param minOccurs Minimum values allowed
     * @param maxOccurs Maximum values allowed
     * @param sRegexPattern Regular expression to match against
     */
    ConfigurationInfo(const std::string & sName,
                      Any::Type type,
                      const ConfigurationProperties & properties,
                      const std::vector<Any> & values,
                      const std::string & sUsage,
                      const Any & minValue,
                      const Any & maxValue,
                      std::size_t minOccurs,
                      std::size_t maxOccurs,
                      const std::string & sRegexPattern);

    /**
     * Creates a ConfiguratonInfo instance for non-numeric
     * configuration items (ones without min and max values)
     *
     * @param sName Name of the configuration item
     * @param properties Configuration properties mask
     * @param values Default configuration values
     * @param sUsage Parameter usage description
     * @param minOccurs Minimum values allowed
     * @param maxOccurs Maximum values allowed
     * @param sRegexPattern Regular expression to match against
     */
    ConfigurationInfo(const std::string & sName,
                      Any::Type type,
                      const ConfigurationProperties & properties,
                      const  std::vector<Any> & values,
                      const std::string & sUsage,
                      std::size_t minOccurs,
                      std::size_t maxOccurs,
                      const std::string & sRegexPattern);

    /**
     * Gets the underlying Any type
     *
     * @return type
     */
    Any::Type getType() const;

    /**
     * Gets the name
     *
     * @return name
     */
    const std::string & getName() const;

    /**
     * Check if there is a default
     *
     * @return @a true if has default
     */
    bool hasDefault() const;

    /**
     * Check if the item is required (item must be present)
     *
     * @return @a true if required
     */
    bool isRequired() const;

    /**
     * Check if the item is running-state modifiable
     *
     * @return @a true if running-state modifiable
     */
    bool isModifiable() const;

    /**
     * Gets the minimum occurrence
     *
     * @return min occurs
     */
    std::size_t getMinOccurs() const;
    
    /**
     * Gets the maximum occurrence
     *
     * @return max occurs
     */
    std::size_t getMaxOccurs() const;

    /**
     * Gets a reference vector of values as Anys
     *
     * @return vector of Any values
     */
    const std::vector<Any> & getValues() const;

    /**
     * Gets the minimum allowable value as an Any
     *
     * @return min value reference
     */
    const Any & getMinValue() const;

    /**
     * Gets the maximum allowable value as an Any
     *
     * @return max value reference
     */ 
    const Any & getMaxValue() const;

    /**
     * Gets the item description
     *
     * @return string description reference
     */
    const std::string & getUsage() const;

    /**
     * Gets the item validation regex
     *
     * @return string regex reference
     */
    const std::string & getRegexPattern() const;

    /**
     * Checks if the item is a numeric type
     *
     * @return @a true if numeric
     */
    bool isNumberic() const;

    /**
     * Sets the item values as Anys
     *
     * @param values  vector of Any values
     */
    void setValues(const std::vector<Any> & values);
    
  private:
    std::string sName_;
    ConfigurationProperties properties_;
    std::string sUsage_;
    std::size_t minOccurs_;
    std::size_t maxOccurs_;
    std::vector<Any> values_;
    Any minValue_;
    Any maxValue_;
    bool bIsNumberic_;
    Any::Type type_;
    std::string sRegexPattern_;
  };
}

#include "emane/configurationinfo.inl"

#endif // EMANECONFIGURATIONINFO_HEADER_
