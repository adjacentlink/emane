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

#ifndef EMANECONFIGURATIONREGISTRAR_HEADER_
#define EMANECONFIGURATIONREGISTRAR_HEADER_

#include "emane/configurationproperties.h"
#include "emane/configurationexception.h"
#include "emane/configurationupdate.h"
#include "emane/any.h"

#include <initializer_list>
#include <string>
#include <limits>

namespace EMANE
{
  /**
   * @class ConfigurationRegistrar
   *
   * @brief The ConfigurationRegistrar allows NEM layers to register
   * the configuration items they require.
   * 
   * @note Registration may only occur during Component::initialize()
   */
  class ConfigurationRegistrar
  {
  public:
    /**
     * Destroys an instance
     */
    virtual ~ConfigurationRegistrar();

    /**
     * Registers a numeric configuration item
     *
     * @param sName Name of the configuration item to be registered
     * @param properties Configuration properties mask
     * @param values Default configuration values
     * @param sUsage Parameter usage description
     * @param minValue Minimum acceptable value
     * @param maxValue Maximum acceptable value
     * @param minOccurs Minimum values allowed
     * @param maxOccurs Maximum values allowed
     * @param sRegexPattern Regular expression to match against
     *
     * @throw ConfigurationException Throw when a registration error
     * occurs.
     */
    template<typename T>
    void registerNumeric(const std::string & sName,
                         const ConfigurationProperties & properties = ConfigurationProperties::NONE,
                         const std::initializer_list<T> & values = {},
                         const std::string & sUsage = "",
                         T minValue = std::numeric_limits<T>::lowest(),
                         T maxValue = std::numeric_limits<T>::max(),
                         std::size_t minOccurs = 1,
                         std::size_t maxOccurs = 1,
                         const std::string & sRegexPattern = {});
    /**
     * Registers a non-numeric configuration item
     *
     * @param sName Name of the configuration item to be registered
     * @param properties Configuration properties mask
     * @param values Default configuration values
     * @param sUsage Parameter usage description
     * @param minOccurs Minimum values allowed
     * @param maxOccurs Maximum values allowed
     * @param sRegexPattern Regular expression to match against
     *
     * @throw ConfigurationException Throw when a registration error
     * occurs.
     */
    template<typename T>
    void registerNonNumeric(const std::string & sName,
                            const ConfigurationProperties & properties = ConfigurationProperties::NONE,
                            const std::initializer_list<T> & values = {},
                            const std::string & sUsage = "",
                            std::size_t minOccurs = 1,
                            std::size_t maxOccurs = 1,
                            const std::string & sRegexPattern = {});

    /**
     * Registers a ConfigurationValidator callable. This callable is passed 
     * the complete list of configuration sent to an NEM during Component::configure()
     * and RunningStateMutable::processConfiguration(). The validator returns @a true to
     * indicate the configuration is valid and @a flase to indicate an error.
     *
     * @param validator
     *
     * @note Configuration updates are transactional, so when a validator returns @a flase
     * the NEM will not see any of the configuration passed in via  Component::configure()
     * or RunningStateMutable::processConfiguration().
     *
     * @note For RunningStateMutable::processConfiguration() a validator is the only way to
     * indicate an error in the configuration.
     */
    virtual void registerValidator(ConfigurationValidator validator) = 0;
    
  protected:
    ConfigurationRegistrar();

    /**
     * Registers a numeric configuration item using an Any.
     *
     * @param sName Name of the configuration item to be registered
     * @param type Underlying Any type
     * @param properties Configuration properties mask
     * @param values Default configuration values
     * @param sUsage Parameter usage description
     * @param minValue Minimum acceptable value
     * @param maxValue Maximum acceptable value
     * @param minOccurs Minimum values allowed
     * @param maxOccurs Maximum values allowed
     * @param sRegexPattern Regular expression to match against
     *
     * @exception ConfigurationException Throw when a registration error
     * occurs.
     */
    virtual void registerNumericAny(const std::string & sName,
                                    Any::Type type,
                                    const ConfigurationProperties & properties,
                                    const std::vector<Any> & values,
                                    const std::string & sUsage,
                                    const Any & minValue,
                                    const Any & maxValue,
                                    std::size_t minOccurs,
                                    std::size_t maxOccurs,
                                    const std::string & sRegexPattern = {}) = 0;

    /**
     * Registers a non-numeric configuration item using an Any.
     *
     * @param sName Name of the configuration item to be registered
     * @param type Underlying Any type
     * @param properties Configuration properties mask
     * @param values Default configuration values
     * @param sUsage Parameter usage description
     * @param minOccurs Minimum values allowed
     * @param maxOccurs Maximum values allowed
     * @param sRegexPattern Regular expression to match against
     *
     * @exception ConfigurationException Throw when a registration error
     * occurs.
     */
    virtual void registerNonNumericAny(const std::string & sName,
                                       Any::Type type,
                                       const ConfigurationProperties & properties,
                                       const std::vector<Any> & values,
                                       const std::string & sUsage,
                                       std::size_t minOccurs,
                                       std::size_t maxOccurs,
                                       const std::string & sRegexPattern = {}) = 0;

  };
}

#include "emane/configurationregistrar.inl"

#endif //EMANECONFIGURATIONREGISTRAR_HEADER_
