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

/**
 * @page ConfigurationService Configuration Service
 *
 * The @ref EMANE::ConfigurationRegistrar "ConfigurationRegistrar" is used by components to register
 * configuration items. Configuration registration provides the emulator with all the knowledge it
 * needs to enforce the correctness of input parameters.
 *
 * @section RegisteringConfiguration Registering Configuration Items
 *
 * Configuration items can only be registered during @ref EMANE::Component::initialize "Component::initialize".
 * The @ref  EMANE::ConfigurationRegistrar "ConfigurationRegistrar" is accessible via the
 *  @ref EMANE::Component::initialize "initialize" method's @ref EMANE::Registrar "Registrar" argument.
 *
 *
 * The @ref EMANE::ConfigurationRegistrar "ConfigurationRegistrar" has two registration template methods which
 * allow components to register
 * numeric and non-numeric configuration items:
 *
 * - @ref EMANE::ConfigurationRegistrar::registerNumeric "registerNumeric"
 * - @ref EMANE::ConfigurationRegistrar::registerNonNumeric "registerNonNumeric"
 *
 * Numeric configuration items may have any one of the following types:
 * - std::int64_t
 * - std::int32_t
 * - std::int16_t
 * - std::int8_t
 * - std::uint64_t
 * - std::uint32_t
 * - std::uint16_t
 * - std::uint8_t
 * - bool
 * - float
 * - double
 *
 * @snippet src/models/mac/rfpipe/maclayer.cc configurationregistrar-registernumeric-snippet
 *
 * Non-Numeric configuration items may have any one of the following types:
 * - std::string
 * - INETAddr
 *
 * @snippet src/models/mac/rfpipe/maclayer.cc configurationregistrar-registernonnumeric-snippet
 *
 * Both register calls have parameters to specify a description string, 0 or more default values,
 * min/max occurrence counts (for multiplicity - xml @a paramlists), item properties such as @a running-state
 * modifiable and a regex for further validation. Additionally, @ref EMANE::ConfigurationRegistrar::registerNumeric
 * "registerNumeric" supports specifying a value range. The emulator will enforce data type, value range, occurrence
 * count, regex match and @a running-state modification permission without any component interaction.

 * Configuration Item properties:
 * - @ref EMANE::ConfigurationProperties::NONE "NONE"- No properties.
 * - @ref EMANE::ConfigurationProperties::REQUIRED "REQUIRED"- The parameter is required. Makes no sense when
 * combined with @ref EMANE::ConfigurationProperties::DEFAULT "DEFAULT".
 * - @ref EMANE::ConfigurationProperties::DEFAULT "DEFAULT"- One or more default values is specified.
 * - @ref EMANE::ConfigurationProperties::MODIFIABLE "MODIFIABLE" - Values may be modified while the emulator is
 * in the running state via @ref EMANE::RunningStateMutable::processConfiguration
 * "RunningStateMutable::processConfiguration".
 *
 * @section ValidatingConfigurationRelationships Validating Configuration Item Relationships
 *
 * An optional validator callable is used by the emulator framework to validate configuration item
 * relationships prior to calling @ref EMANE::Component::configure "Component::configure" or
 * @ref EMANE::RunningStateMutable::processConfiguration "RunningStateMutable::processConfiguration" on a
 * component.
 *
 * A component can register a validator callable using @ref EMANE::ConfigurationRegistrar::registerValidator
 * "ConfigurationRegistrar::registerValidator".
 *
 * @snippet src/models/mac/ieee80211abg/maclayer.cc configurationregistrar-registervalidator-snippet
 *
 * This callable is passed all known configuration items and returns a std::pair<std::string,bool> where @a second
 * is either @a true or @a false depending on whether the relationship of those configuration items are valid. When
 * an invalid parameter value is detected, @a first should contain an error description. Configuration updates are
 * transactional, so when a validator returns @a false the component will not see any of the configuration passed in
 * via @ref EMANE::Component::configure "configure" or @ref EMANE::RunningStateMutable::processConfiguration
 * "processConfiguration".
 *
 * For example, a model may have a min and max contention window. An invariant would be that min <= max. If a validator
 * testing that invariant was registered, the component would not have @ref EMANE::Component::configure "configure" or
 * @ref EMANE::RunningStateMutable::processConfiguration "processConfiguration" called on it if an attempt
 * was made to configure min > max. The validator would detect the error and return false.
 *
 * The framework keeps a cache of the current running configuration of each component. If an attempt was made
 * to only change a single parameter, in this illustrative case min contention window, the validator would
 * receive all the current cached configuration with the exception of min contention window which would reflect
 * the proposed value (min > max). This provides the validator with all it needs in order to verify all parameter
 * relationships.
 *
 * The validator callback is executed on an internal framework thread. You should not need to access any shared data
 * in the callable since all known configuration is passed in. However, if for some reason a component needs
 * to access shared data in the callable a synchronization object must be used.
 *
 * Once @ref EMANE::RunningStateMutable::processConfiguration "processConfiguration" is called a component has no
 * ability to reject configuration - that opportunity is provided by the validator callable. The framework will update
 * it's configuration cache and report those values as the new running state.
 *
 * Both @ref EMANE::Component::configure "configure" and @ref EMANE::RunningStateMutable::processConfiguration
 * "processConfiguration" are passed a single @ref EMANE::ConfigurationUpdate "ConfigurationUpdate" parameter.
 *
 * @snippet include/emane/configurationupdate.h configurationregistrar-configurationnameanyvalues-snippet
 *
 * @section HandlingConfigurationItems Handling Configuration Items
 *
 * For a configuration item with a min/max instance of [0:1], you can safely process the configuration value by accessing
 * element 0. You will only have a configuration item in @ref EMANE::ConfigurationUpdate "ConfigurationUpdate" if it
 * has 1 or more values.
 *
 * @snippet src/libemane/frameworkphy.cc configurationregistrar-processsingle-snippet
 *
 * For a parameter with multiple values, you will have to iterate over them.
 *
 * @snippet src/libemane/frameworkphy.cc configurationregistrar-processmultiplicity-snippet
 *
 * @section RunningStateConfigurationUpdates Running-State Configuration Updates
 *
 * When a running-state configuration update is received it is pushed onto the NEM's functor queue as an @ref
 * EMANE::RunningStateMutable::processConfiguration "RunningStateMutable::processConfiguration" method. In order for an
 * update to be passed to an NEM it must pass configuration item property checks and any registered validator.
 */
