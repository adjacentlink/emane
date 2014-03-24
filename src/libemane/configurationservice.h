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

#ifndef EMANECONFIGURATIONSERVICE_HEADER_
#define EMANECONFIGURATIONSERVICE_HEADER_

#include "emane/configurationinfo.h"
#include "emane/configurationupdate.h"
#include "emane/configurationproperties.h"
#include "emane/configurationexception.h"
#include "emane/runningstatemutable.h"
#include "emane/any.h"
#include "emane/types.h"
#include "emane/utils/singleton.h"

#include <string>
#include <map>

#include <mutex>

namespace EMANE
{
  class ConfigurationService : public Utils::Singleton<ConfigurationService>
  {
  public:
    void registerRunningStateMutable(BuildId buildId,
                                     RunningStateMutable * pRunningStateMutable);

    /**
     * Registers a numeric configuration item using an Any.
     *
     * @param buildId Build id of the registering component
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
    void registerNumericAny(BuildId buildId,
                            const std::string & sName,
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
     * Registers a non-numeric configuration item using an Any.
     *
     * @param buildId Build id of the registering component
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
    void registerNonNumericAny(BuildId buildId,
                               const std::string & sName,
                               Any::Type type,
                               const ConfigurationProperties & properties,
                               const std::vector<Any> & values,
                               const std::string & sUsage,
                               std::size_t minOccurs,
                               std::size_t maxOccurs,
                               const std::string & sRegexPattern);

    /**
     * Gets the configuration information for all registered configuration
     * items associated with a specified build id.
     *
     * @param buildId Build id of the component
     *
     * @return ConfigurationInfos container holding configuration information.
     *
     * @exception ConfigurationException Thrown when an invalid buildId is specified.
     */
    ConfigurationManifest getConfigurationManifest(BuildId buildId) const;


    /**
     * Gets the configuration parmeter value(s) for specified registered configuration
     * items associated with a specified build id.
     *
     * @param buildId Build id of the component
     * @param names Names of items of interest. Empty list will return all items.
     *
     * @return configuration name values pairs
     *
     * @exception ConfigurationException Thrown when an invalid buildId is specified or
     * an unknown configuration parameter is requested.
     */
    std::vector<std::pair<std::string,std::vector<Any>>>
      queryConfiguration(BuildId buildId,
                         const std::vector<std::string> & names = {}) const;
    
    /**
     * Builds a ConfigurationUpdate for processing by a component. Local configuration
     * cache is updated and future calls to getConfigurationInfos will reflect the latest
     * values processed by buildUpdates. Cache update only occurs if the entire update
     * request is successfully validated.
     *
     * @param buildId Build id of the component
     * @param parameters Name string values pairs of requested configuration updates. 
     *
     * @return ConfigurationInfos container holding configuration information.
     *
     * @exception ConfigurationException Thrown when a configuration validation error occurs.
     * Errors include values or instance counts out of range, incorrect data type, string to 
     * data type conversion errors and unregistered configuration targets.
     */
    ConfigurationUpdate
    buildUpdates(BuildId buildId,
                 const ConfigurationUpdateRequest & parameters);


    void update(BuildId buildId, const ConfigurationUpdate & update);

    void registerValidator(BuildId buildId, ConfigurationValidator validator);

  protected:
    ConfigurationService();

  private:
    using ConfigurationStore = std::map<std::string,ConfigurationInfo>;
    using BuildIdConfigurationStore = std::map<BuildId,ConfigurationStore>;
    using RunningStateMUtables = std::map<BuildId,RunningStateMutable *>;
    using ValidatorStore = std::map<BuildId,std::vector<ConfigurationValidator>>;

    BuildIdConfigurationStore buildIdConfigurationStore_;
    RunningStateMUtables runningStateMutables_;
    ValidatorStore validatorStore_;

    mutable std::mutex mutex_;

    void registerAny(BuildId buildId,
                     const std::string & sName,
                     ConfigurationInfo && configurationInfo);
  };

  using ConfigurationServiceSingleton = ConfigurationService;
}

#endif //EMANECONFIGURATIONSERVICE_HEADER_
