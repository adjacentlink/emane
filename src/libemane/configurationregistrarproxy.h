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

#ifndef EMANECONFIGURATIONREGISTRARPROXY_HEADER_
#define EMANECONFIGURATIONREGISTRARPROXY_HEADER_

#include "emane/configurationregistrar.h"
#include "emane/types.h"
#include "configurationservice.h"

namespace EMANE
{
  /**
   * @class ConfigurationRegistrarProxy
   *
   * @brief Provides component specific proxy access to
   * the configuration service.
   */

  class ConfigurationRegistrarProxy : public ConfigurationRegistrar
  {
  public:
    /**
     * Creates a ConfigurationRegistrarProxy.
     * 
     * @param service Reference to the configuration service
     * @param buildId BUild id associated with the proxy
     */
    ConfigurationRegistrarProxy(ConfigurationService & service,
                                BuildId buildId);
    
    void registerValidator(ConfigurationValidator validator) override;

  private:
    ConfigurationService & service_;
    BuildId buildId_;

    void registerNumericAny(const std::string & sName,
                            Any::Type type,
                            const ConfigurationProperties & properties,
                            const std::vector<Any> & values,
                            const std::string & sUsage,
                            const Any & minValue,
                            const Any & maxValue,
                            std::size_t minOccurs,
                            std::size_t maxOccurs,
                            const std::string & sRegexPattern) override;
    
    void registerNonNumericAny(const std::string & sName,
                               Any::Type type,
                               const ConfigurationProperties & properties,
                               const std::vector<Any> & values,
                               const std::string & sUsage,
                               std::size_t minOccurs,
                               std::size_t maxOccurs,
                               const std::string & sRegexPattern) override;

    
  };
}

#endif //EMANECONFIGURATIONREGISTRARPROXY_HEADER_
