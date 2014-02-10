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

#include "emane/types.h"
#include "emane/configurationupdate.h"

namespace EMANE
{
  namespace Application
  {
    class ConfigurationController
    {
    public:
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
      static ConfigurationUpdate buildUpdates(BuildId buildId,
                                              const ConfigurationUpdateRequest & parameters);
      
      
      static ConfigurationManifest getConfigurationManifest(BuildId buildId);
    };
  }
}
