/*
 * Copyright (c) 2013-2014,2018 - Adjacent Link LLC, Bridgewater,
 * New Jersey
 * Copyright (c) 2008-2011 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANEAPPLICATIONNEMBUILDER_HEADER_
#define EMANEAPPLICATIONNEMBUILDER_HEADER_

#include "emane/application/nemmanager.h"
#include "emane/application/nem.h"
#include "emane/nemlayer.h"
#include "emane/maclayerimpl.h"
#include "emane/radioserviceprovider.h"

#include "emane/configurationupdate.h"

#include <string>
#include <list>
#include <memory>

namespace EMANE
{
  namespace Application
  {
    using NEMLayers = std::list<std::unique_ptr<NEMLayer>>;
    using NEMs = std::list<std::unique_ptr<NEM>>;

    /**
     * @class NEMBuilder
     *
     * @brief Provides methods for constructing an emulator instance from
     * its constituent parts.
     */
    class NEMBuilder
    {
    public:
      NEMBuilder();

      ~NEMBuilder();

      /**
       * Builds a PHY layer
       *
       * @param id id of the NEM that will contain the phy
       * @param sLibraryFile Name of the dll containing the layer
       * @param request Configuration update request
       * @param bSkipConfigure Flag indicating whether to skip
       * calling Component::configure
       *
       * @return Unique pointer to an initialized PHY
       *
       * @throw Utils::FactoryException when a DLL load error occurs.
       * @throw InitializeException when an error occurs during
       * initialization.
       * @throw ConfigureException when an error occurs during
       * configure.
       */
      std::unique_ptr<NEMLayer>
      buildPHYLayer(NEMId id,
                    const std::string & sLibraryFile,
                    const ConfigurationUpdateRequest & request,
                    bool bSkipConfigure = false);

      /**
       * Builds a MAC layer
       *
       * @param id id of the NEM that will contain the mac
       * @param sLibraryFile Name of the dll containing the layer
       * @param request Configuration update request
       * @param bSkipConfigure Flag indicating whether to skip
       * calling Component::configure
       *
       * @return Unique pointer to an initialized MAC
       *
       * @throw Utils::FactoryException when a DLL load error occurs.
       * @throw InitializeException when an error occurs during
       * initialization.
       * @throw ConfigureException when an error occurs during
       * configure.
       */
      std::unique_ptr<NEMLayer>
      buildMACLayer(NEMId id,
                    const std::string & sLibraryFile,
                    const ConfigurationUpdateRequest & request,
                    bool bSkipConfigure = false);

      /**
       * Builds a MAC layer from template.
       *
       * @tparam T MACLayerImplementor derived implementation
       *
       * @param id id of the NEM that will contain the mac
       * @param RegistrationName Registration name
       * @param request Configuration update request
       * @param bSkipConfigure Flag indicating whether to skip
       * calling Component::configure
       *
       * @return std::pair with first being a borrowed reference to
       * the newly created mac layer and second being a
       * unique_ptr<NEMLayer> that should be added (moved)
       * to an NEM layers list.
       *
       * @throw InitializeException when an error occurs during
       * initialization.
       * @throw ConfigureException when an error occurs during
       * configure.
       */
      template<typename T, typename... Args>
      std::pair<T *, std::unique_ptr<EMANE::NEMLayer>>
      buildMACLayer_T(NEMId id,
                      const std::string & RegistrationName,
                      const ConfigurationUpdateRequest & request,
                      bool bSkipConfigure,
                      Args... args);

      /**
       * Builds a Shim layer
       *
       * @param id id of the NEM that will contain the shim
       * @param sLibraryFile Name of the dll containing the layer
       * @param request Configuration update request
       * @param bSkipConfigure Flag indicating whether to skip
       * calling Component::configure
       *
       * @return Unique pointer to an initialized and configured Shim
       *
       * @throw Utils::FactoryException when a DLL load error occurs.
       * @throw InitializeException when an error occurs during
       * initialization.
       * @throw ConfigureException when an error occurs during
       * configure.
       */
      std::unique_ptr<NEMLayer>
      buildShimLayer(NEMId id,
                     const std::string & sLibraryFile,
                     const ConfigurationUpdateRequest & request,
                     bool bSkipConfigure = false);

      /**
       * Builds a Transport layer
       *
       * @param id id of the NEM that will contain the transport
       * @param sLibraryFile Name of the dll containing the layer
       * @param request Configuration update request
       * @param bSkipConfigure Flag indicating whether to skip
       * calling Component::configure
       *
       * @return Unique pointer to an initialized and configured Transport
       *
       * @throw Utils::FactoryException when a DLL load error occurs.
       * @throw InitializeException when an error occurs during
       * initialization.
       * @throw ConfigureException when an error occurs during
       * configure.
       */
      std::unique_ptr<NEMLayer>
      buildTransportLayer(NEMId id,
                          const std::string & sLibraryFile,
                          const ConfigurationUpdateRequest & request,
                          bool bSkipConfigure = false);


      /**
       * Builds an NEM
       *
       * @param id NEM id
       * @param layers The NEMLayers comprising the NEM
       * @param request Configuration update request
       * @param bHasExternalTransport Flag indicating whether transport is
       * external
       *
       * @return Unique pointer to an initialized NEM
       *
       * @throw InitializeException when an error occurs during
       * initialization.
       * @throw BuildException when an error occurs building an NEM
       * from the specified layers.
       * @throw ConfigureException when an error occurs during
       * configure.
       */
      std::unique_ptr<NEM> buildNEM(NEMId id,
                                    NEMLayers & layers,
                                    const ConfigurationUpdateRequest & request,
                                    bool bHasExternalTransport);

      /**
       * Builds an NEM Manager
       *
       * @param uuid Instance UUID
       * @param nems The NEMs comprising the Platform
       * @param request Configuration update request
       *
       * @return Unique pointer to an initialized Platform
       *
       * @throw Utils::FactoryException when a DLL load error occurs.
       * @throw InitializeException when an error occurs during
       * initialization.
       * @throw BuildException when an error occurs building an NEM
       * from the specified layers.
       * @throw ConfigureException when an error occurs during
       * configure.
       */
      std::unique_ptr<NEMManager>
      buildNEMManager(const uuid_t & uuid,
                      NEMs & nems,
                      const ConfigurationUpdateRequest & request);

    private:
      class NEMBuilderImpl;
      NEMBuilderImpl * pImpl_;

      std::unique_ptr<NEMLayer>
      buildMACLayer_i(MACLayerImplementor * pImpl,
                      PlatformServiceProvider * pProvider,
                      NEMId id,
                      const std::string & sLibraryFile,
                      const ConfigurationUpdateRequest & request,
                      bool bSkipConfigure);

      RadioServiceProvider * createRadioService(NEMId id);

      PlatformServiceProvider * createPlatformService();
    };
  }
}

#include "emane/application/nembuilder.inl"

#endif // EMANEAPPLICATIONNEMBUILDER_HEADER_
