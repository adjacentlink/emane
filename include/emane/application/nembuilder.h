/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
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
     * @brief Provides methods for contructing an EMANE Platform from
     * its constituent parts. 
     *
     * @note Erich Gamma, Richard Helm, Ralph Johnson, and John Vlissides.
     * Design Patterns: Elements of Reusable Object-Oriented Software.
     * Addison-Wesley, Reading MA, 1995
     * Bridge, p 152
     */
    class NEMBuilder
    {
    public:
      NEMBuilder();

      ~NEMBuilder();
    
      /**
       * Build a PHY layer
       *
       * @param id id of the NEM that will contain the phy
       * @param sLibraryFile Name of the dll containing the layer

       * @return Smart pointer to an initialized PHY

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
       * Build a MAC layer
       *
       * @param id id of the NEM that will contain the mac
       * @param sLibraryFile Name of the dll containing the layer
       *
       * @return Smart pointer to an initialized MAC
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
       * Build a Shim layer
       *
       * @param id id of the NEM that will contain the shim
       * @param sLibraryFile Name of the dll containing the layer
       *
       * @return Smart pointer to an initialized and configured Shim
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
       * Build a NEM
       *
       * @param id NEM id
       * @param layers The NEMLayers comprising the NEM
       *
       * @return Smart pointer to an initialized NEM
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
                                    const ConfigurationUpdateRequest & request);

      /**
       * Build an NEM Manager
       *
       * @param id Platform id
       * @param nems The NEMs comprising the Platform
       *
       * @return Smart pointer to an initialized Platform
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
      buildNEMManager(PlatformId id,
                      NEMs & nems,
                      const ConfigurationUpdateRequest & request);

    private:
      class NEMBuilderImpl;
      NEMBuilderImpl * pImpl_;
    };
  }
}

#endif // EMANEAPPLICATIONNEMBUILDER_HEADER_
