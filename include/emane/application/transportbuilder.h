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

#ifndef EMANETRANSPORTBUILDER_HEADER_
#define EMANETRANSPORTBUILDER_HEADER_
 
#include "emane/types.h"
#include "emane/application/transportmanager.h"
#include "emane/application/transportadapter.h"
#include "emane/transport.h"
#include "emane/configurationupdate.h"

#include <string>
#include <list>
#include <memory>

namespace EMANE
{
  namespace Application
  {
    using TransportAdapters = std::list<std::unique_ptr<TransportAdapter>>;

    /**
     * @class TransportBuilder
     *
     * @brief Provides methods for contructing transports and a manager
     * to contain and control them as a a group.
     *
     * Reference:
     * Erich Gamma, Richard Helm, Ralph Johnson, and John Vlissides.
     * Design Patterns: Elements of Reusable Object-Oriented Software.
     * Addison-Wesley, Reading MA, 1995
     * Builder, p 97
     */
    class TransportBuilder
    {
    public:
      TransportBuilder();
    
      ~TransportBuilder();
    
      /**
       * Build a TransportManager
       *
       * @param adapters transport pairs to manage
       * @param request configuration update
       *
       * @return Smart pointer to an initialized and configured TransportManager
       *
       * @throw InitializeException when an error occurs during
       * initialization.
       * @throw BuildException when an error occurs building an NEM
       * from the specified layers.
       * @throw ConfigureException when an error occurs during
       * configure.
       */
      std::unique_ptr<TransportManager>
      buildTransportManager(TransportAdapters & adapters,
                            const ConfigurationUpdateRequest& request);


      /**
       * Build a TransportAdapter
       *
       * @param pTransport the transport connected to the adapter
       * @param pItems pointer to the ConfigurationItems list
       *
       * @return Smart pointer to an initialized and configured
       * TransportAdapter
       *
       * @throw InitializeException when an error occurs during
       * initialization.
       * @throw BuildException when an error occurs building an NEM
       * from the specified layers.
       * @throw ConfigureException when an error occurs during
       * configure.
       */
      std::unique_ptr<TransportAdapter>
      buildTransportAdapter(std::unique_ptr<Transport> & pTransport,
                            const ConfigurationUpdateRequest & request);
        
      //throw(BuildException,ConfigureException,InitializeException);

      /**
       * Build a Transport 
       *
       * @param id NEMId of the NEM associated with this transport
       * @param sLibraryFile Name of the dll containing the generator
       * @param request configuration update
       *
       * @return Smart pointer to an initialized and configured Transport
       *
       * @throw Utils::FactoryException when a DLL load error occurs.
       * @throw InitializeException when an error occurs during
       * initialization.
       * @throw ConfigureException when an error occurs during
       * configure.
       */
      std::unique_ptr<Transport>
      buildTransport(NEMId id,
                     const std::string & sLibraryFile,
                     const ConfigurationUpdateRequest & request,
                     bool bSkipConfigure = false);

    
      /**
       * Build a Transport 
       *
       * @brief Build an instance of the transport named by the template 
       * parameter. T must be a subclass of EMANE::Transport and provide 
       * a constructor with signature:
       * T(EMANE::NEMId id, EMANE::PlatformServiceProvider * p)
       * T will be instantiated via this constructor.
       *
       * @param id NEMId of the NEM associated with this transport
       * @param request configuration update
       *
       * @throw InitializeException when an error occurs during
       * initialization.
       * @throw ConfigureException when an error occurs during
       * configure.
       */
      template<typename T>
      T *
      buildTransport(const NEMId id,
                     const ConfigurationUpdateRequest& request);

    private:
      PlatformServiceProvider * newPlatformService() const;
      
      void initializeTransport(Transport * pTransport, 
                               PlatformServiceProvider * pProvider,
                               const ConfigurationUpdateRequest & request,
                               bool bSkipConfigure = false) const;
    };
  }
}

#include "emane/application/transportbuilder.inl"


#endif //EMANETRANSPORTBUILDER_HEADER_
