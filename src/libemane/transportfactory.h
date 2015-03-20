/*
 * Copyright (c) 2013,2015 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2008-2009 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANETRANSPORTFACTORY_HEADER_
#define EMANETRANSPORTFACTORY_HEADER_

#include "emane/transport.h"

#include "emane/utils/factoryexception.h"

namespace EMANE
{
  /**
   * @class TransportFactory
   *
   * @brief Factory for creating Transports.  The factory
   * manages the DLL allowing for the creation of multiple transports.
   * It does not appear obvious why we would expaind functionality to
   * include multiple transports for a single NEM...
   *
   * @note DLLs must stay open in order to use their contents
   */
  class TransportFactory
  {
  public:
    /**
     * Constructor
     *
     * @param sLibraryName Filename of DLL
     *
     * @throw Utils::FactoryException
     */
    TransportFactory(const std::string & sLibraryName);
    
    ~TransportFactory();
    
    /**
     * Create an Transport
     *
     * @param nemId the NEMId
     * @param  pPlatformService pointer to the PlatformServiceProvider 
     *
     *
     * @returns Transport reference
     */
    Transport * createTransport(NEMId nemId, PlatformServiceProvider *pPlatformService) const;

    /**
     * Destory an Transport
     *
     * @param pTransport Reference to Transport 
     */
    void destoryTransport(Transport * pTransport) const;
    
  private:
    using CreateTransportFunc = Transport * (*)(NEMId, PlatformServiceProvider *);
    using DestroyTransportFunc = void (*)(Transport*); 

    void * pLibHandle_;
    CreateTransportFunc createTransportFunc_;
    DestroyTransportFunc destroyTransportFunc_;
  };
}

#endif //EMANETRANSPORTFACTORY_HEADER_

