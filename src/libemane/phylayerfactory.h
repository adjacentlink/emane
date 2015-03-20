/*
 * Copyright (c) 2013,2015 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2008 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANEPHYLAYERFACTORY_HEADER_
#define EMANEPHYLAYERFACTORY_HEADER_

#include "emane/types.h"
#include "emane/phylayerimpl.h"

namespace EMANE
{
  /**
   * @class LayerFactory
   *
   * @brief Factory for creating PHYLayerImplementor layers .  The factory
   * manages the DLL allowing for the creation of layers
   * of the same type.
   *
   * @note DLLs must stay open in order to use their contents
   */
  class PHYLayerFactory
  {
  public:
    /**
     * Constructor
     *
     * @param sLibraryName Filename of DLL
     *
     * @throw Utils::FactoryException
     */
    PHYLayerFactory(const std::string & sLibraryName);

    ~PHYLayerFactory();
    
    /**
     * Create a PHYLayerImplementor layer
     *
     * @param nemId NEM id
     * @param pPlatformServiceProvider platform service
     *
     * @returns PHYLayerImplementor reference to newly created layer
     */
    PHYLayerImplementor * createLayer(NEMId nemId,
                                      PlatformServiceProvider * pPlatformServiceProvider) const;

    /**
     * Destory a PHYLayerImplementor layer
     *
     * @param layer Layer to destory
     *
     */
    void destoryLayer(PHYLayerImplementor * layer) const;
    
  private:
    using CreateLayerFunc = PHYLayerImplementor * (*)(NEMId,
                                                      PlatformServiceProvider *); 
    using DestroyLayerFunc = void (*)(PHYLayerImplementor*); 

    void * pLibHandle_;
    CreateLayerFunc  createLayerFunc_;
    DestroyLayerFunc destroyLayerFunc_;
  };
}


#endif //EMANEPHYLAYERFACTORY_HEADER_
