/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2009-2010 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef TRANSPORTINSTANCECONFIGURATION_HEADER_
#define TRANSPORTINSTANCECONFIGURATION_HEADER_

#include "layerconfiguration.h"
#include "nemconfiguration.h"
#include "emaneparseexception.h"

#include <string>
#include <list>

namespace EMANE
{
  /**
   * @class TransportInstanceConfiguration
   *
   * @brief Contains all configuration associated with a TransportInstance
   *
   * @sa LayerConfiguration
   */
  class TransportInstanceConfiguration : public LayerConfiguration
  {
  public:
    /**
     * Constructor
     *
     * @param pTransportInstanceNode Pointer to the Root node of this TransportInstance
     * @param sDefinitionURI URI of the definition file
     *
     * @exception ParseException ValidateException
     */
    TransportInstanceConfiguration(xmlNodePtr pTransportInstanceNode,
                                   const std::string sDefinitionURI);

    /**
     * Destructor
     */
    ~TransportInstanceConfiguration();

    /**
     * Gets this instance's id
     *
     * @return Id of this instance
     */
    ACE_UINT16 getId();

    /**
     * Returns the container with Layers
     *
     * @return Container with Layer (phy/mac/shim/transport) config objects
     */
    const LayerConfigurations &getLayers();

  protected:
    /**
     * Does processing of a child node as if it was a 'layer'
     * 
     * @param pNode Pointer to the node
     *
     * @exception ParseException ValidateException
     */
    void doProcessChildNode(xmlNodePtr pNode);

  private:
    /**
     * Id of this TransportInstance
     */
    ACE_UINT16 u16Id_;

    /**
     * Container with LayerConfigurations
     */
    LayerConfigurations layers_;
  };

} // end namespace EMANE

#endif /* TRANSPORTINSTANCECONFIGURATION_HEADER_ */
