/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef NEMCONFIGURATION_HEADER_
#define NEMCONFIGURATION_HEADER_

#include "layerconfiguration.h"
#include "emaneparseexception.h"

#include <string>
#include <list>

namespace EMANE
{
  /* Container for layers */
  using LayerConfigurations = std::list<LayerConfiguration *>;
  using LayerConfigurationsIter = LayerConfigurations::iterator;

  /**
   * @class NEMConfiguration
   *
   * @brief Contains all configuration associated with an NEM
   *
   * @sa LayerConfiguration
   */
  class NEMConfiguration : public LayerConfiguration
  {
  public:
    /* NEM type */
    enum NEMType
      {
        STRUCTURED,
        UNSTRUCTURED,
        INVALID
      };

    /**
     * Constructor
     *
     * @param pNEMNode Pointer to the Root node of this NEM
     * @param sDefinitionURI URI of the NEM definition file
     *
     * @exception ParseException ValidateException
     */
    NEMConfiguration(xmlNodePtr pNEMNode, const std::string sDefinitionURI);

    /**
     * Destructor
     */
    ~NEMConfiguration();

    /**
     * Gets this nem's id
     *
     * @return Id of this nem
     */
    ACE_UINT16 getNEMId();

    /**
     * Returns the container with Layers
     *
     * @return Container with Layer (phy/mac/shim/transport) config objects
     */
    const LayerConfigurations &getLayers();

    /**
     * Returns whether-or-not this NEM is 'valid'
     *
     * @return True if this NEM is valid, false otherwise
     */
    bool isValid();


    /**
     * Returns whether-or-not this NEM transport is external
     *
     * @return True if external, false otherwise
     */
    bool isExternalTransport();

  protected:
    /**
     * Does processing of the root node as if it was an 'nem'
     *
     * @param pRoot Pointer to the root node
     *
     * @exception ParseException ValidateException
     */
    void doProcessRootAttributes(xmlNodePtr pRoot);

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
     * Id of this NEM
     */
    ACE_UINT16 u16Id_;

    /**
     * Container with LayerConfigurations
     */
    LayerConfigurations layers_;

    /**
     * NEM type
     */
    NEMType type_;

    /**
     * Transport external flag
     */
    bool bExternalTransport_;

    /**
     * Process a layer element
     */
    void doProcessLayer(xmlNodePtr pNode);

    /**
     * Find layer type
     */
    LayerConfigurationsIter findFirstLayer(const std::string& sType);

    /**
     * Check for a given layer type
     */
    bool haveLayer(const std::string& type);
  };

} // end namespace EMANE

#endif /* NEMCONFIGURATION_HEADER_ */
