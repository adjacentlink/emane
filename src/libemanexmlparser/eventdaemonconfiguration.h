/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2009 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANEEVENTDAEMONCONFIGURATION_HEADER
#define EMANEEVENTDAEMONCONFIGURATION_HEADER

/*
 * Includes
 */
#include "layerconfiguration.h"
#include "nemconfiguration.h"
#include "emane/types.h"

#include <string>

namespace EMANE
{
  /**
   * @class EventDaemonConfiguration
   *
   * @brief Implementation of Configuration to properly configure the EventDaemonManager.
   *
   * The EventDaemonConfiguration class is a concrete class implementing the 
   * Configuration interface. The implementation allows for general treatment
   * of layer configuration data for any/all config layers (including event 
   * generators.
   *
   * @sa LayerConfiguration
   */
  class EventDaemonConfiguration : public EMANE::LayerConfiguration
  {
  public:
    /**
     * Constructor
     *
     * @param sFile File with the eventdaemon/daemon configuration
     *
     * @exception ParseException ValidateException
     */
    EventDaemonConfiguration(const std::string &sFile);
    
    /**
     * Destructor
     */
    ~EventDaemonConfiguration();
    
    /**
     * Gets this daemon's nem id
     *
     * @return Id of this daemon's nem
     */
    ACE_UINT16 getNEMId();

    /**
     * Returns the container with Agent Layers
     *
     * @return Container with Layer (agent) config objects
     */
    const LayerConfigurations &getAgents();
    
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
     * Container with Daemon configurations
     */
    LayerConfigurations agents_;

    /**
     * NEMId
     */
    EMANE::NEMId nemId_;
  };

} // end namespace EMANE  
#endif /* EMANEEVENTDAEMONCONFIGURATION_HEADER */
