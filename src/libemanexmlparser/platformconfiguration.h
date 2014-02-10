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

#ifndef PLATFORMCONFIGURATION_HEADER_
#define PLATFORMCONFIGURATION_HEADER_

#include "emaneparseexception.h"
#include "nemconfiguration.h"

#include <string>
#include <list>

namespace EMANE
{
  using NEMConfigurations = std::list<NEMConfiguration*> ;

  /**
   * @class PlatformConfiguration
   *
   * @brief Contains configuration for the Platform
   */
  class PlatformConfiguration : public LayerConfiguration
  {
  public:
    /**
     * Constructor
     *
     * @param sFile   Filename of the platform config
     *
     * @exception ParseException ValidateException
     */
    PlatformConfiguration(const std::string &sFile);

    /**
     * Destructor
     */
    ~PlatformConfiguration();

    /**
     * Gets this platform's id
     *
     * @return Id of this platform
     */
    ACE_UINT16 getPlatformId();

    /**
     * Returns the container with NEMs
     *
     * @return Container with NEM configuration objects
     */
    const NEMConfigurations &getNEMs();

  protected:
    /**
     * Does processing of the root node as if it was a 'platform'
     *
     * @param pRoot Pointer to the root node
     *
     * @exception ParseException ValidateException
     */
    void doProcessRootAttributes(xmlNodePtr pRoot);

    /**
     * Does processing of a child node as if it was an 'nem'
     * 
     * @param pNode Pointer to the node
     *
     * @exception ParseException ValidateException
     */
    void doProcessChildNode(xmlNodePtr pNode);

  private:
    /**
     * Platform Id
     */
    ACE_UINT16 u16Id_;

    /**
     * List with NEM Configurations
     */
    NEMConfigurations nems_;
  };

}

#endif /* PLATFORMCONFIGURATION_HEADER_ */
