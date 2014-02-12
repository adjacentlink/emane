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

#ifndef LAYERCONFIGURATION_HEADER_
#define LAYERCONFIGURATION_HEADER_

#include "emane/configurationupdate.h"
#include "emaneparseexception.h"
#include "emane/component.h"

#include <string>
#include <map>

#include <libxml/tree.h>

namespace EMANE
{
  /**
   * @class LayerConfiguration
   *
   * @brief Provides default implementation to common layer functionalities
   */
  class LayerConfiguration
  {
    using ParamStringValues = std::vector<std::string>;
    using ParamMap = std::map<std::string, ParamStringValues>;
    using ParamMapIter = ParamMap::iterator;

  public:
    /**
     * Constructor
     *
     * @param sType Layer type
     */
    LayerConfiguration(std::string sType);

    /**
     * Destructor
     */
    virtual ~LayerConfiguration() {}

    /**
     * Return A ConfigurationUpdateRequest built from the XML contents
     *
     * @return A ConfigurationUpdateRequest built from the XML contents
     */
    const ConfigurationUpdateRequest getConfigurationUpdateRequest();

    /**
     * Returns the type with this layer
     *
     * @return Type for this layer
     */
    std::string getType() const;

    /**
     * Returns the library with this layer
     *
     * @return Library for this layer
     */
    std::string getLibrary() const;

    /**
     * Returns the path to the definition file
     *
     * @return Path to the definition file
     */
    std::string getDefinitionPath() const;

    /**
     * Returns the name of the definition file
     *
     * @return Path the name of the definition file
     */
    std::string getDefinitionFile() const;

    /**
     * Overlay parent parameter values atop child values already parsed
     *
     * @param pNode Pointer to parent node of params to overlay
     */
    void overlayParams(xmlNodePtr pNode);

  protected:
    /**
     * Processes the definition of a given layer
     *
     * @param pxzName Name of the xml node to expect as root
     * @param sURI The URI to parse
     *
     * @exception ParseException ValidateException
     */
    void processDefinition(const char * pxzName, const std::string & sURI);

    /**
     * Forwards the processing of the root node to a specific derived class
     *
     * @param pRoot Pointer to the root node
     *
     * @exception ParseException ValidateException
     */
    virtual void doProcessRootAttributes(xmlNodePtr pRoot) ;


    /**
     * Process the children of the specifie node
     * 
     * @param pParent Pointer to the parent
     * @param paramMap Parameter map
     *
     * @exception ParseException ValidateException
     */
    virtual void doProcessChildren(xmlNodePtr pParent, 
                                   ParamMap & paramMap);

    /**
     * Forwards the child node to a specific derived class
     * 
     * @param pNode Pointer to the node
     *
     * @exception ParseException ValidateException
     */
    virtual void doProcessChildNode(xmlNodePtr pNode);

    /**
     * Parse the param node
     * 
     * @param paramMap to store parsed value
     * @param pParamNode Pointer to the param node
     *
     */
    void doProcessParam(ParamMap & paramMap, xmlNodePtr pParamNode);

    /**
     * Parse the param node
     * 
     * @param paramMap to store parsed values
     * @param pParamlistNode Pointer to the param list node
     *
     */
    void doProcessParamlist(ParamMap & paramMap, 
                            xmlNodePtr pParamlistNode);


    /**
     * Get the value of the attribute of the passed node
     * 
     * @param pNode Pointer to the xml Node
     * @param attributename Name of the attribute to retrieve
     *
     * @return String value of named attribute
     */
    std::string getAttrVal(xmlNodePtr pNode, const char * attributename);

    /**
     * Get the numeric value of the attribute of the passed node
     * 
     * @param pNode Pointer to the xml Node
     * @param attributename Name of the attribute to retrieve
     *
     * @return Numeric value of the named attribute
     */
    std::uint16_t getAttrValNumeric(xmlNodePtr pNode, 
                                    const char * attributename);

    /**
     * Identify URL & URN portion of the definition file
     */
    void setPathAndFile(const std::string sURI);

    /**
     * Configured name of this layer
     */
    std::string sName_;

    /**
     * Name of implementing library
     */
    std::string sLibrary_;
    
    /**
     * Internal map { paramname, [ paramvals ] }
     */
    ParamMap params_;

  private:
    /**
     * Type of layer
     */
    std::string sType_;

    /**
     * The definition path. The part of the definition URI up to 
     * and including the last separator - '/' or '\' depending on 
     * host (*nix or Windows) and whether the definition location is
     * passed as a true URI or just a local file path
     * The URI = sDefinitionPath_ + sDefinitionFile_
     */
    std::string sDefinitionPath_;
    
    /**
     * The definition file name. The part of the definition URI after 
     * the last separator - '/' or '\' depending on 
     * host (*nix or Windows) and whether the definition location is
     * passed as a true URI or just a local file path
     * The URI = sDefinitionPath_ + sDefinitionFile_
     */
    std::string sDefinitionFile_;
  };

}

#endif /* LAYERCONFIGURATION_HEADER_ */
