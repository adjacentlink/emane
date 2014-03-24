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

#include "nemconfiguration.h"
#include "pluginconfiguration.h"

#include "emane/exception.h"

#include <libxml/tree.h>


EMANE::NEMConfiguration::NEMConfiguration(xmlNodePtr pNEMNode, 
                                          std::string sURI) : 
  LayerConfiguration{"nem"},
  u16Id_{0},
  type_{STRUCTURED}
{
  processDefinition("nem", sURI);

  u16Id_ = getAttrValNumeric(pNEMNode, "id");

  // overlay these param values on previously parsed
  overlayParams(pNEMNode);
}


EMANE::NEMConfiguration::~NEMConfiguration()
{
  for(auto layer : layers_) { delete layer; }

  layers_.clear();
}


ACE_UINT16 
EMANE::NEMConfiguration::getNEMId()
{
  return u16Id_;
}


const EMANE::LayerConfigurations 
&EMANE::NEMConfiguration::getLayers()
{
  return layers_;
}


bool 
EMANE::NEMConfiguration::isValid()
{
  return (type_ == UNSTRUCTURED) || (haveLayer("phy") && 
                                     haveLayer("mac") &&
                                     haveLayer("transport"));
}


void 
EMANE::NEMConfiguration::doProcessRootAttributes(xmlNodePtr pRoot)
{
  if(getAttrVal(pRoot, "type") == "unstructured")
    {
      type_ = UNSTRUCTURED;
    }
}


void 
EMANE::NEMConfiguration::doProcessChildNode(xmlNodePtr pNode)
{
  doProcessLayer(pNode);
}


void 
EMANE::NEMConfiguration::doProcessLayer(xmlNodePtr pNode)
{
  std::string sLayerName{reinterpret_cast<const char*>(pNode->name)};

  std::string sDefinitionFile{getAttrVal(pNode, "definition")};
  
  std::string sURI{};
  
  if(!sDefinitionFile.empty())
    {
      sURI = getDefinitionPath() + sDefinitionFile;
    }
  
  if(sLayerName == "shim")
    {
      layers_.push_back(new PluginConfiguration{pNode, sURI, sLayerName});
    }
  else
    {
      LayerConfigurationsIter pIter = findFirstLayer(sLayerName);

      if (pIter == layers_.end())
        {
          pIter = 
            layers_.insert(pIter, 
                           new PluginConfiguration{pNode, sURI, sLayerName});
        }
      else
        {
          // we previously parsed a similar layer, if the new definition
          // matches the old, just update
          if ((*pIter)->getDefinitionFile() == sDefinitionFile)
            {
              (*pIter)->overlayParams(pNode);
            }
          else
            {
              throw makeException<ValidateException>(
                                 "Unexpected definition for '%s'.\n\n"
                                 " * Found: '%s', expected: '%s'\n",
                                 pNode->name,
                                 sDefinitionFile.c_str(),
                                 (*pIter)->getDefinitionFile().c_str());
            }
        }
    }
}


EMANE::LayerConfigurationsIter
EMANE::NEMConfiguration::findFirstLayer(const std::string & sType)
{
  return find_if(layers_.begin(),
                 layers_.end(),
                 [sType](LayerConfiguration *l) 
                 { 
                   return l->getType() == sType; 
                 });
}


bool 
EMANE::NEMConfiguration::haveLayer(const std::string & type)
{
  return findFirstLayer(type) != layers_.end();
}
