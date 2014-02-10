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

#include "layerconfiguration.h"
#include "configurationparser.h"

#include "emane/exception.h"
#include "emane/utils/parameterconvert.h"

#include <cstdlib>
#include <memory>
#include <libxml/xmlstring.h>


EMANE::LayerConfiguration::LayerConfiguration(std::string sType) : 
  sType_{sType}  
{ }


void 
EMANE::LayerConfiguration::processDefinition(const char * pszName,
                                             const std::string & sURI)
{
  setPathAndFile(sURI);

  xmlDocPtr pDoc{ConfigurationParser{}.parse(sURI)};

  xmlNodePtr pRoot{xmlDocGetRootElement(pDoc)};

  if (!xmlStrEqual(pRoot->name, reinterpret_cast<const xmlChar *>(pszName)))
    {
      xmlFreeDoc(pDoc);

      throw makeException<ParseException>(
                          "Failed to parse document in %s.\n\n"
                          "Possible reason(s):\n"
                          " * %s: Document root node is not '%s'\n", 
                          sURI.c_str(), 
                          sURI.c_str(), 
                          pszName);
    }

  // Allow derived-class to extract info from root
  doProcessRootAttributes(pRoot);

  sName_ = getAttrVal(pRoot, "name");
 
  doProcessChildren(pRoot, params_);

  xmlFreeDoc(pDoc);
}



void 
EMANE::LayerConfiguration::doProcessRootAttributes(xmlNodePtr)
{
  //default: do nothing
}


void 
EMANE::LayerConfiguration::doProcessChildren(xmlNodePtr pParent,
                                             ParamMap & paramMap)
{
  // process children
  xmlNodePtr pNode = pParent->children;

  while (pNode)
    {
      if (pNode->type == XML_ELEMENT_NODE)
        {
          if (xmlStrEqual(pNode->name, 
                          reinterpret_cast<const xmlChar*>("param")))
            {
              doProcessParam(paramMap, pNode);
            }
          else if(xmlStrEqual(pNode->name, 
                              reinterpret_cast<const xmlChar*>("paramlist")))
            {
              doProcessParamlist(paramMap, pNode);
            }
          else
            {
              doProcessChildNode(pNode);
            }
        }

      pNode = pNode->next;
    }
}


void 
EMANE::LayerConfiguration::doProcessChildNode(xmlNodePtr)
{
  //default: do nothing
}


void 
EMANE::LayerConfiguration::doProcessParam(ParamMap & paramMap,
                                          xmlNodePtr pParamNode)
{
  std::string sItemName{getAttrVal(pParamNode, "name")};
  
  if(paramMap.find(sItemName) != paramMap.end())
    {
      throw makeException<ParseException>(
                         "Failed to parse configuration document.\n\n"
                         "Possible reason(s):\n"
                         " * Duplicate configuration item '%s'\n", 
                         sItemName.c_str());
    }

  paramMap.insert(
           std::make_pair(sItemName, 
                          ParamStringValues{getAttrVal(pParamNode, "value")}));
}


void 
EMANE::LayerConfiguration::doProcessParamlist(ParamMap & paramMap,
                                              xmlNodePtr pParamlistNode)
{
  std::string sItemName{getAttrVal(pParamlistNode, "name")};

  if(paramMap.find(sItemName) != paramMap.end())
    {
      throw makeException<ParseException>(
                         "Failed to parse configuration document.\n\n"
                         "Possible reason(s):\n"
                         " * Duplicate configuration item '%s'\n", 
                         sItemName.c_str());
    }

  paramMap.insert(std::make_pair(sItemName, ParamStringValues{}));

  xmlNodePtr pItem = pParamlistNode->children;

  // DTD validation enforces at least one item element here
  while(pItem)
    {
      if(xmlStrEqual(pItem->name, reinterpret_cast<const xmlChar*>("item")))
        {
          paramMap[sItemName].push_back(getAttrVal(pItem, "value"));
        }

      pItem = pItem->next;
    }
}


std::string 
EMANE::LayerConfiguration::getAttrVal(xmlNodePtr pNode, 
                                      const char * attributename)
{
  std::string sVal{};

  xmlChar * pxzVal = 
    xmlGetProp(pNode, reinterpret_cast<const xmlChar*>(attributename));

  if(pxzVal)
    {
      sVal = reinterpret_cast<const char*>(pxzVal);

      xmlFree(pxzVal);
    }

  return sVal;
}


std::uint16_t 
EMANE::LayerConfiguration::getAttrValNumeric(xmlNodePtr pNode, 
                                             const char * attributename)
{
  return EMANE::Utils::ParameterConvert{
    getAttrVal(pNode, attributename)}.toUINT16();
}


void 
EMANE::LayerConfiguration::overlayParams(xmlNodePtr pRoot)
{
  ParamMap localParamMap{};

  doProcessChildren(pRoot, localParamMap);

  for(auto param : localParamMap)
    {
      ParamMapIter pos{params_.find(param.first)};
      if(pos != params_.end())
        {
          params_.erase(pos);
        }
      params_.insert(param);
    }
}


const EMANE::ConfigurationUpdateRequest
EMANE::LayerConfiguration::getConfigurationUpdateRequest()
{
  ConfigurationUpdateRequest request{};

  for (auto param : params_)
    {
      request.push_back(std::make_pair(param.first, param.second));
    }

  return request;
}


std::string 
EMANE::LayerConfiguration::getType() const
{
  return sType_;
}


std::string 
EMANE::LayerConfiguration::getLibrary() const
{
  return sLibrary_;
}


std::string 
EMANE::LayerConfiguration::getDefinitionPath() const
{
  return sDefinitionPath_;
}


std::string 
EMANE::LayerConfiguration::getDefinitionFile() const
{
  return sDefinitionFile_;
}


void 
EMANE::LayerConfiguration::setPathAndFile(const std::string sURI)
{
  /* Eval location and name */
  size_t pos = sURI.npos;

  if ((pos = sURI.rfind("/")) != sURI.npos) 
    {
      /* *nix */
      sDefinitionPath_ = sURI.substr(0, pos+1); //includes the '/'    
      sDefinitionFile_ = sURI.substr(pos+1); //to the end
    }
  else if ( (pos = sURI.rfind("\\")) != sURI.npos) 
    {
      /* windoze */
      sDefinitionPath_ = sURI.substr(0, pos+1); //includes the '\'    
      sDefinitionFile_ = sURI.substr(pos+1); //to the end
    }
  else 
    {
      sDefinitionPath_ = "";    
      sDefinitionFile_ = sURI;
    }
}
