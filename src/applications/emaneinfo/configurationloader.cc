/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
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
 * * Neither the name of Adjacent Link LLC nor the names of its
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

#include "configurationloader.h"
#include "emane/configureexception.h"

#include <libxml/parser.h>

#include <utility>

EMANE::Application::ConfigurationLoader::ConfigurationLoader(std::string & sConfigurationFile)
{
  xmlParserCtxtPtr pContext{xmlNewParserCtxt()};

  xmlDocPtr pDoc = xmlCtxtReadFile(pContext,
                                   sConfigurationFile.c_str(),
                                   0,
                                   XML_PARSE_NOENT |   
                                   XML_PARSE_DTDLOAD | 
                                   XML_PARSE_DTDATTR | 
                                   XML_PARSE_DTDVALID);
      
  if(!pDoc)
    {
      throw makeException<ConfigureException>("Failed to parse document: %s",
                                              sConfigurationFile.c_str());
    }

  if(pContext->valid == 0)
    {
      throw makeException<ConfigureException>("Failed to validate document: %s",
                                              sConfigurationFile.c_str());
    }

  xmlNodePtr pRoot = xmlDocGetRootElement(pDoc);

  xmlChar * pLibrary = xmlGetProp(pRoot,BAD_CAST "library");
  
  sPluginName_ = reinterpret_cast<char *>(pLibrary);

  xmlFree(pLibrary);

  if(!xmlStrcmp(pRoot->name,BAD_CAST "mac"))
    {
      type_ = PluginType::MAC;
    }
  else if(!xmlStrcmp(pRoot->name,BAD_CAST "phy"))
    {
      type_ = PluginType::PHY;
    }
  else if(!xmlStrcmp(pRoot->name,BAD_CAST "shim"))
    {
      type_ = PluginType::SHIM;
    }
  else if(!xmlStrcmp(pRoot->name,BAD_CAST "eventgenerator"))
    {
      type_ = PluginType::GENERATOR;
    }
  else if(!xmlStrcmp(pRoot->name,BAD_CAST "eventagent"))
    {
      type_ = PluginType::AGENT;
    }
  else if(!xmlStrcmp(pRoot->name,BAD_CAST "transport"))
    {
      type_ = PluginType::TRANSPORT;
    }
  else
    {
      throw makeException<ConfigureException>("Unkown plugin type: %s",
                                              pRoot->name);
    }
  
  for(xmlNodePtr pNode = pRoot->children; pNode; pNode = pNode->next)
    {
      if(pNode->type == XML_ELEMENT_NODE)
        {
          if(!xmlStrcmp(pNode->name,BAD_CAST "param"))
            {
              xmlChar * pName = xmlGetProp(pNode,BAD_CAST "name");
              xmlChar * pValue = xmlGetProp(pNode,BAD_CAST "value");

              std::string sName{std::string{reinterpret_cast<const char *>(pName)}};
              std::string sValue{std::string{reinterpret_cast<const char *>(pValue)}};
              
              xmlFree(pName);
              xmlFree(pValue);
              
              // test for duplicate configuration items
              if(isUniqueConfigurationName(sName))
                {
                  request_.push_back(std::make_pair(sName,
                                                    std::vector<std::string>{sValue}));
                }
              else
                {
                  throw makeException<ConfigureException>("Duplicate parameter name: %s",
                                                          sName.c_str());
                }
            }
          else if(!xmlStrcmp(pNode->name,BAD_CAST "paramlist"))
            {
              xmlChar * pName = xmlGetProp(pNode,BAD_CAST "name");
              
              std::string sName{std::string{reinterpret_cast<const char *>(pName)}};
              
              xmlFree(pName);
              
              // test for duplicate configuration items
              if(isUniqueConfigurationName(sName))
                {
                  std::vector<std::string> values;
                  
                  for(xmlNodePtr pItem = pNode->children; pItem; pItem = pItem->next)
                    {
                      if(pItem->type == XML_ELEMENT_NODE)
                        {
                          if(!xmlStrcmp(pItem->name,BAD_CAST "item"))
                            {
                              xmlChar * pValue = xmlGetProp(pItem,BAD_CAST "value");
                              values.push_back(reinterpret_cast<char *>(pValue));
                              xmlFree(pValue);
                            }
                        }
                    }

                  request_.push_back(std::make_pair(sName,values));
                }
              else
                {
                  throw makeException<ConfigureException>("Duplicate parameter name: %s",
                                                          sName.c_str());
                }
            }
        }
    }
}
        
const EMANE::ConfigurationUpdateRequest & 
EMANE::Application::ConfigurationLoader::getConfigurationUpdateRequest() const
{
  return request_;
}
        
const std::string
EMANE::Application::ConfigurationLoader::getPluginName() const
{
  return sPluginName_;
}

bool
EMANE::Application::ConfigurationLoader::isUniqueConfigurationName(std::string & sName)
{
  return std::find_if(request_.begin(),
                      request_.end(),
                      std::bind(std::equal_to<std::string>(),
                                std::bind(&ConfigurationUpdateRequest::value_type::first,
                                          std::placeholders::_1),
                                sName)) == request_.end();
}


EMANE::Application::PluginType EMANE::Application::ConfigurationLoader::getPluginType() const
{
  return type_;
}
