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

#include "emane/models/frameworkphy/antennaprofilemanifest.h"
#include "emane/utils/parameterconvert.h"
#include "emane/models/frameworkphy/antennaprofileexception.h"
#include "emane/models/frameworkphy/positionneu.h"
#include "emane/constants.h"

#include <libxml/parser.h>

void EMANE::AntennaProfileManifest::load(const std::string & sAntennaProfileURI)
{
  xmlParserCtxtPtr pContext{xmlNewParserCtxt()};

  if(!pContext)
    {
      throw AntennaProfileException{"Unable to create a parser context"};
    }

  xmlDocPtr pDoc{xmlCtxtReadFile(pContext, sAntennaProfileURI.c_str(), NULL, XML_PARSE_DTDVALID)};

  if(!pDoc)
    {
      
      xmlFreeParserCtxt(pContext);
      throw makeException<AntennaProfileException>("Unable to read %s",sAntennaProfileURI.c_str());
    }

  if(pContext->valid == false)
    {          
      xmlFreeDoc(pDoc);
      xmlFreeParserCtxt(pContext);
      throw makeException<AntennaProfileException>("Validation failure %s",sAntennaProfileURI.c_str());
    }
   
  xmlNodePtr pRoot{xmlDocGetRootElement(pDoc)};
    
  if(!pRoot)
    {
      xmlFreeDoc(pDoc);
      xmlFreeParserCtxt(pContext);
      throw makeException<AntennaProfileException>("Invalid document root %s",
                                                   sAntennaProfileURI.c_str());
    }

  for(xmlNodePtr pNode = pRoot->children; pNode; pNode = pNode->next)
    {
      if(pNode->type == XML_ELEMENT_NODE)
        {
          if(!xmlStrcmp(pNode->name,BAD_CAST "profile"))
            {
              xmlChar * pId{xmlGetProp(pNode,BAD_CAST "id")};
                  
              std::uint16_t u16Id =
                EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pId)).toUINT16();
                  
              xmlFree(pId);

              xmlChar * pAntennaPatternURI{xmlGetProp(pNode,BAD_CAST "antennapatternuri")};

              std::string sAntennaPatternURI{ reinterpret_cast<const char *>(pAntennaPatternURI)};

              xmlFree(pAntennaPatternURI);

              xmlChar * pBlockagePatternURI{xmlGetProp(pNode,BAD_CAST "blockagepatternuri")};

              std::string sBlockagePatternURI{};

              if(pBlockagePatternURI)
                {
                  sBlockagePatternURI = reinterpret_cast<const char *>(pBlockagePatternURI);

                  xmlFree(pBlockagePatternURI);
                }

              PositionNEU antennaPlacement{};

              for(xmlNodePtr pChileNode = pNode->children; pChileNode; pChileNode = pChileNode->next)
                {
                  if(pChileNode->type == XML_ELEMENT_NODE)
                    {
                      if(!xmlStrcmp(pChileNode->name,BAD_CAST "placement"))
                        {
                          xmlChar * pNorth{xmlGetProp(pChileNode,BAD_CAST "north")};
                  
                          double dNorth =
                            EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pNorth)).toDouble();
                          
                          xmlFree(pNorth);

                          xmlChar * pEast{xmlGetProp(pChileNode,BAD_CAST "east")}; 
                          double dEast =
                            EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pEast)).toDouble();
                          
                          xmlFree(pEast);

                          xmlChar * pUp{xmlGetProp(pChileNode,BAD_CAST "up")};
                  
                          double dUp =
                            EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pUp)).toDouble();
                          
                          xmlFree(pUp);

                          antennaPlacement = PositionNEU{dNorth,dEast,dUp};
                        }
                    }
                }

              AntennaPattern * pAntennaPattern{};

              auto iter = antennaPatternStore_.find(sAntennaPatternURI);

              if(iter == antennaPatternStore_.end())
                {
                  pAntennaPattern = new AntennaPattern{sAntennaPatternURI,"antennapattern",DBM_MIN};
                  
                  antennaPatternStore_.insert(std::make_pair(sAntennaPatternURI,
                                                             std::unique_ptr<AntennaPattern>(pAntennaPattern)));
                }
              else
                {
                  pAntennaPattern = iter->second.get();
                }

              
              AntennaPattern * pBlockagePattern{};

              if(!sBlockagePatternURI.empty())
                {
                  auto iter = antennaPatternStore_.find(sBlockagePatternURI);

                  if(iter == antennaPatternStore_.end())
                    {
                      pBlockagePattern = new AntennaPattern{sBlockagePatternURI,"blockagepattern",0};
                      
                      antennaPatternStore_.insert(std::make_pair(sBlockagePatternURI,
                                                                 std::unique_ptr<AntennaPattern>(pBlockagePattern)));
                    }
                  else
                    {
                      pBlockagePattern = iter->second.get();
                    }
                }

              if(!profiles_.insert(std::make_pair(u16Id,
                                                  std::make_tuple(pAntennaPattern,pBlockagePattern,antennaPlacement))).second)
                {
                  throw makeException<AntennaProfileException>("Duplicate antenna profile id %hu",u16Id);
                }
            }
        }
    }

  xmlFreeDoc(pDoc);
  xmlFreeParserCtxt(pContext);
}


std::pair<std::tuple<EMANE::AntennaPattern *,EMANE::AntennaPattern *,EMANE::PositionNEU>,bool>
EMANE::AntennaProfileManifest::getProfileInfo(AntennaProfileId antennaProfileId) const
{
  const auto iter = profiles_.find(antennaProfileId);

  if(iter != profiles_.end())
    {
      return {iter->second,true};
    }

  return {{},false};
}
