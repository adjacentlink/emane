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

#include "antennapattern.h"
#include "emane/utils/parameterconvert.h"
#include "antennaprofileexception.h"

#include <libxml/parser.h>

EMANE::AntennaPattern::AntennaPattern(const std::string & sAntennaPatternURI,
                                      const std::string sSubRootName,
                                      double dMissingValue):
  dMissingValue_{dMissingValue}
{
  xmlParserCtxtPtr pContext{xmlNewParserCtxt()};

  if(!pContext)
    {
      throw AntennaProfileException{"Unable to create a parser context"};
    }

  xmlDocPtr pDoc{xmlCtxtReadFile(pContext,sAntennaPatternURI.c_str(),nullptr,XML_PARSE_DTDVALID)};

  if(!pDoc)
    {
      
      xmlFreeParserCtxt(pContext);
      throw makeException<AntennaProfileException>("Unable to read %s",sAntennaPatternURI.c_str());
    }

  if(pContext->valid == false)
    {          
      xmlFreeDoc(pDoc);
      xmlFreeParserCtxt(pContext);
      throw makeException<AntennaProfileException>("Validation failure %s",sAntennaPatternURI.c_str());
    }
   
  xmlNodePtr pRoot{xmlDocGetRootElement(pDoc)};
    
  if(!pRoot)
    {
      xmlFreeDoc(pDoc);
      xmlFreeParserCtxt(pContext);
      throw makeException<AntennaProfileException>("Invalid document root %s",
                                                   sAntennaPatternURI.c_str());
    }

  
  for(xmlNodePtr pNode = pRoot->children; pNode; pNode = pNode->next)
    {
      if(pNode->type == XML_ELEMENT_NODE)
        {
          if(!xmlStrcmp(pNode->name,BAD_CAST sSubRootName.c_str()))
            {
              std::int16_t i16ElevationRangeMax{};
              
              for(xmlNodePtr pElevationNode = pNode->children;
                  pElevationNode;
                  pElevationNode = pElevationNode->next)
                {
                  if(pElevationNode->type == XML_ELEMENT_NODE)
                    {
                      if(!xmlStrcmp(pElevationNode->name,BAD_CAST "elevation"))
                        {
                          xmlChar * pMin{xmlGetProp(pElevationNode,BAD_CAST "min")};
                          
                          std::int16_t i16ElevationMin =
                            EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pMin)).toINT16();
                          
                          xmlFree(pMin);
                          
                          xmlChar * pMax{xmlGetProp(pElevationNode,BAD_CAST "max")};
                          
                          std::int16_t i16ElevationMax =
                            EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pMax)).toINT16();
                          
                          xmlFree(pMax);
              
                          if(i16ElevationMin > i16ElevationMax)
                            {
                              throw makeException<AntennaProfileException>("Bad entry %s: elevation [%hd,%hd] min > max",
                                                                           sAntennaPatternURI.c_str(),
                                                                           i16ElevationMin,
                                                                           i16ElevationMax);
                            }

                          if(i16ElevationRangeMax >= i16ElevationMin && !elevationBearingGainMap_.empty())
                            {
                              throw makeException<AntennaProfileException>("Bad entry %s: elevation [%hd,%hd] may may be"
                                                                           " out of order or overlap with a previous elevation",
                                                                           sAntennaPatternURI.c_str(),
                                                                           i16ElevationMin,
                                                                           i16ElevationMax);
                            }
                          else
                            {
                              i16ElevationRangeMax = i16ElevationMax;
                            }
            
                          BearingGainMap * pBearingGainMap{new BearingGainMap};
                          
                          bearings_.push_back(std::unique_ptr<BearingGainMap>(pBearingGainMap));

                          std::int16_t i16BearingRangeMax{};

                          for(xmlNodePtr pBearingNode = pElevationNode->children;
                              pBearingNode;
                              pBearingNode = pBearingNode->next)
                            {
                              if(pBearingNode->type == XML_ELEMENT_NODE)
                                {
                                  if(!xmlStrcmp(pBearingNode->name,BAD_CAST "bearing"))
                                    {
                                     xmlChar * pMin{xmlGetProp(pBearingNode,BAD_CAST "min")};
                          
                                     std::int16_t i16BearingMin =
                                       EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pMin)).toINT16();
                                     
                                     xmlFree(pMin);
                                     
                                     xmlChar * pMax{xmlGetProp(pBearingNode,BAD_CAST "max")};
                                     
                                     std::int16_t i16BearingMax =
                                       EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pMax)).toINT16();
                                     
                                     xmlFree(pMax); 
                                     
                                     if(i16BearingMin > i16BearingMax)
                                       {
                                         throw makeException<AntennaProfileException>("Bad entry %s: elevation [%hd,%hd] bearing [%hd,%hd] min > max",
                                                                                      sAntennaPatternURI.c_str(),
                                                                                      i16ElevationMin,
                                                                                      i16ElevationMax,
                                                                                      i16BearingMin,
                                                                                      i16BearingMax);
                                       }
                                     
                                     if(i16BearingRangeMax >= i16BearingMin && !pBearingGainMap->empty())
                                       {
                                         throw makeException<AntennaProfileException>("Bad entry %s: elevation [%hd,%hd] bearing [%hd,%hd]"
                                                                                      " may be out of order or overlap with a previous bearing",
                                                                                      sAntennaPatternURI.c_str(),
                                                                                      i16ElevationMin,
                                                                                      i16ElevationMax,
                                                                                      i16BearingMin,
                                                                                      i16BearingMax);
                                       }
                                     else
                                       {
                                         i16BearingRangeMax = i16BearingMax;
                                       }
                                     
                                     for(xmlNodePtr pGainNode = pBearingNode->children;
                                         pGainNode;
                                         pGainNode = pGainNode->next)
                                       {
                                         if(pGainNode->type == XML_ELEMENT_NODE)
                                           {
                                             if(!xmlStrcmp(pGainNode->name,BAD_CAST "gain"))
                                               {
                                                 xmlChar * pGain{xmlGetProp(pGainNode,BAD_CAST "value")};
                                                 
                                                 double dGain =
                                                   EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pGain)).toDouble();
                                                 
                                                 xmlFree(pGain);
                                                 
                                                 if(pBearingGainMap->find(i16BearingMin-1) == pBearingGainMap->end())
                                                   {
                                                     (*pBearingGainMap)[i16BearingMin-1] = dMissingValue_;
                                                   }

                                                 (*pBearingGainMap)[i16BearingMax] = dGain;
                                               }
                                           }
                                       }
                                    }
                                }
                            }

                          if(elevationBearingGainMap_.find(i16ElevationMin-1) == elevationBearingGainMap_.end())
                            {
                              elevationBearingGainMap_[i16ElevationMin-1] = nullptr;
                            }
                          
                          elevationBearingGainMap_[i16ElevationMax] = pBearingGainMap;
                        }
                    }
                }
            }
        }
    }

  xmlFreeDoc(pDoc);
  xmlFreeParserCtxt(pContext);
}

double EMANE::AntennaPattern::getGain(std::int16_t iBearing,std::int16_t iElevation) const
{
  double dGain{dMissingValue_};

  //  use 0 for bearing when 360
  if(iBearing == 360)
    {
      iBearing = 0;
    }
  
  const auto iter = elevationBearingGainMap_.lower_bound(iElevation);

  if(iter != elevationBearingGainMap_.end())
    {
      if(iter->second)
        {
          const auto bearingIter = iter->second->lower_bound(iBearing);
          
          if(bearingIter != iter->second->end())
            {
              dGain = bearingIter->second;
            }
        }
    }

  return dGain;
}
