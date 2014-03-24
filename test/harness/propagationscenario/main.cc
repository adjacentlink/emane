/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include <cstdlib>
#include <iostream>
#include <memory>

#include <libxml/parser.h>
#include <libxml/xmlschemas.h>

#include "locationmanager.h"
#include "propagationmodelalgorithm.h"
#include "freespacepropagationmodelalgorithm.h"
#include "tworaypropagationmodelalgorithm.h"
#include "precomputedpropagationmodelalgorithm.h"

#include "emane/utils/parameterconvert.h"
#include "emane/frequencysegment.h"

#include <ace/Get_Opt.h>

void usage();

int main(int argc, char* argv[])
{
  LIBXML_TEST_VERSION;
  
  const ACE_TCHAR options[] = ACE_TEXT("hs:");

  ACE_Get_Opt cmd_opts(argc,argv,options);
  
  if(cmd_opts.long_option(ACE_TEXT("help"),'h') == -1)
    {
      ACE_ERROR_RETURN((LM_ERROR,ACE_TEXT("%s\n"),
                        ACE_TEXT("config long option: help")),
                       EXIT_FAILURE);
    }
  
  if(cmd_opts.long_option(ACE_TEXT("schema"),'s',ACE_Get_Opt::ARG_REQUIRED) == -1)
    {
      ACE_ERROR_RETURN((LM_ERROR,ACE_TEXT("%s\n"),
                        ACE_TEXT("config long option: schema")),EXIT_FAILURE);
    }
  
  int iOption{};
  std::string sScenario{};
  std::string sManifest{};
  std::string sSchema{"propagationscenario.xsd"};
  
  while((iOption = cmd_opts()) != EOF)
    {
      switch(iOption)
        {
        case 'h':
          // --help
          usage();
          return 0;
              
        case 's':
          // --schema
          sSchema = cmd_opts.opt_arg();
          break;

        case ':':
          // missing arguement
          std::cerr<<"-"<<cmd_opts.opt_opt()<<"requires an argument"<<std::endl;
          return EXIT_FAILURE;
              
        default:
          std::cerr<<"Unknown option: "<<cmd_opts.last_option()<<std::endl;
          return EXIT_FAILURE;
        }
    }

  int iIndex = cmd_opts.opt_ind();
  
  if(iIndex < cmd_opts.argc())
    {
      sScenario = argv[iIndex];
    }
  else
    {
      std::cerr<<"missing scenario"<<std::endl;
      return EXIT_FAILURE;
    }

      
  std::cout.precision(10);

  try
    {
      xmlDocPtr pSchemaDoc{xmlReadFile(sSchema.c_str(),
                                       NULL,
                                       XML_PARSE_NONET)};
      
      if(!pSchemaDoc)
        {
          std::cerr<<"unable to open schema"<<std::endl;
          return EXIT_FAILURE;
        }

  
      xmlSchemaParserCtxtPtr pParserContext{xmlSchemaNewDocParserCtxt(pSchemaDoc)};
  
      if(!pParserContext)
        {
          std::cerr<<"bad schema context"<<std::endl;
          return EXIT_FAILURE;
        }
  
      xmlSchemaPtr pSchema{xmlSchemaParse(pParserContext)};

      if(!pSchema)
        {
          std::cerr<<"bad schema parser"<<std::endl;
          return EXIT_FAILURE;
        }


      xmlSchemaValidCtxtPtr pSchemaValidCtxtPtr{xmlSchemaNewValidCtxt(pSchema)};

      if(!pSchemaValidCtxtPtr)
        {
          std::cerr<<"bad schema valid context"<<std::endl;
          return EXIT_FAILURE;
        }
  
      xmlDocPtr pDoc = xmlReadFile(sScenario.c_str(),nullptr,0);

  
      if(xmlSchemaValidateDoc(pSchemaValidCtxtPtr, pDoc))
        {
          return EXIT_FAILURE;
        }

   
      xmlNodePtr pRoot = xmlDocGetRootElement(pDoc);

      xmlChar * pNEMId = xmlGetProp(pRoot,BAD_CAST "nem");
      
      auto id =
        EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pNEMId)).toUINT16();
      
      xmlFree(pNEMId);
      

      xmlChar * pAlgorithm = xmlGetProp(pRoot,BAD_CAST "algorithm");

      std::unique_ptr<EMANE::PropagationModelAlgorithm> pPropagationModelAlgorithm_;


      // regex has already validated values
      if(!xmlStrcmp(pAlgorithm,BAD_CAST "precomputed"))
        {
          pPropagationModelAlgorithm_.reset(new EMANE::PrecomputedPropagationModelAlgorithm{id});
        }
      else if(!xmlStrcmp(pAlgorithm,BAD_CAST "2ray"))
        {
          pPropagationModelAlgorithm_.reset(new EMANE::TwoRayPropagationModelAlgorithm{id});
        }
      else
        {
          pPropagationModelAlgorithm_.reset(new EMANE::FreeSpacePropagationModelAlgorithm{id});
        }
      
      xmlFree(pAlgorithm);

      EMANE::LocationManager locationManager{id};

      int iActionIndex{};
      
      for(xmlNodePtr pNode = pRoot->children; pNode; pNode = pNode->next)
        {
          if(pNode->type == XML_ELEMENT_NODE)
            {
              if(!xmlStrcmp(pNode->name,BAD_CAST "action"))
                {
                  for(xmlNodePtr pActionNode = pNode->children;
                      pActionNode;
                      pActionNode = pActionNode->next)
                    {
                      if(pActionNode->type == XML_ELEMENT_NODE)
                        {
                          if(!xmlStrcmp(pActionNode->name,BAD_CAST "locations"))
                            {
                              EMANE::Events::Locations locations;
                              ++iActionIndex;
                              for(xmlNodePtr pPOVNode = pActionNode->children;
                                  pPOVNode;
                                  pPOVNode = pPOVNode->next)
                                {
                                  if(pPOVNode->type == XML_ELEMENT_NODE)
                                    {
                                      if(!xmlStrcmp(pPOVNode->name,BAD_CAST "pov"))
                                        {
                                          xmlChar * pNEMId = xmlGetProp(pPOVNode,BAD_CAST "nem");
                                          
                                          auto nemId =
                                            EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pNEMId)).toUINT16();
                                          
                                          xmlFree(pNEMId);

                                          EMANE::Position position{};
                                          EMANE::Orientation orientation{};
                                          EMANE::Velocity velocity{};
                                          bool bHasVelocity{};
                                          bool bHasOrientation{};

                                          for(xmlNodePtr pEntryNode = pPOVNode->children;
                                              pEntryNode;
                                              pEntryNode = pEntryNode->next)
                                            {
                                              if(pEntryNode->type == XML_ELEMENT_NODE)
                                                {
                                                  if(!xmlStrcmp(pEntryNode->name,BAD_CAST "position"))
                                                    {
                                                      xmlChar * pLatitude = xmlGetProp(pEntryNode,BAD_CAST "latitude");

                                                      auto latitude =
                                                        EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pLatitude)).toDouble();

                                                      xmlFree(pLatitude);

                                                      xmlChar * pLongitude = xmlGetProp(pEntryNode,BAD_CAST "longitude");
                              
                                                      auto longitude =
                                                        EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pLongitude)).toDouble();
                                                      
                                                      xmlFree(pLongitude);

                                                      xmlChar * pAltitude = xmlGetProp(pEntryNode,BAD_CAST "altitude");
                              
                                                      auto altitude =
                                                        EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pAltitude)).toDouble();
                                                      
                                                      xmlFree(pAltitude);
                                                      
                                                      position = EMANE::Position{latitude,longitude,altitude};
                                                    }
                                                  else if(!xmlStrcmp(pEntryNode->name,BAD_CAST "orientation"))
                                                    {
                                                      xmlChar * pPitch = xmlGetProp(pEntryNode,BAD_CAST "pitch");
                                                       
                                                      auto pitch =
                                                        EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pPitch)).toDouble();
                                                      
                                                      xmlFree(pPitch);

                                                      xmlChar * pRoll = xmlGetProp(pEntryNode,BAD_CAST "roll");
                              
                                                      auto roll =
                                                        EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pRoll)).toDouble();
                                                      
                                                      xmlFree(pRoll); 
                                                      
                                                      xmlChar * pYaw = xmlGetProp(pEntryNode,BAD_CAST "yaw");
                              
                                                      auto yaw =
                                                        EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pYaw)).toDouble();
                                                      
                                                      xmlFree(pYaw);

                                                      orientation = EMANE::Orientation{roll,pitch,yaw};

                                                      bHasOrientation = true;

                                                    } 
                                                  else if(!xmlStrcmp(pEntryNode->name,BAD_CAST "velocity"))
                                                    {
                                                      xmlChar * pAzimuth = xmlGetProp(pEntryNode,BAD_CAST "azimuth");
                                                       
                                                      auto azimuth =
                                                        EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pAzimuth)).toDouble();
                                                      
                                                      xmlFree(pAzimuth);

                                                      xmlChar * pElevation = xmlGetProp(pEntryNode,BAD_CAST "elevation");
                              
                                                      auto elevation =
                                                        EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pElevation)).toDouble();
                                                      
                                                      xmlFree(pElevation); 

                                                      xmlChar * pMagnitude = xmlGetProp(pEntryNode,BAD_CAST "magnitude");
                              
                                                      auto magnitude =
                                                        EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pMagnitude)).toDouble();
                                                      
                                                      xmlFree(pMagnitude); 

                                                      velocity = EMANE::Velocity{azimuth,elevation,magnitude};

                                                      bHasVelocity = true;
                                                    } 
                                                }
                                            }

                                          std::cout<<"["<<iActionIndex<<"] location "<<nemId<<" P:["<<position.getLatitudeDegrees()<<","<<position.getLongitudeDegrees()<<","<<position.getAltitudeMeters()<<"]";
                                          
                                          if(bHasOrientation)
                                            {
                                              std::cout<<" O:["<<orientation.getPitchDegrees()<<","<<orientation.getRollDegrees()<<","<<orientation.getYawDegrees()<<"]";
                                            }

                                          if(bHasVelocity)
                                            {
                                              std::cout<<" V:["<<velocity.getAzimuthDegrees()<<","<<velocity.getElevationDegrees()<<","<<velocity.getMagnitudeMetersPerSecond()<<"]";
                                            }

                                          std::cout<<std::endl;

                                          locations.push_back({nemId,position,{orientation,bHasOrientation},{velocity,bHasVelocity}});
                                        }
                                    }
                                }
                              std::cout<<std::endl;
                              
                              locationManager.update(locations);
                              
                            }
                          else if(!xmlStrcmp(pActionNode->name,BAD_CAST "pathloss"))
                            {
                              EMANE::Events::Pathlosses pathlosses;

                              ++iActionIndex;

                              for(xmlNodePtr pEntryNode = pActionNode->children;
                                  pEntryNode;
                                  pEntryNode = pEntryNode->next)
                                {
                                  if(pEntryNode->type == XML_ELEMENT_NODE)
                                    {
                                      if(!xmlStrcmp(pEntryNode->name,BAD_CAST "entry"))
                                        {
                                          xmlChar * pNEMId = xmlGetProp(pEntryNode,BAD_CAST "nem");

                                          auto nemId =
                                            EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pNEMId)).toUINT16();
                                          
                                          xmlFree(pNEMId);

                                          xmlChar * pPathloss = xmlGetProp(pEntryNode,BAD_CAST "pathloss");

                                          auto fPathloss =
                                            EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pPathloss)).toFloat();
                                          
                                          xmlFree(pPathloss);

                                          float fRevPathloss{fPathloss};

                                          xmlChar * pRevPathloss = xmlGetProp(pEntryNode,BAD_CAST "rpathloss");
                                          

                                          if(pRevPathloss)
                                            {
                                              fRevPathloss = 
                                                EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pRevPathloss)).toFloat();

                                              xmlFree(pRevPathloss);
                                            }
                                          
                                     
                                          pathlosses.push_back({nemId,fPathloss,fRevPathloss});

                                          std::cout<<"["<<iActionIndex<<"] pathloss "<<nemId<<" Forward Pathloss: "<<fPathloss<< " Reverse Pathloss: "<<fRevPathloss<<std::endl;
                                        }
                                    }
                                }
                              std::cout<<std::endl;

                              pPropagationModelAlgorithm_->update(pathlosses);
                            }
                          else if(!xmlStrcmp(pActionNode->name,BAD_CAST "request"))
                            {
                              ++iActionIndex;
                              EMANE::FrequencySegments segments;
                              std::vector<double> powers;
                              
                              for(xmlNodePtr pChildNode = pActionNode->children; pChildNode; pChildNode = pChildNode->next)
                                {
                                  if(pChildNode->type == XML_ELEMENT_NODE)
                                    {
                                      if(!xmlStrcmp(pChildNode->name,BAD_CAST "segment"))
                                        {
                                          xmlChar * pFrequency = xmlGetProp(pChildNode,BAD_CAST "frequency");
                                       
                                          auto frequency = EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pFrequency)).toUINT64();
                                          
                                          xmlFree(pFrequency);
                                       
                                          segments.push_back({frequency,EMANE::Microseconds::zero(),EMANE::Microseconds::zero()});
                                        }
                                    }
                                }

                              xmlChar * pNEMId = xmlGetProp(pActionNode,BAD_CAST "nem");
                                          
                              auto nemId =
                                EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pNEMId)).toUINT16();
                              
                              xmlFree(pNEMId);

                              
                              auto locationInfoRet = locationManager.getLocationInfo(nemId);
                              
                              // get the propagation model pathloss between a pair of nodes for *each* segment
                              auto pathlossInfo = (*pPropagationModelAlgorithm_)(nemId,
                                                                                 locationInfoRet.first,
                                                                                 segments);

                              std::cout<<"["<<iActionIndex<<"] request "<<nemId<<std::endl;

                              if(pathlossInfo.second)
                                {
                                  for(const auto & dPathloss : pathlossInfo.first)
                                    {
                                      std::cout<<"  "<<dPathloss<<std::endl;
                                    }
                                }
                              else
                                {
                                  std::cout<<"  Not enough information to determine pathloss"<<std::endl;
                                }
                              
                              std::cout<<std::endl;
                            }
                        }
                    }
                }
            }
        }

    }
  catch(EMANE::Exception & exp)
    {
      std::cout<<exp.what();
      return EXIT_FAILURE;
    }
  
  return EXIT_SUCCESS;
}

void usage()
{
  std::cout<<"usage: gaintest [OPTIONS]... MANIFESTXML SCENARIOXML"<<std::endl;
  std::cout<<std::endl;
  std::cout<<"options:"<<std::endl;
  std::cout<<"  -h, --help                     Print this message and exit."<<std::endl;
  std::cout<<"  -r, --realtime                 Set realtime scheduling"<<std::endl;
  std::cout<<"  -s, --schema                   Scenario schema"<<std::endl;
  std::cout<<"                                   Default: gainscenario.xsd"<<std::endl;
  std::cout<<std::endl;
}
