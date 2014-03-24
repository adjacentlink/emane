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

#include <cstdlib>
#include <iostream>

#include <libxml/parser.h>
#include <libxml/xmlschemas.h>

#include "antennaprofilemanifest.h"
#include "emane/utils/parameterconvert.h"

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
  std::string sSchema{"profilescenario.xsd"};
  
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
      sManifest = argv[iIndex];
    }
  else
    {
      std::cerr<<"missing manifest"<<std::endl;
      return EXIT_FAILURE;
    }

  ++iIndex;
  
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
      EMANE::AntennaProfileManifest::instance()->load(sManifest);


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
                          if(!xmlStrcmp(pActionNode->name,BAD_CAST "gain"))
                            {
                              ++iActionIndex;
                              
                              xmlChar * pProfileId = xmlGetProp(pActionNode,BAD_CAST "id");
                              
                              auto profileId =
                                EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pProfileId)).toUINT16();
                                          
                              xmlFree(pProfileId);
                              
                              xmlChar * pBearing = xmlGetProp(pActionNode,BAD_CAST "bearing");
                              
                              auto bearing =
                                EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pBearing)).toUINT16();
                              
                              xmlFree(pBearing);
                              
                              xmlChar * pElevation = xmlGetProp(pActionNode,BAD_CAST "elevation");
                              
                              auto elevation =
                                EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pElevation)).toINT16();
                              
                              xmlFree(pElevation); 
                              
                              //profiles.push_back({nemId,profileId,azimuth,elevation});
                              
                              std::cout<<"["<<iActionIndex<<"] gain  profileId: "<<profileId<<" bearing: "<<bearing<<" elevation: "<<elevation<<std::endl;

                              auto ret = EMANE::AntennaProfileManifest::instance()->getProfileInfo(profileId);

                              if(ret.second)
                                {
                                  auto pAntennaPattern = std::get<0>(ret.first);
                                  
                                  if(pAntennaPattern)
                                    {
                                      std::cout<<"  gain: "<<pAntennaPattern->getGain(bearing,elevation)<<std::endl;
                                    }
                                  else
                                    {
                                      std::cout<<"  gain: N/A"<<std::endl;
                                    }
                                }
                              else
                                {
                                  std::cout<<"  gain: Bad profile id"<<std::endl;
                                }

                              std::cout<<std::endl;
                            }
                          else if(!xmlStrcmp(pActionNode->name,BAD_CAST "blockage"))
                            {
                              ++iActionIndex;
                              
                              xmlChar * pProfileId = xmlGetProp(pActionNode,BAD_CAST "id");
                              
                              auto profileId =
                                EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pProfileId)).toUINT16();
                                          
                              xmlFree(pProfileId);
                              
                              xmlChar * pBearing = xmlGetProp(pActionNode,BAD_CAST "bearing");
                              
                              auto bearing =
                                EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pBearing)).toUINT16();
                              
                              xmlFree(pBearing);
                              
                              xmlChar * pElevation = xmlGetProp(pActionNode,BAD_CAST "elevation");
                              
                              auto elevation =
                                EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pElevation)).toINT16();
                              
                              xmlFree(pElevation); 
                              
                              std::cout<<"["<<iActionIndex<<"] blockage  profileId: "<<profileId<<" bearing: "<<bearing<<" elevation: "<<elevation<<std::endl;

                              auto ret = EMANE::AntennaProfileManifest::instance()->getProfileInfo(profileId);

                              if(ret.second)
                                {
                                  auto pBlockagePattern = std::get<1>(ret.first);
                                  
                                  if(pBlockagePattern)
                                    {
                                      std::cout<<"  blockage: "<<pBlockagePattern->getGain(bearing,elevation)<<std::endl;
                                    }
                                  else
                                    {
                                      std::cout<<"  blockage: N/A"<<std::endl;
                                    }
                                }
                              else
                                {
                                  std::cout<<"  blockage: Bad profile id"<<std::endl;
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
  std::cout<<"usage: profilescenario [OPTIONS]... MANIFESTXML SCENARIOXML"<<std::endl;
  std::cout<<std::endl;
  std::cout<<"options:"<<std::endl;
  std::cout<<"  -h, --help                     Print this message and exit."<<std::endl;
  std::cout<<"  -s, --schema                   Scenario schema"<<std::endl;
  std::cout<<"                                   Default: profilescenario.xsd"<<std::endl;
  std::cout<<std::endl;
}
