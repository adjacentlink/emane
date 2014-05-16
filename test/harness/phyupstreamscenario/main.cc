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

#include <iostream>
#include <cstdlib>

#include "platformservice.h"
#include "buildidservice.h"
#include "configurationservice.h"
#include "registrarproxy.h"
#include "eventservice.h"
#include "frameworkphy.h"
#include "antennaprofilemanifest.h"
#include "statisticservice.h"

#include "emane/configurationupdate.h"
#include "emane/utils/parameterconvert.h"
#include "emane/utils/spectrumwindowutils.h"
#include "emane/events/locationevent.h"
#include "emane/events/pathlossevent.h"
#include "emane/events/antennaprofileevent.h"

#include "emane/controls/frequencycontrolmessageformatter.h"
#include "emane/controls/receivepropertiescontrolmessageformatter.h"

#include "emane/controls/frequencyofinterestcontrolmessage.h"

#include "emane/utils/spectrumwindowutils.h"

#include <libxml/parser.h>
#include <libxml/xmlschemas.h>
#include <iomanip>
#include <ace/Get_Opt.h>

namespace
{
  class HarnessUpstreamTransport : public EMANE::UpstreamTransport
  {
  public:
    void setAction(int iAction)
    {
      iAction_ = iAction;
    }

    std::size_t getTotalProcessed() const
    {
      return totalProcessed_;
    }

    void processUpstreamPacket(EMANE::UpstreamPacket &, 
                               const EMANE::ControlMessages & msgs) override
    {
      std::cout<<"["<<iAction_<<"]  Packet forwarded to next layer"<<std::endl<<std::endl;;
      ++totalProcessed_;

      for(const auto & pControlMessage : msgs)
        {
          switch(pControlMessage->getId())
            {
            case EMANE::Controls::FrequencyControlMessage::IDENTIFIER:
              {
                const auto pFrequencyControlMessage =
                  static_cast<const EMANE::Controls::FrequencyControlMessage *>(pControlMessage); 

                std::cout<<"["<<iAction_<<"]  MAC FrequencyControlMessage data:"<<std::endl;

                for(const auto & sLine : EMANE::Controls::FrequencyControlMessageFormatter{pFrequencyControlMessage}())
                  {
                    std::cout<<"["<<iAction_<<"]   "<<sLine<<std::endl;
                  }
                std::cout<<std::endl;
              }
              
              break;

            case EMANE::Controls::ReceivePropertiesControlMessage::IDENTIFIER:
           {
             const auto pReceivePropertiesControlMessage =
               static_cast<const EMANE::Controls::ReceivePropertiesControlMessage *>(pControlMessage); 

             std::cout<<"["<<iAction_<<"]  MAC ReceivePropertiesControlMessage data:"<<std::endl;
             for(const auto & sLine : EMANE::Controls::ReceivePropertiesControlMessageFormatter{pReceivePropertiesControlMessage}())
               {
                 std::cout<<"["<<iAction_<<"]   "<<sLine<<std::endl;
               }
             std::cout<<std::endl;
           }

           break;
           
         default:
           std::cout<<"Unknown control message id: "<<pControlMessage->getId()<<std::endl;
           break;
            }
        }
      
      std::cout<<"Processing upstream packet..."<<std::endl;
    }

    void processUpstreamControl(const EMANE::ControlMessages &) override
    {
    }

  private:
    std::size_t totalProcessed_ = 0;
    int iAction_ = 0;
  };
}

void usage();

EMANE::FrameworkPHY * createPHY(EMANE::NEMId id,EMANE::SpectrumMonitor * pSpectrumMonitor);


int main(int argc, char * argv[])
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

  if(cmd_opts.long_option(ACE_TEXT("profiles"),'p',ACE_Get_Opt::ARG_REQUIRED) == -1)
    {
      ACE_ERROR_RETURN((LM_ERROR,ACE_TEXT("%s\n"),
                        ACE_TEXT("config long option: profiles")),EXIT_FAILURE);
    }
  
  int iOption{};
  std::string sScenario{};
  std::string sManifest{};
  std::string sSchema{"gainscenario.xsd"};
  std::string sProfiles{};

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

        case 'p':
          // --profiles
          sProfiles = cmd_opts.opt_arg();
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
      if(!sProfiles.empty())
        {
          EMANE::AntennaProfileManifest::instance()->load(sProfiles);
        }
     

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

      EMANE::SpectrumMonitor spectrumMonitor{};

      auto pPHYLayer = createPHY(id,&spectrumMonitor);
 
      HarnessUpstreamTransport harnessUpstreamTransport;
      
      pPHYLayer->setUpstreamTransport(&harnessUpstreamTransport);

      bool bRunning{};
      int iActionIndex{};
      std::uint16_t u16SequenceNumber{};

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
                          if(!xmlStrcmp(pActionNode->name,BAD_CAST "configure"))
                            {
                              EMANE::ConfigurationUpdateRequest request;
                         
                              ++iActionIndex;

                              for(xmlNodePtr pParamListNode = pActionNode->children;
                                  pParamListNode;
                                  pParamListNode = pParamListNode->next)
                                {
                                  if(pParamListNode->type == XML_ELEMENT_NODE)
                                    {
                                      if(!xmlStrcmp(pParamListNode->name,BAD_CAST "paramlist"))
                                        {
                                          EMANE::ConfigurationNameStringValues values;
                                     
                                          xmlChar * pName = xmlGetProp(pParamListNode,BAD_CAST "name");
                                     
                                          values.first = reinterpret_cast<const char *>(pName);

                                          xmlFree(pName);
                                     

                                          for(xmlNodePtr pItemNode = pParamListNode->children;
                                              pItemNode;
                                              pItemNode = pItemNode->next)
                                            {
                                              if(pItemNode->type == XML_ELEMENT_NODE)
                                                {
                                                  if(!xmlStrcmp(pItemNode->name,BAD_CAST "item"))
                                                    {
                                                      xmlChar * pValue = xmlGetProp(pItemNode,BAD_CAST "value");
                                                  
                                                      values.second.push_back(reinterpret_cast<const char *>(pValue));

                                                      xmlFree(pValue);
                                                    }
                                                }
                                            }

                                          request.push_back(values);
                                        }
                                    }
                                }
                         
                              if(!bRunning)
                                {
                                  pPHYLayer->configure(EMANE::ConfigurationServiceSingleton::instance()->buildUpdates(pPHYLayer->getBuildId(),
                                                                                                                      request));
                                  pPHYLayer->start();
      
                                  bRunning = true;
                                  
                                }
                              else
                                {
                                  pPHYLayer->processConfiguration(EMANE::ConfigurationServiceSingleton::instance()->buildUpdates(pPHYLayer->getBuildId(),
                                                                                                                                 request));
                                }

                              std::cout<<"["<<iActionIndex<<"] configure "<<std::endl;
                              
                              auto results = EMANE::ConfigurationServiceSingleton::instance()->queryConfiguration(pPHYLayer->getBuildId());
                              
                              for(const auto & entry : results)
                                {
                                  std::cout<<"["<<iActionIndex<<"]  "<<entry.first<<":"<<std::endl;
                                  
                                  for(const auto & value :  entry.second)
                                    {
                                      std::cout<<"["<<iActionIndex<<"]    "<<value.toString()<<std::endl;
                                    }
                                }
                              
                              std::cout<<std::endl;
                            }
                          else if(!xmlStrcmp(pActionNode->name,BAD_CAST "locations"))
                            {
                              if(!bRunning)
                                {
                                  std::cerr<<"Scenario error: configure must be first action";
                                  return EXIT_FAILURE;
                                }

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

                              pPHYLayer->processEvent(EMANE::Events::LocationEvent::IDENTIFIER,
                                                      EMANE::Events::LocationEvent{locations}.serialize());

                              std::cout<<std::endl;
                            }
                          else if(!xmlStrcmp(pActionNode->name,BAD_CAST "antennaprofiles"))
                            {
                              if(!bRunning)
                                {
                                  std::cerr<<"Scenario error: configure must be first action";
                                  return EXIT_FAILURE;
                                }


                              EMANE::AntennaProfiles profiles;
                              ++iActionIndex;
                              
                              for(xmlNodePtr pProfileNode = pActionNode->children;
                                  pProfileNode;
                                  pProfileNode = pProfileNode->next)
                                {
                                  if(pProfileNode->type == XML_ELEMENT_NODE)
                                    {
                                      if(!xmlStrcmp(pProfileNode->name,BAD_CAST "profile"))
                                        {
                                          xmlChar * pNEMId = xmlGetProp(pProfileNode,BAD_CAST "nem");
                                          
                                          auto nemId =
                                            EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pNEMId)).toUINT16();
                                          
                                          xmlFree(pNEMId);

                                          xmlChar * pProfileId = xmlGetProp(pProfileNode,BAD_CAST "id");
                                          
                                          auto profileId =
                                            EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pProfileId)).toUINT16();
                                          
                                          xmlFree(pProfileId);

                                          xmlChar * pAzimuth = xmlGetProp(pProfileNode,BAD_CAST "azimuth");
                                                       
                                          auto azimuth =
                                            EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pAzimuth)).toDouble();
                                          
                                          xmlFree(pAzimuth);
                                          
                                          xmlChar * pElevation = xmlGetProp(pProfileNode,BAD_CAST "elevation");
                                          
                                          auto elevation =
                                            EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pElevation)).toDouble();
                                          
                                          xmlFree(pElevation); 

                                          profiles.push_back({nemId,profileId,azimuth,elevation});

                                          std::cout<<"["<<iActionIndex<<"] profile "<<nemId<<" profileId: "<<profileId<<" azimuth: "<<azimuth<<" elevation: "<<elevation<<std::endl;
                                        }
                                    }
                                }

                                                 
                              pPHYLayer->processEvent(EMANE::Events::AntennaProfileEvent::IDENTIFIER,
                                                      EMANE::Events::AntennaProfileEvent{profiles}.serialize());

                              std::cout<<std::endl;                              
                            }
                          else if(!xmlStrcmp(pActionNode->name,BAD_CAST "pathloss"))
                            {
                              if(!bRunning)
                                {
                                  std::cerr<<"Scenario error: configure must be first action";
                                  return EXIT_FAILURE;
                                }

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

                              pPHYLayer->processEvent(EMANE::Events::PathlossEvent::IDENTIFIER,
                                                      EMANE::Events::PathlossEvent{pathlosses}.serialize());

                              std::cout<<std::endl;
                            }
                          else if(!xmlStrcmp(pActionNode->name,BAD_CAST "packet"))
                            {
                              ++iActionIndex;

                              std::cout<<"["<<iActionIndex<<"] packet"<<std::endl;

                              if(!bRunning)
                                {
                                  std::cerr<<"Scenario error: configure must be first action";
                                  return EXIT_FAILURE;
                                }

                              // what would get gettimeofday() in running code
                              xmlChar * pNow = xmlGetProp(pActionNode,BAD_CAST "now");
              
                              auto now = 
                                EMANE::Microseconds{EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pNow)).toUINT64()};
                           
                              xmlFree(pNow);

                              // actual tx time presumed to be <= 'now'
                              xmlChar * pTxTime = xmlGetProp(pActionNode,BAD_CAST "txtime");
                           
                              auto txTime =
                                EMANE::Microseconds{EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pTxTime)).toUINT64()};
                         
                              xmlFree(pTxTime);


                              xmlChar * pDestination = xmlGetProp(pActionNode,BAD_CAST "destination");

                              auto destination =
                                EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pDestination)).toUINT16();
                                          
                              xmlFree(pDestination);

                              xmlChar * pSource = xmlGetProp(pActionNode,BAD_CAST "source");
                                          
                              auto source =
                                EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pSource)).toUINT16();
                                          
                              xmlFree(pSource);


                              xmlChar * pSize = xmlGetProp(pActionNode,BAD_CAST "size");
                                          
                              auto size =
                                EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pSize)).toUINT16();
                                          
                              xmlFree(pSize);

                              // actual tx time presumed to be <= 'now'
                              xmlChar * pBandwidth = xmlGetProp(pActionNode,BAD_CAST "bandwidth");
                           
                              auto bandwidth =
                                EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pBandwidth)).toUINT64();
                         
                              xmlFree(pBandwidth);
                         
                              double dTxFixedGain{};
                              bool bTxHasFixedGain{false};
                         
                              xmlChar * pTxFixedGain = xmlGetProp(pActionNode,BAD_CAST "fixedgain");
                         
                              if(pTxFixedGain)
                                {
                                  dTxFixedGain =
                                    EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pTxFixedGain)).toDouble();
                             
                                  xmlFree(pTxFixedGain);
                             
                                  bTxHasFixedGain = true;
                                }

                              xmlChar * pSubId = xmlGetProp(pActionNode,BAD_CAST "subid");
                                          
                              auto subId =
                                EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pSubId)).toUINT16();
                                          
                              xmlFree(pSubId);

                              EMANE::FrequencySegments segments;
                              EMANE::Transmitters transmitters;

                              for(xmlNodePtr pChildNode = pActionNode->children;
                                  pChildNode;
                                  pChildNode = pChildNode->next)
                                {
                                  if(pChildNode->type == XML_ELEMENT_NODE)
                                    {
                                      if(!xmlStrcmp(pChildNode->name,BAD_CAST "segments"))
                                        {
                                          for(xmlNodePtr pSegmentNode = pChildNode->children;
                                              pSegmentNode;
                                              pSegmentNode = pSegmentNode->next)
                                            {
                                              if(pSegmentNode->type == XML_ELEMENT_NODE)
                                                {
                                                  if(!xmlStrcmp(pSegmentNode->name,BAD_CAST "segment"))
                                                    {
                                                      xmlChar * pFrequency = xmlGetProp(pSegmentNode,BAD_CAST "frequency");
                                                 
                                                      auto frequency = 
                                                        EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pFrequency)).toUINT64();
                                                 
                                                      xmlFree(pFrequency);
                                                 
                                                      xmlChar * pOffset = xmlGetProp(pSegmentNode,BAD_CAST "offset");
                                                 
                                                      auto offset = 
                                                        EMANE::Microseconds{EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pOffset)).toUINT64()};
                                                 
                                                      xmlFree(pOffset);
                                                 
                                                      xmlChar * pDuration = xmlGetProp(pSegmentNode,BAD_CAST "duration");
                                                 
                                                      auto duration =
                                                        EMANE::Microseconds{EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pDuration)).toUINT64()};
                                                 
                                                      xmlFree(pDuration);
                                                 
                                                      segments.push_back({frequency,duration,offset});
                                                    }
                                                }
                                            }
                                        }
                                      else if(!xmlStrcmp(pChildNode->name,BAD_CAST "transmitters"))
                                        {
                                          for(xmlNodePtr pTransmitterNode = pChildNode->children;
                                              pTransmitterNode;
                                              pTransmitterNode = pTransmitterNode->next)
                                            {
                                              if(pTransmitterNode->type == XML_ELEMENT_NODE)
                                                {
                                                  if(!xmlStrcmp(pTransmitterNode->name,BAD_CAST "transmitter"))
                                                    {
                                                      xmlChar * pTransmitter = xmlGetProp(pTransmitterNode,BAD_CAST "nem");
                                                      
                                                      auto transmitter = 
                                                        EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pTransmitter)).toUINT16();
                                                      
                                                      xmlFree(pTransmitter);
                                                      
                                                      xmlChar * pTxPowerdBm = xmlGetProp(pTransmitterNode,BAD_CAST "power");
                                                      
                                                      auto txPowerdBm = 
                                                        EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pTxPowerdBm)).toDouble();
                                                      
                                                      xmlFree(pTxPowerdBm);
                                                      
                                                      transmitters.push_back({transmitter,txPowerdBm});
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                              
                              std::vector<std::uint8_t> data(size,0);

                              EMANE::UpstreamPacket pkt{{source,destination,0,EMANE::TimePoint{now}},&data[0],size};


                              harnessUpstreamTransport.setAction(iActionIndex);

                              std::size_t currentTotalProccessed = harnessUpstreamTransport.getTotalProcessed();

                              
                              EMANE::CommonPHYHeader hdr{EMANE::REGISTERED_EMANE_PHY_FRAMEWORK,
                                  subId,
                                  ++u16SequenceNumber,
                                  bandwidth,
                                  EMANE::TimePoint{txTime},
                                  segments,
                                    transmitters,
                                      {dTxFixedGain,bTxHasFixedGain}};
                              
                              std::cout<<"["<<iActionIndex<<"]  Common PHY Header data:"<<std::endl;

                              for(const auto & sLine : hdr.format())
                                {
                                  std::cout<<"["<<iActionIndex<<"]   "<<sLine<<std::endl;
                                }
                               
                              std::cout<<std::endl;

                              pPHYLayer->processUpstreamPacket_i(EMANE::TimePoint{now},
                                                                 hdr,
                                                                 pkt,
                                                                 {});


                              if(currentTotalProccessed == harnessUpstreamTransport.getTotalProcessed())
                                {
                                  std::cout<<"["<<iActionIndex<<"]  Packet dropped"<<std::endl<<std::endl;

                                  auto results = 
                                    EMANE::StatisticService::instance()->queryTable(pPHYLayer->getBuildId(),
                                                                                    {"UnicastPacketDropTable0",
                                                                                        "BroadcastPacketDropTable0"});
                                  

                                  for(const auto entry : results)
                                    {
                                      std::cout<<"["<<iActionIndex<<"] "<<entry.first<<std::endl;
                                      std::vector<int> labelLengths;
                                      std::cout<<"["<<iActionIndex<<"] "<<'|';
                                      for(const auto & label : entry.second.first)
                                        {
                                          labelLengths.push_back(label.size());
                                          std::cout<<std::setiosflags(ios::left) <<std::setw(label.size())<<label<<"|";
                                        }
                                      
                                      std::cout<<std::endl;
                                      
                                      for(const auto & row : entry.second.second)
                                        {
                                          int i{};
                                          std::cout<<"["<<iActionIndex<<"] "<<'|';
                                          for(const auto & any : row)
                                            {
                                              std::cout<<std::setiosflags(ios::left) <<std::setw(labelLengths[i++])<<any.toString()<<"|";
                                            }

                                          std::cout<<std::endl;
                                        }
                                    }
                                  std::cout<<std::endl;
                                }
                            }
                          else if(!xmlStrcmp(pActionNode->name,BAD_CAST "window"))
                            {
                              ++iActionIndex;

                              std::cout<<"["<<iActionIndex<<"] window"<<std::endl;
                              
                              // what would get gettimeofday() in running code
                              xmlChar * pNow = xmlGetProp(pActionNode,BAD_CAST "now");
              
                              auto now = 
                                EMANE::Microseconds{EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pNow)).toUINT64()};
                           
                              xmlFree(pNow);
                                                      
                              xmlChar * pFrequency = xmlGetProp(pActionNode,BAD_CAST "frequency");
                              
                              auto frequency = EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pFrequency)).toUINT64();
                           
                              xmlFree(pFrequency);

                              EMANE::TimePoint timepoint = EMANE::TimePoint::min();
                              
                              // what would get gettimeofday() in running code<
                              xmlChar * pTime = xmlGetProp(pActionNode,BAD_CAST "time");
                              
                              if(pTime)
                                {
                                  timepoint = 
                                    EMANE::TimePoint{EMANE::Microseconds{EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pTime)).toUINT64()}};
                           
                                  xmlFree(pTime);
                                }

                              EMANE::Microseconds duration{0};
                              
                              xmlChar * pDuration = xmlGetProp(pActionNode,BAD_CAST "duration");
              
                              if(pDuration)
                                {
                                  duration = 
                                    EMANE::Microseconds{EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pDuration)).toUINT64()};
                                  
                                  xmlFree(pDuration);
                                }


                              try
                                {
                                  std::vector<double> bins{};
                                  EMANE::TimePoint startOfBinTime{};
                                  EMANE::Microseconds binSize{};
                                  double dReceiverSensitivityMilliWatt{};
                                  bool bSignalInNoise{};
                              
                                  auto window = spectrumMonitor.request_i(EMANE::TimePoint{now},frequency,duration,timepoint);

                                  std::tie(bins,startOfBinTime,binSize,dReceiverSensitivityMilliWatt,bSignalInNoise) = window;
                                    

                                  std::cout<<"["
                                           <<iActionIndex
                                           <<"]  request window time: "
                                           <<std::chrono::duration_cast<EMANE::Microseconds>(timepoint.time_since_epoch()).count()
                                           <<std::endl;

                                  std::cout<<"["
                                           <<iActionIndex
                                           <<"]  frequency: "
                                           <<frequency
                                           <<std::endl;

                                  std::cout<<"["
                                           <<iActionIndex
                                           <<"]  window response timepoint: "
                                           <<std::chrono::duration_cast<EMANE::Microseconds>(startOfBinTime.time_since_epoch()).count()
                                           <<std::endl;

                                  std::cout<<"["
                                           <<iActionIndex
                                           <<"]  bin size: "
                                           << binSize.count()
                                           <<std::endl;
                                  
                                  std::cout<<"["
                                           <<iActionIndex
                                           <<"]  rx sensitivity: "
                                           << dReceiverSensitivityMilliWatt
                                           <<std::endl;

                                  std::cout<<"["
                                           <<iActionIndex
                                           <<"]  signal in noise: "
                                           << (bSignalInNoise ? "yes" : "no")
                                           <<std::endl;
                                      
                                  for(const auto & entry : EMANE::Utils::spectrumCompress(bins))
                                    {
                                      std::cout<<"["
                                               <<iActionIndex
                                               <<"]    "
                                               <<entry.first
                                               <<":"
                                               <<entry.second<<std::endl;
                                    }
                                 

                                  for(xmlNodePtr pMaxBinNoiseFloorNode = pActionNode->children;
                                      pMaxBinNoiseFloorNode;
                                      pMaxBinNoiseFloorNode = pMaxBinNoiseFloorNode->next)
                                    {
                                      if(pMaxBinNoiseFloorNode->type == XML_ELEMENT_NODE)
                                        {
                                          if(!xmlStrcmp(pMaxBinNoiseFloorNode->name,BAD_CAST "maxbinnoisefloor"))
                                            {
                                              xmlChar * pPowerdBm = xmlGetProp(pMaxBinNoiseFloorNode,BAD_CAST "power");

                                              auto powerdBm =
                                                EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pPowerdBm)).toDouble();

                                              xmlFree(pPowerdBm);

                                              std::cout<<"["
                                                       <<iActionIndex
                                                       <<"]  Power (dBm): "
                                                       << powerdBm
                                                       <<std::endl;
                                              
                                              xmlChar * pStartTime = xmlGetProp(pMaxBinNoiseFloorNode,BAD_CAST "starttime");
                           
                                              if(pStartTime)
                                                {
                                                  auto startTime =
                                                    EMANE::Microseconds{EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pStartTime)).toUINT64()};
                                                  
                                                  xmlFree(pStartTime);

                                                  std::cout<<"["
                                                           <<iActionIndex
                                                           <<"]  max bin noise floor start time: "
                                                           << startTime.count()
                                                           <<std::endl;


                                                  xmlChar * pEndTime = xmlGetProp(pMaxBinNoiseFloorNode,BAD_CAST "endtime");
                                                  
                                                  if(pEndTime)
                                                    {
                                                      auto endTime =
                                                        EMANE::Microseconds{EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pEndTime)).toUINT64()};
                                                      
                                                      xmlFree(pEndTime);
                                                      
                                                      std::cout<<"["
                                                               <<iActionIndex
                                                               <<"]  max bin noise floor end time: "
                                                               << endTime.count()
                                                               <<std::endl;
                                                       
                                                      auto ret = EMANE::Utils::maxBinNoiseFloorRange(window,powerdBm,EMANE::TimePoint{startTime},EMANE::TimePoint{endTime});

                                                       std::cout<<"["
                                                                <<iActionIndex
                                                                <<"]  noise floor (dBm): "
                                                                << ret.first
                                                                << " (mW):  "
                                                                << EMANE::Utils::DB_TO_MILLIWATT(ret.first)
                                                                <<std::endl;


                                                       std::cout<<"["
                                                                <<iActionIndex
                                                                <<"]  signal in noise: "
                                                                << (ret.second ? "yes" : "no")
                                                                <<std::endl;

                                                    }
                                                  else
                                                    {
                                                      auto ret = EMANE::Utils::maxBinNoiseFloor(window,powerdBm,EMANE::TimePoint{startTime});

                                                      std::cout<<"["
                                                               <<iActionIndex
                                                               <<"]  noise floor (dBm): "
                                                               << ret.first
                                                               << " (mW):  "
                                                               << EMANE::Utils::DB_TO_MILLIWATT(ret.first)
                                                               <<std::endl;

                                                      std::cout<<"["
                                                               <<iActionIndex
                                                               <<"]  signal in noise: "
                                                               << (ret.second ? "yes" : "no")
                                                               <<std::endl;
                                                    }
                                                }
                                              else
                                                {
                                                  auto ret = EMANE::Utils::maxBinNoiseFloor(window,powerdBm);

                                                  std::cout<<"["
                                                           <<iActionIndex
                                                           <<"]  noise floor (dBm): "
                                                           << ret.first
                                                           << " (mW):  "
                                                           << EMANE::Utils::DB_TO_MILLIWATT(ret.first)
                                                           <<std::endl;

                                                  std::cout<<"["
                                                           <<iActionIndex
                                                           <<"]  signal in noise: "
                                                           << (ret.second ? "yes" : "no")
                                                           <<std::endl;
                                                  
                                                }
                                            }
                                        }
                                    }

                                  std::cout<<std::endl;
                                }
                              catch(...)
                                {
                                  std::cerr<<"error occurred while procssing request at "<<now.count()<<std::endl;
                                  throw;
                                }
                            }
                          else if(!xmlStrcmp(pActionNode->name,BAD_CAST "frequencyofinterest"))
                            {
                              xmlChar * pBandwidth = xmlGetProp(pActionNode,BAD_CAST "bandwidth");
                           
                              auto bandwidth = 
                                EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pBandwidth)).toUINT64();
                           
                              xmlFree(pBandwidth);

                              EMANE::FrequencySet foi{};

                              for(xmlNodePtr pChildNode = pActionNode->children; pChildNode; pChildNode = pChildNode->next)
                                {
                                  if(pChildNode->type == XML_ELEMENT_NODE)
                                    {
                                      if(!xmlStrcmp(pChildNode->name,BAD_CAST "frequency"))
                                        {
                                          xmlChar * pFrequency = xmlGetProp(pChildNode,BAD_CAST "value");
                                       
                                          foi.insert(EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pFrequency)).toUINT64());
                                       
                                          xmlFree(pFrequency);
                                        }
                                    }
                                }

                              std::cout<<"["<<++iActionIndex
                                       <<"] frequencyofinterest "
                                       <<std::endl;

                              std::cout<<"["<<iActionIndex
                                       <<"]    bandwidth: "
                                       <<bandwidth
                                       <<std::endl;

                              for(const auto & freqHz : foi)
                                {
                                  std::cout<<"["<<iActionIndex
                                           <<"]    frequency: "
                                           <<freqHz
                                           <<std::endl;
                                }

                              auto pFrequencyOfInterestControlMessage = 
                                EMANE::Controls::FrequencyOfInterestControlMessage::create(bandwidth,foi);

                              pPHYLayer->processDownstreamControl({pFrequencyOfInterestControlMessage});

                              std::cout<<std::endl;

                            }

                          //XXX
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
  std::cout<<"usage: phyupstreamscenario [OPTIONS]... MANIFESTXML SCENARIOXML"<<std::endl;
  std::cout<<std::endl;
  std::cout<<"options:"<<std::endl;
  std::cout<<"  -h, --help                     Print this message and exit."<<std::endl;
  std::cout<<"  -s, --schema                   Scenario schema"<<std::endl;
  std::cout<<"                                   Default: phyupstreamscenario.xsd"<<std::endl;
  std::cout<<"  -p, --profiles PROFILEXML      Antenna profile XML"<<std::endl;
  std::cout<<std::endl;
}

EMANE::FrameworkPHY * createPHY(EMANE::NEMId id, EMANE::SpectrumMonitor * pSpectrumMonitor)
{
  EMANE::PlatformService * pPlatformService{new EMANE::PlatformService{}};
  
  EMANE::FrameworkPHY * pPHYLayer{new EMANE::FrameworkPHY{id, pPlatformService,pSpectrumMonitor}};
  
  EMANE::BuildId buildId{EMANE::BuildIdServiceSingleton::instance()->registerBuildable(pPHYLayer,
                                                                                       EMANE::COMPONENT_PHYILAYER,
                                                                                       "")};
  
  EMANE::ConfigurationServiceSingleton::instance()->registerRunningStateMutable(buildId,
                                                                                pPHYLayer);
  
  pPlatformService->setPlatformServiceUser(buildId,pPHYLayer);
   
  // register event service handler with event service
  EMANE::EventServiceSingleton::instance()->registerEventServiceUser(buildId,
                                                                     pPHYLayer,
                                                                     id);
   
  EMANE::RegistrarProxy registrarProxy{buildId};
   
  // initialize
  pPHYLayer->initialize(registrarProxy);
   
  return pPHYLayer;
}
