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
#include <chrono>
#include <fstream>
#include <libxml/parser.h>
#include <libxml/xmlschemas.h>
#include <numeric>

#include "emane/utils/parameterconvert.h"
#include "emane/utils/spectrumwindowutils.h"

#include "spectrummonitor.h"

#include <ace/Get_Opt.h>

void usage();


// simple test driver that loads an scenrio XML file and outputs the entire
// wheel after each update (packet processed)
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

  int iOption{};
  std::string sScenario{};
  std::string sSchema{"noisescenario.xsd"};
  
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

  if(cmd_opts.opt_ind() < cmd_opts.argc())
    {
      sScenario = argv[cmd_opts.opt_ind()];
    }
  else
    {
      std::cerr<<"missing scenario"<<std::endl;
      return EXIT_FAILURE;
    }

      
  EMANE::SpectrumMonitor spectrumMonitor{};
      
  EMANE::FrequencySet foi;
      
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

  // absoulute start time that allows you to define the rest of the scenario relative to this time
  // 0 is the epoch
  xmlChar * pStart = xmlGetProp(pRoot,BAD_CAST "start");
  
  auto start =
    EMANE::Microseconds{EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pStart)).toUINT64()};
  
  xmlFree(pStart);

  using TimePoint = std::chrono::high_resolution_clock::time_point;

  int iActionIndex{};

  for(xmlNodePtr pNode = pRoot->children; pNode; pNode = pNode->next)
    {
      if(pNode->type == XML_ELEMENT_NODE)
        {
          if(!xmlStrcmp(pNode->name,BAD_CAST "action"))
            {
              for(xmlNodePtr pActionNode = pNode->children; pActionNode; pActionNode = pActionNode->next)
                {
                  if(pActionNode->type == XML_ELEMENT_NODE)
                    {
                      if(!xmlStrcmp(pActionNode->name,BAD_CAST "initialize"))
                        {
                          xmlChar * pBinDuration = xmlGetProp(pActionNode,BAD_CAST "binduration");
                              
                          auto binDuration =
                            EMANE::Microseconds{EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pBinDuration)).toUINT64()};
                                       
                          xmlFree(pBinDuration);

                          xmlChar * pMaxSegmentOffset = xmlGetProp(pActionNode,BAD_CAST "maxsegmentoffset");
                              
                          auto maxSegmentOffset =
                            EMANE::Microseconds{EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pMaxSegmentOffset)).toUINT64()};
                                       
                          xmlFree(pMaxSegmentOffset);

                          xmlChar * pMaxSegmentDuration = xmlGetProp(pActionNode,BAD_CAST "maxsegmentduration");
                              
                          auto maxSegmentDuration =
                            EMANE::Microseconds{EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pMaxSegmentDuration)).toUINT64()};
                                       
                          xmlFree(pMaxSegmentDuration);

                          xmlChar * pMaxMessagePropagation = xmlGetProp(pActionNode,BAD_CAST "maxmessagepropagation");
                              
                          auto maxMessagePropagation =
                            EMANE::Microseconds{EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pMaxMessagePropagation)).toUINT64()};
                                       
                          xmlFree(pMaxMessagePropagation);

                          xmlChar * pTimeSyncThreshold = xmlGetProp(pActionNode,BAD_CAST "timesyncthreshold");
                              
                          auto timeSyncThreshold =
                            EMANE::Microseconds{EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pTimeSyncThreshold)).toUINT64()};
                                       
                          xmlFree(pTimeSyncThreshold);

                          xmlChar * pBandwidth = xmlGetProp(pActionNode,BAD_CAST "bandwidth");
                           
                          auto bandwidth = 
                            EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pBandwidth)).toUINT64();
                           
                          xmlFree(pBandwidth);

                          xmlChar * pNoiseMode = xmlGetProp(pActionNode,BAD_CAST "mode");

                          EMANE::SpectrumMonitor::NoiseMode mode{};
                              
                          if(!xmlStrcmp(pNoiseMode,BAD_CAST "none"))
                            {
                              mode = EMANE::SpectrumMonitor::NoiseMode::NONE;
                            }
                          else if(!xmlStrcmp(pNoiseMode,BAD_CAST "all"))
                            {
                              mode = EMANE::SpectrumMonitor::NoiseMode::ALL;
                            }
                          else if(!xmlStrcmp(pNoiseMode,BAD_CAST "outofband"))
                            {
                              mode = EMANE::SpectrumMonitor::NoiseMode::OUTOFBAND;
                            }
                          else
                            {
                              std::cerr<<"unknown noise mode"<<std::endl;
                              return EXIT_FAILURE;
                            }
                                

                          xmlFree(pNoiseMode);
                              
                          // message duration
                          xmlChar * pRxSensitivityMilliWatt = xmlGetProp(pActionNode,BAD_CAST "sensitivity");
                          
                          double dRxSensitivityMilliWatt  =
                            EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pRxSensitivityMilliWatt)).toDouble();
                              
                          xmlFree(pRxSensitivityMilliWatt);
                              
                          xmlChar * pClamp = xmlGetProp(pActionNode,BAD_CAST "clamp");

                          auto bClamp =
                            EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pClamp)).toBool();
                                       
                          xmlFree(pClamp);

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
                                       <<"] initialize "
                                       <<std::endl;

                             std::cout<<"["<<iActionIndex
                                      <<"]    rx sensitivity mW: "
                                      <<dRxSensitivityMilliWatt
                                      <<std::endl;

                             std::cout<<"["<<iActionIndex
                                      <<"]    bandwidth: "
                                      <<bandwidth
                                      <<std::endl;

                             std::cout<<"["<<iActionIndex
                                      <<"]    bin usec: "
                                      <<binDuration.count()
                                      <<std::endl;

                             std::cout<<"["<<iActionIndex
                                      <<"]    max offset usec: "
                                      <<maxSegmentOffset.count()
                                      <<std::endl;

                             std::cout<<"["<<iActionIndex
                                      <<"]    max propagation usec: "
                                      <<maxMessagePropagation.count()
                                      <<std::endl;

                             std::cout<<"["<<iActionIndex
                                      <<"]    max duration usec: "
                                      <<maxSegmentDuration.count()
                                      <<std::endl;

                             std::cout<<"["<<iActionIndex
                                      <<"]    time sync threshold: "
                                      <<timeSyncThreshold.count()
                                      <<std::endl;

                             std::cout<<"["<<iActionIndex
                                      <<"]    max clamp: "
                                      <<(bClamp ? "yes" : "no")
                                      <<std::endl;

                              std::cout<<"["<<iActionIndex
                                      <<"]    mode: "
                                       <<(mode == EMANE::SpectrumMonitor::NoiseMode::ALL ? "all" :
                                          (mode == EMANE::SpectrumMonitor::NoiseMode::NONE ? "none" : "outofband"))
                                      <<std::endl;
                              
                           
                              spectrumMonitor.initialize(foi,
                                                         bandwidth,
                                                         dRxSensitivityMilliWatt,
                                                         mode,
                                                         binDuration,
                                                         maxSegmentOffset,
                                                         maxMessagePropagation,
                                                         maxSegmentDuration,
                                                         timeSyncThreshold,
                                                         bClamp);
                          std::cout<<std::endl;

                        }
                      else if(!xmlStrcmp(pActionNode->name,BAD_CAST "update"))
                        {
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
                           
                           
                          // propagation delay
                          xmlChar * pPropagation = xmlGetProp(pActionNode,BAD_CAST "propagation");
                           
                          auto propagation = 
                            EMANE::Microseconds{EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pPropagation)).toUINT64()};
                           
                          xmlFree(pPropagation);

                          xmlChar * pTransmitterBandwidth = xmlGetProp(pActionNode,BAD_CAST "transmitterbandwidth");

                          auto transmitterBandwidth = 
                            EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pTransmitterBandwidth)).toUINT64();
                           
                          xmlFree(pTransmitterBandwidth);

                          xmlChar * pInBand = xmlGetProp(pActionNode,BAD_CAST "inband");


                          bool bInBand{};

                          bInBand =
                            EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pInBand)).toBool();
                                       
                          xmlFree(pInBand);


                          EMANE::FrequencySegments segments;
                          std::vector<double> powers;
                          std::vector<EMANE::NEMId> transmitters;

                          for(xmlNodePtr pChildNode = pActionNode->children; pChildNode; pChildNode = pChildNode->next)
                            {
                              if(pChildNode->type == XML_ELEMENT_NODE)
                                {
                                  if(!xmlStrcmp(pChildNode->name,BAD_CAST "segment"))
                                    {
                                      xmlChar * pFrequency = xmlGetProp(pChildNode,BAD_CAST "frequency");
                                       
                                      auto frequency = EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pFrequency)).toUINT64();
                                       
                                      xmlFree(pFrequency);
                                       
                                      xmlChar * pOffset = xmlGetProp(pChildNode,BAD_CAST "offset");
                                       
                                      auto offset = 
                                        EMANE::Microseconds{EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pOffset)).toUINT64()};

                                      xmlFree(pOffset);
                           
                                      // message duration
                                      xmlChar * pDuration = xmlGetProp(pChildNode,BAD_CAST "duration");
                                       
                                      auto duration =
                                        EMANE::Microseconds{EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pDuration)).toUINT64()};
                                       
                                      xmlFree(pDuration);
                                       
                                      segments.push_back({frequency,duration,offset});

                                      // message duration
                                      xmlChar * pRxPowerMilliWatt = xmlGetProp(pChildNode,BAD_CAST "rxpower");
                                       
                                      double dRxPowerMilliWatt  =
                                        EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pRxPowerMilliWatt)).toDouble();
                                       
                                      xmlFree(pRxPowerMilliWatt);

                                      powers.push_back(dRxPowerMilliWatt);
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
                                                  
                                                  transmitters.push_back(transmitter);
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                          
                          try
                            {
                              std::cout<<"["<<++iActionIndex
                                       <<"] update abs time: "
                                       <<std::chrono::duration_cast<EMANE::Microseconds>(TimePoint{start+now}.time_since_epoch()).count()
                                       <<" relative time: "
                                       <<std::chrono::duration_cast<EMANE::Microseconds>(now).count()
                                       <<std::endl;

                              std::cout<<"["<<iActionIndex
                                       <<"] propagation: "
                                       <<propagation.count()
                                       <<std::endl;

                              std::cout<<"["<<iActionIndex
                                       <<"] bandwidth: "
                                       <<transmitterBandwidth
                                       <<std::endl;
                              
                              std::cout<<"["<<iActionIndex
                                       <<"] in-band: "
                                       <<(bInBand ? "yes" : "no")
                                       <<std::endl;

                              int i = 0;
                              
                              for(const auto & segment : segments)
                                {
                                  std::cout<<"["<<iActionIndex
                                           <<"] frequency: "
                                           <<segment.getFrequencyHz()
                                           <<std::endl;

                                  std::cout<<"["<<iActionIndex
                                           <<"] offset: "
                                           <<segment.getOffset().count()
                                           <<std::endl;

                                  std::cout<<"["<<iActionIndex
                                           <<"] duration: "
                                           <<segment.getDuration().count()
                                           <<std::endl;

                                  std::cout<<"["<<iActionIndex
                                           <<"] power mW: "
                                           <<powers[i]
                                           <<std::endl;

                                  ++i;
                                }

                              for(const auto & transmitter : transmitters)
                                {
                                  std::cout<<"["<<iActionIndex
                                           <<"] transmitter: "
                                           <<transmitter
                                           <<std::endl;
                                }
                              
                              EMANE::TimePoint  bin0Time{};
                              EMANE::Microseconds reportablePropagation{};
                              EMANE::Microseconds span{};
                              EMANE::FrequencySegments reportableSegments{};
                              bool bTreatAsInBand{};

                              std::tie(bin0Time,
                                       reportablePropagation,
                                       span,
                                       reportableSegments,
                                       bTreatAsInBand)  = spectrumMonitor.update(TimePoint{start+now},
                                                                                 TimePoint{start+txTime},
                                                                                 propagation,
                                                                                 segments,
                                                                                 transmitterBandwidth,
                                                                                 powers,
                                                                                 bInBand,
                                                                                 transmitters);
                              

                              //std::tuple<TimePoint,Microseconds,Microseconds,FrequencySegments,bool>
                              std::cout<<"["<<iActionIndex
                                       <<"] reportable bin 0 time: "
                                       <<std::chrono::duration_cast<EMANE::Microseconds>(bin0Time.time_since_epoch()).count()
                                       <<std::endl;

                              std::cout<<"["<<iActionIndex
                                       <<"] reportable propagation: "
                                       <<reportablePropagation.count()
                                       <<std::endl;

                              std::cout<<"["<<iActionIndex
                                       <<"] reportable span: "
                                       <<span.count()
                                       <<std::endl;

                              std::cout<<"["<<iActionIndex
                                       <<"] reportable in-band: "
                                       <<(bTreatAsInBand ? "yes" : "no")
                                       <<std::endl;

                              for(const auto & segment : reportableSegments)
                                {
                                  std::cout<<"["<<iActionIndex
                                           <<"] reportable frequency: "
                                           <<segment.getFrequencyHz()
                                           <<std::endl;

                                  std::cout<<"["<<iActionIndex
                                           <<"] reportable offset: "
                                           <<segment.getOffset().count()
                                           <<std::endl;

                                  std::cout<<"["<<iActionIndex
                                           <<"] reportable duration: "
                                           <<segment.getDuration().count()
                                           <<std::endl;

                                  std::cout<<"["<<iActionIndex
                                           <<"] reportable rx power dBm: "
                                           <<segment.getRxPowerdBm()
                                           <<std::endl;
                                }
                                  
                              for(const auto & freq : spectrumMonitor.getFrequencies())
                                {
                                  std::cout<<" Frequency: "<<freq<<std::endl;
                                      
                                  for(const auto & entry : EMANE::Utils::spectrumCompress(spectrumMonitor.dump(freq)))
                                    {
                                      std::cout<<"  "<<entry.first<<":"<<entry.second<<std::endl;
                                    }
                                }
                              std::cout<<std::endl;
                            }
                          catch(...)
                            {
                              std::cerr<<"error occurred while procssing update at "<<now.count()<<std::endl;
                              throw;
                            }

                        }
                      else if(!xmlStrcmp(pActionNode->name,BAD_CAST "request"))
                        {
                          // what would get gettimeofday() in running code
                          xmlChar * pNow = xmlGetProp(pActionNode,BAD_CAST "now");
              
                          auto now = 
                            EMANE::Microseconds{EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pNow)).toUINT64()};
                           
                          xmlFree(pNow);
                           
                           
                          xmlChar * pFrequency = xmlGetProp(pActionNode,BAD_CAST "frequency");
                           
                          auto frequency = EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pFrequency)).toUINT64();
                           
                          xmlFree(pFrequency);

                          EMANE::TimePoint timepoint = EMANE::TimePoint::min();
                              
                          // what would get gettimeofday() in running code
                          xmlChar * pTime = xmlGetProp(pActionNode,BAD_CAST "time");
              
                          if(pTime)
                            {
                              timepoint = 
                                TimePoint{start + EMANE::Microseconds{EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pTime)).toUINT64()}};
                           
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
                              std::vector<double> bins;
                              EMANE::TimePoint startOfBinTime;
                              EMANE::Microseconds binSize;
                              double dReceiverSensativityMilliWatt;
                              bool bSignalInNoise;
                              
                              std::tie(bins,startOfBinTime,binSize,dReceiverSensativityMilliWatt,bSignalInNoise) =
                                spectrumMonitor.request_i(TimePoint{start+now},frequency,duration,timepoint);

                              std::cout<<"["<<++iActionIndex
                                       <<"] request abs time: "
                                       <<std::chrono::duration_cast<EMANE::Microseconds>(TimePoint{start+now}.time_since_epoch()).count()
                                       <<" relative time: "
                                       <<std::chrono::duration_cast<EMANE::Microseconds>(now).count()
                                       <<" bin size: "
                                       <<binSize.count()
                                       <<" rx sensativity (mW): "
                                       <<dReceiverSensativityMilliWatt
                                       <<" signal in noise: "
                                       <<(bSignalInNoise ? "yes"  : "no")
                                       <<std::endl;

                              if(timepoint != EMANE::TimePoint::min())
                                {
                                  std::cout<<" Frequency: "
                                           <<frequency
                                           <<" request timepoint: "
                                           <<std::chrono::duration_cast<EMANE::Microseconds>(timepoint.time_since_epoch()).count()
                                           <<" response timepoint: "
                                           <<std::chrono::duration_cast<EMANE::Microseconds>(startOfBinTime.time_since_epoch()).count()
                                           <<std::endl;
                                }
                              else
                                {
                                  std::cout<<" Frequency: "
                                           <<frequency
                                           <<" request timepoint: "
                                           <<" (now - duration)"
                                           <<" response timepoint: "
                                           <<std::chrono::duration_cast<EMANE::Microseconds>(startOfBinTime.time_since_epoch()).count()
                                           <<std::endl;
                                }
                                      
                              for(const auto & entry : EMANE::Utils::spectrumCompress(bins))
                                {
                                  std::cout<<"  "<<entry.first<<":"<<entry.second<<std::endl;
                                }
                                 
                              std::cout<<std::endl;
                            }
                          catch(...)
                            {
                              std::cerr<<"error occurred while procssing request at "<<now.count()<<std::endl;
                              throw;
                            }
                        }
                    }
                }
            }
        }
    }

  if(pDoc)
    {
      xmlFreeDoc(pDoc);
    }


  xmlCleanupParser();

  return 0;
}

void usage()
{
  std::cout<<"usage: ew [OPTIONS]... SCENARIOXML"<<std::endl;
  std::cout<<std::endl;
  std::cout<<"options:"<<std::endl;
  std::cout<<"  -h, --help                     Print this message and exit."<<std::endl;
  std::cout<<"  -i, --iterations NUM           Number of iterations per test case."<<std::endl;
  std::cout<<"                                   Default: 1"<<std::endl;
  std::cout<<"  -o, --outdir DIR               Directory to write output."<<std::endl;
  std::cout<<"                                   Default: CWD"<<std::endl;
  std::cout<<"  -s, --schema                   Scenario schema"<<std::endl;
  std::cout<<"                                   Default: noisescenario.xsd"<<std::endl;
  std::cout<<std::endl;
}
