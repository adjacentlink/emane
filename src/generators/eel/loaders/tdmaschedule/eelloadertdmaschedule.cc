/*
 * Copyright (c) 2025 - Adjacent Link LLC, Bridgewater, New Jersey
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
 *
 */

#include "eelloadertdmaschedule.h"
#include "schedulehelper.h"

#include "emane/events/tdmascheduleevent.h"
#include "emane/utils/parameterconvert.h"
#include "emane/generators/eel/formatexception.h"

#include "emane/events/tdmascheduleslotinfo.h"
#include "emane/events/tdmascheduleslotstructure.h"

#include <cstring>
#include <optional>

#include <libxml/parser.h>
#include <libxml/xmlschemas.h>

namespace
{
  const char * pzSchema="\
<xs:schema xmlns:xs='http://www.w3.org/2001/XMLSchema'>\
  <xs:simpleType name='IndexType'>\
    <xs:restriction base='xs:token'>\
      <xs:pattern value='([0-9]+,{0,1}|[0-9]+:[0-9]+,{0,1}){1,}'/>\
    </xs:restriction>\
  </xs:simpleType>\
  <xs:simpleType name='DestinationType'>\
    <xs:restriction base='xs:token'>\
      <xs:pattern value='([1-9][0-9]*|broadcast)'/>\
    </xs:restriction>\
  </xs:simpleType>\
  <xs:simpleType name='NodeType'>\
    <xs:restriction base='xs:token'>\
      <xs:pattern value='([1-9][0-9]*,{0,1}|[1-9][0-9]*:[1-9][0-9]*,{0,1}){1,}'/>\
    </xs:restriction>\
  </xs:simpleType>\
  <xs:simpleType name='SIPrefixType'>\
    <xs:restriction base='xs:token'>\
      <xs:pattern value='[0-9]+(.[0-9]+){0,1}(G|M|K){0,1}'/>\
    </xs:restriction>\
  </xs:simpleType>\
  <xs:simpleType name='ClassType'>\
    <xs:restriction base='xs:unsignedShort'>\
      <xs:minInclusive value='0'/>\
      <xs:maxInclusive value='4'/>\
    </xs:restriction>\
  </xs:simpleType>\
  <xs:element name='emane-tdma-schedule'>\
    <xs:complexType>\
      <xs:sequence>\
        <xs:element name='structure' minOccurs='0'>\
          <xs:complexType>\
            <xs:attribute name='slotduration' type='xs:unsignedLong' use='required'/>\
            <xs:attribute name='slotoverhead' type='xs:unsignedLong' use='required'/>\
            <xs:attribute name='frames' type='xs:unsignedShort' use='required'/>\
            <xs:attribute name='slots' type='xs:unsignedShort' use='required'/>\
            <xs:attribute name='bandwidth' type='SIPrefixType' use='required'/>\
          </xs:complexType>\
        </xs:element>\
        <xs:element name='multiframe'>\
          <xs:complexType>\
            <xs:sequence>\
              <xs:element name='frame' maxOccurs='unbounded'>\
                <xs:complexType>\
                  <xs:sequence>\
                    <xs:element name='slot' maxOccurs='unbounded'>\
                      <xs:complexType>\
                        <xs:choice minOccurs='0'>\
                          <xs:element name='tx'>\
                            <xs:complexType>\
                              <xs:attribute name='frequency' type='SIPrefixType' use='optional'/>\
                              <xs:attribute name='power' type='xs:decimal' use='optional'/>\
                              <xs:attribute name='class' type='ClassType' use='optional'/>\
                              <xs:attribute name='datarate' type='SIPrefixType' use='optional'/>\
                              <xs:attribute name='destination' type='DestinationType' use='optional'/>\
                            </xs:complexType>\
                          </xs:element>\
                          <xs:element name='rx'>\
                            <xs:complexType>\
                              <xs:attribute name='frequency' type='SIPrefixType' use='optional'/>\
                            </xs:complexType>\
                          </xs:element>\
                          <xs:element name='idle'/>\
                        </xs:choice>\
                        <xs:attribute name='index' type='IndexType' use='required'/>\
                        <xs:attribute name='nodes' type='NodeType' use='required'/>\
                      </xs:complexType>\
                    </xs:element>\
                  </xs:sequence>\
                  <xs:attribute name='index' type='IndexType' use='required'/>\
                  <xs:attribute name='frequency' type='SIPrefixType' use='optional'/>\
                  <xs:attribute name='power' type='xs:decimal' use='optional'/>\
                  <xs:attribute name='class' type='ClassType' use='optional'/>\
                  <xs:attribute name='datarate' type='SIPrefixType' use='optional'/>\
                </xs:complexType>\
              </xs:element>\
            </xs:sequence>\
            <xs:attribute name='frequency' type='SIPrefixType' use='optional'/>\
            <xs:attribute name='power' type='xs:decimal' use='optional'/>\
            <xs:attribute name='class' type='ClassType' use='optional'/>\
            <xs:attribute name='datarate' type='SIPrefixType' use='optional'/>\
          </xs:complexType>\
        </xs:element>\
      </xs:sequence>\
    </xs:complexType>\
  </xs:element>\
</xs:schema>";

  void load_default(const xmlNode * pNode,
                    std::optional<std::uint64_t> & u64FrequencyHz,
                    std::optional<double> & dPowerdBm,
                    std::optional<std::uint8_t> & u8ServiceClass,
                    std::optional<std::uint64_t> & u64Dataratebps)
  {
    xmlChar * pFrequency =
      xmlGetProp(pNode,BAD_CAST "frequency");

    xmlChar * pPower =
      xmlGetProp(pNode,BAD_CAST "power");

    xmlChar * pServiceClass =
      xmlGetProp(pNode,BAD_CAST "class");

    xmlChar * pDatarate =
      xmlGetProp(pNode,BAD_CAST "datarate");

    if(pFrequency)
      {
        u64FrequencyHz =
          EMANE::Utils::ParameterConvert{reinterpret_cast<const char *>(pFrequency)}.toUINT64();

        xmlFree(pFrequency);
      }

    if(pPower)
      {
        dPowerdBm =
          EMANE::Utils::ParameterConvert{reinterpret_cast<const char *>(pPower)}.toDouble();

        xmlFree(pPower);
      }

    if(pServiceClass)
      {
        u8ServiceClass =
          EMANE::Utils::ParameterConvert{reinterpret_cast<const char *>(pServiceClass)}.toUINT8();

        xmlFree(pServiceClass);
      }


    if(pDatarate)
      {
        u64Dataratebps =
          EMANE::Utils::ParameterConvert{reinterpret_cast<const char *>(pDatarate)}.toUINT64();

        xmlFree(pDatarate);
      }
  }
}

EMANE::Generators::EEL::LoaderTDMASchedule::LoaderTDMASchedule()
{}

EMANE::Generators::EEL::LoaderTDMASchedule::~LoaderTDMASchedule()
{}

void EMANE::Generators::EEL::LoaderTDMASchedule::load(const ModuleType & moduleType,
                                                      const ModuleId   &,
                                                      const EventType  &,
                                                      const InputArguments & args)
{
  if(moduleType == "nem")
    {
      if(args.size() != 1)
        {
          throw FormatException("LoaderTDMASchedule loader expects 1 argument");
        }
      else
        {
          std::string sTDMAScheduleXMLFile{args[0]};

          LIBXML_TEST_VERSION;

          xmlDocPtr pSchemaDoc{xmlReadMemory(pzSchema,
                                             strlen(pzSchema),
                                             "file:///emaneboundaryservice.xsd",
                                             NULL,
                                             0)};

          if(!pSchemaDoc)
            {
              throw FormatException{"unable to open schema"};
            }

          xmlSchemaParserCtxtPtr pParserContext{xmlSchemaNewDocParserCtxt(pSchemaDoc)};

          if(!pParserContext)
            {
              throw FormatException{"bad schema context"};
            }

          xmlSchemaPtr pSchema{xmlSchemaParse(pParserContext)};

          if(!pSchema)
            {
              throw FormatException{"bad schema parser"};
            }

          xmlSchemaValidCtxtPtr pSchemaValidCtxtPtr{xmlSchemaNewValidCtxt(pSchema)};

          if(!pSchemaValidCtxtPtr)
            {
              throw  FormatException{"bad schema valid context"};
            }

          xmlSchemaSetValidOptions(pSchemaValidCtxtPtr,XML_SCHEMA_VAL_VC_I_CREATE);

          xmlDocPtr pDoc = xmlReadFile(sTDMAScheduleXMLFile.c_str(),nullptr,0);

          if(xmlSchemaValidateDoc(pSchemaValidCtxtPtr,pDoc))
            {
              throw FormatException{"invalid document"};
            }

          xmlNodePtr pRoot = xmlDocGetRootElement(pDoc);

          for(xmlNodePtr pNode = pRoot->children;
              pNode;
              pNode = pNode->next)
            {
              if(pNode->type == XML_ELEMENT_NODE)
                {
                  if(!xmlStrcmp(pNode->name,BAD_CAST "structure"))
                    {
                      xmlChar * pSlotDuration =
                        xmlGetProp(pNode,BAD_CAST "slotduration");

                      xmlChar * pSlotOverhead =
                        xmlGetProp(pNode,BAD_CAST "slotoverhead");

                      xmlChar * pBandwidth =
                        xmlGetProp(pNode,BAD_CAST "bandwidth");

                      xmlChar * pSlots =
                        xmlGetProp(pNode,BAD_CAST "slots");

                      xmlChar * pFrames =
                        xmlGetProp(pNode,BAD_CAST "frames");

                      slotStructure_ =
                        Events::TDMASchedule::
                        SlotStructure{Utils::ParameterConvert{reinterpret_cast<const char *>(pBandwidth)}.toUINT64(),
                                      Utils::ParameterConvert{reinterpret_cast<const char *>(pFrames)}.toUINT32(),
                                      Utils::ParameterConvert{reinterpret_cast<const char *>(pSlots)}.toUINT32(),
                                      Microseconds{Utils::ParameterConvert{reinterpret_cast<const char *>( pSlotDuration)}.toUINT64()},
                                      Microseconds{Utils::ParameterConvert{reinterpret_cast<const char *>( pSlotOverhead)}.toUINT64()}};

                      xmlFree(pSlotDuration);
                      xmlFree(pSlotOverhead);
                      xmlFree(pBandwidth);
                      xmlFree(pSlots);
                      xmlFree(pFrames);
                    }
                  else if(!xmlStrcmp(pNode->name,BAD_CAST "multiframe"))
                    {
                      const xmlNode * pMultiFrameNode{pNode};

                      load_default(pMultiFrameNode,
                                   multiFrameParameters_.u64FrequencyHz_,
                                   multiFrameParameters_.dPowerdBm_,
                                   multiFrameParameters_.u8ServiceClass_,
                                   multiFrameParameters_.u64Dataratebps_);

                      for(xmlNodePtr pNode = pMultiFrameNode->children;
                          pNode;
                          pNode = pNode->next)
                        {
                          if(pNode->type == XML_ELEMENT_NODE)
                            {
                              if(!xmlStrcmp(pNode->name,BAD_CAST "frame"))
                                {
                                  const xmlNode * pFrameNode{pNode};

                                  xmlChar * pIndex =
                                    xmlGetProp(pNode,BAD_CAST "index");

                                  auto frameIndexes =
                                    expandRange<std::uint32_t>(reinterpret_cast<const char *>(pIndex));

                                  xmlFree(pIndex);

                                  Events::TDMASchedule::FrameParameters frameParameters{};

                                  load_default(pFrameNode,
                                               frameParameters.u64FrequencyHz_,
                                               frameParameters.dPowerdBm_,
                                               frameParameters.u8ServiceClass_,
                                               frameParameters.u64Dataratebps_);

                                  for(const auto & frameIndex : frameIndexes)
                                    {
                                      if(!frameParameterMap_.
                                         emplace(frameIndex,frameParameters).second)
                                        {
                                          throw FormatException("LoaderTDMASchedule"
                                                                "duplicate frame"
                                                                "defaults for index"
                                                                + std::to_string(frameIndex));
                                        }
                                    }

                                  for(xmlNodePtr pNode = pFrameNode->children;
                                      pNode;
                                      pNode = pNode->next)
                                    {
                                      if(pNode->type == XML_ELEMENT_NODE)
                                        {
                                          if(!xmlStrcmp(pNode->name,BAD_CAST "slot"))
                                            {
                                              const xmlNode * pSlotNode{pNode};

                                              xmlChar * pIndex =
                                                xmlGetProp(pSlotNode,BAD_CAST "index");

                                              auto slotIndexes =
                                                expandRange<std::uint32_t>(reinterpret_cast<const char *>(pIndex));

                                              xmlFree(pIndex);

                                              xmlChar * pNodes =
                                                xmlGetProp(pSlotNode,BAD_CAST "nodes");

                                              auto nodes =
                                                expandRange<NEMId>(reinterpret_cast<const char *>(pNodes));

                                              xmlFree(pNodes);

                                              Events::TDMASchedule::SlotParameters slotParameters{};

                                              for(xmlNodePtr pNode = pSlotNode->children;
                                                  pNode;
                                                  pNode = pNode->next)
                                                {
                                                  if(pNode->type == XML_ELEMENT_NODE)
                                                    {
                                                      if(!xmlStrcmp(pNode->name,BAD_CAST "tx"))
                                                        {
                                                          const xmlNode *  pTxNode{pNode};

                                                          Events::TDMASchedule::TxSlotParameters txSlotParameters{};

                                                          xmlChar * pDestination =
                                                            xmlGetProp(pTxNode,BAD_CAST "destination");

                                                          if(pDestination)
                                                            {
                                                              txSlotParameters.destination_ =
                                                                Utils::ParameterConvert{reinterpret_cast<const char *>(pDestination)}.toUINT16();

                                                              xmlFree(pDestination);
                                                            }

                                                          load_default(pTxNode,
                                                                       txSlotParameters.u64FrequencyHz_,
                                                                       txSlotParameters.dPowerdBm_,
                                                                       txSlotParameters.u8ServiceClass_,
                                                                       txSlotParameters.u64Dataratebps_);

                                                          txSlotParameters.destination_ = 0;

                                                          slotParameters.parameters_.
                                                            emplace<Events::TDMASchedule::TxSlotParameters>(std::move(txSlotParameters));
                                                        }
                                                      else if(!xmlStrcmp(pNode->name,BAD_CAST "rx"))
                                                        {
                                                          const xmlNode *  pRxNode{pNode};

                                                          xmlChar * pFrequency =
                                                            xmlGetProp(pRxNode,BAD_CAST "frequency");

                                                          Events::TDMASchedule::RxSlotParameters rxSlotParameters{};

                                                          if(pFrequency)
                                                            {
                                                              rxSlotParameters.u64FrequencyHz_ =
                                                                EMANE::Utils::ParameterConvert{reinterpret_cast<const char *>(pFrequency)}.toUINT64();

                                                              xmlFree(pFrequency);
                                                            }

                                                          slotParameters.parameters_.
                                                            emplace<Events::TDMASchedule::RxSlotParameters>(std::move(rxSlotParameters));
                                                        }
                                                      else if(!xmlStrcmp(pNode->name,BAD_CAST "idle"))
                                                        {
                                                          slotParameters.parameters_.
                                                            emplace<Events::TDMASchedule::IdleSlotParameters>(Events::TDMASchedule::IdleSlotParameters{});
                                                        }
                                                    }
                                                }

                                              if(slotParameters.parameters_.index() == std::variant_npos)
                                                {
                                                  slotParameters.parameters_.
                                                    emplace<Events::TDMASchedule::TxSlotParameters>(Events::TDMASchedule::TxSlotParameters{});
                                                }

                                              for(const auto & node : nodes)
                                                {
                                                  for(const auto & frameIndex : frameIndexes)
                                                    {
                                                      for(const auto & slotIndex : slotIndexes)
                                                        {
                                                          auto iter = scheduleNEMMap_.find(node);

                                                          if(iter == scheduleNEMMap_.end())
                                                            {
                                                              iter =
                                                                scheduleNEMMap_.emplace(node,Events::TDMASchedule::FrameSlotMap{}).first;
                                                            }

                                                          auto & frameInfo = iter->second;

                                                          auto frameIter = frameInfo.find(frameIndex);

                                                          if(frameIter == frameInfo.end())
                                                            {
                                                              frameIter =
                                                                frameInfo.emplace(frameIndex,Events::TDMASchedule::SlotParameterMap{}).first;
                                                            }

                                                          auto & slotInfo = frameIter->second;

                                                          slotInfo.emplace(slotIndex,std::move(slotParameters));
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

          xmlFreeDoc(pDoc);
        }
    }
}

EMANE::Generators::EEL::EventInfoList
EMANE::Generators::EEL::LoaderTDMASchedule::getEvents(EventPublishMode)
{
  EventInfoList eventInfoList;

  for(const auto & entry : scheduleNEMMap_)
    {
      NEMId nemId{entry.first};
      const auto & frameSlotMap{entry.second};

      eventInfoList.push_back({nemId,
          Events::TDMAScheduleEvent::IDENTIFIER,
          Events::TDMAScheduleEvent(slotStructure_,
                                    multiFrameParameters_,
                                    frameParameterMap_,
                                    frameSlotMap).serialize()});
    }

  scheduleNEMMap_.clear();
  frameParameterMap_.clear();
  slotStructure_.reset();
  multiFrameParameters_.reset();

  return eventInfoList;
}

DECLARE_EEL_LOADER_PLUGIN(EMANE::Generators::EEL::LoaderTDMASchedule)
