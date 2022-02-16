/*
 * Copyright (c) 2021 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "spectralmaskmanager.h"
#include "frequencyoverlapratio.h"

#include "emane/configureexception.h"
#include "emane/spectralmaskexception.h"
#include "emane/utils/parameterconvert.h"
#include "emane/utils/conversionutils.h"

#include <libxml/parser.h>
#include <libxml/xmlschemas.h>
#include <libxml/xpath.h>

namespace
{
  const char * pzSchema="\
<xs:schema xmlns:xs='http://www.w3.org/2001/XMLSchema'>\
  <xs:simpleType name='MaskIdType'>\
    <xs:restriction base='xs:unsignedShort'>\
      <xs:minInclusive value='1'/>\
    </xs:restriction>\
  </xs:simpleType>\
  <xs:simpleType name='SIPrefixType'>\
    <xs:restriction base='xs:token'>\
      <xs:pattern value='[0-9]+(.[0-9]+){0,1}(G|M|K){0,1}'/>\
    </xs:restriction>\
  </xs:simpleType>\
  <xs:simpleType name='SSIPrefixType'>\
    <xs:restriction base='xs:token'>\
      <xs:pattern value='(\\+|-){0,1}[0-9]+(.[0-9]+){0,1}(G|M|K){0,1}'/>\
    </xs:restriction>\
  </xs:simpleType>\
  <xs:element name='spectral-mask-manifest'>\
    <xs:complexType>\
      <xs:sequence>\
        <xs:element name='mask'\
                    maxOccurs='unbounded'>\
          <xs:complexType>\
            <xs:sequence>\
              <xs:element name='primary'>\
                <xs:complexType>\
                  <xs:sequence>\
                    <xs:element name='width'\
                                maxOccurs='unbounded'>\
                      <xs:complexType>\
                        <xs:attribute name='hz'\
                                      type='SIPrefixType'\
                                      use='required'/>\
                        <xs:attribute name='dBr'\
                                      type='xs:decimal'\
                                      use='optional'/>\
                      </xs:complexType>\
                    </xs:element>\
                  </xs:sequence>\
                </xs:complexType>\
              </xs:element>\
              <xs:element name='spurs'\
                          minOccurs='0'>\
                <xs:complexType>\
                  <xs:sequence>\
                    <xs:element name='spur'\
                                minOccurs='0'\
                                maxOccurs='unbounded'>\
                      <xs:complexType>\
                        <xs:sequence>\
                          <xs:element name='width'\
                                      maxOccurs='unbounded'>\
                            <xs:complexType>\
                              <xs:attribute name='hz'\
                                            type='SIPrefixType'\
                                            use='required'/>\
                              <xs:attribute name='dBr'\
                                            type='xs:decimal'\
                                            use='required'/>\
                            </xs:complexType>\
                          </xs:element>\
                        </xs:sequence>\
                        <xs:attribute name='offset_from_center_hz'\
                                      type='SSIPrefixType'\
                                      use='required'/>\
                      </xs:complexType>\
                    </xs:element>\
                  </xs:sequence>\
                </xs:complexType>\
              </xs:element>\
            </xs:sequence>\
            <xs:attribute name='id'\
                          type='MaskIdType'\
                          use='required'/>\
          </xs:complexType>\
        </xs:element>\
      </xs:sequence>\
    </xs:complexType>\
    <xs:unique name='UniqueMaskId'>\
      <xs:selector xpath='mask'/>\
      <xs:field xpath='@id'/>\
    </xs:unique>\
  </xs:element>\
</xs:schema>";


}


void EMANE::SpectralMaskManager::load(const std::string & sSpectralMaskManifestURI)
{
  xmlDocPtr pSchemaDoc{xmlReadMemory(pzSchema,
                                     strlen(pzSchema),
                                     "file:///spectral-mask-manifest.xsd",
                                     NULL,
                                     0)};

  if(!pSchemaDoc)
    {
      throw SpectralMaskException{"unable to open schema"};
    }

  xmlSchemaParserCtxtPtr pParserContext{xmlSchemaNewDocParserCtxt(pSchemaDoc)};

  if(!pParserContext)
    {
      throw SpectralMaskException{"bad schema context"};
    }

  xmlSchemaPtr pSchema{xmlSchemaParse(pParserContext)};

  if(!pSchema)
    {
      throw SpectralMaskException{"bad schema parser"};
    }

  xmlSchemaValidCtxtPtr pSchemaValidCtxtPtr{xmlSchemaNewValidCtxt(pSchema)};

  if(!pSchemaValidCtxtPtr)
    {
      throw SpectralMaskException{"bad schema valid context"};
    }

  xmlSchemaSetValidOptions(pSchemaValidCtxtPtr,XML_SCHEMA_VAL_VC_I_CREATE);

  xmlDocPtr pDoc = xmlReadFile(sSpectralMaskManifestURI.c_str(),nullptr,0);

  if(xmlSchemaValidateDoc(pSchemaValidCtxtPtr, pDoc))
    {
      throw SpectralMaskException{"invalid document"};
    }

  xmlNodePtr pRoot = xmlDocGetRootElement(pDoc);

  if(!pRoot)
    {
      xmlFreeDoc(pDoc);
      throw makeException<SpectralMaskException>("Invalid document root %s",
                                                 sSpectralMaskManifestURI.c_str());
    }

  for(xmlNodePtr pNode = pRoot->children; pNode; pNode = pNode->next)
    {
      if(pNode->type == XML_ELEMENT_NODE)
        {
          if(!xmlStrcmp(pNode->name,BAD_CAST "mask"))
            {
              xmlChar * pId{xmlGetProp(pNode,BAD_CAST "id")};

              std::uint16_t u16Id =
                EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pId)).toUINT16();

              xmlFree(pId);

              SpectralMasks spectralMasks{};

              for(xmlNodePtr pChildNode = pNode->children; pChildNode; pChildNode = pChildNode->next)
                {
                  if(pChildNode->type == XML_ELEMENT_NODE)
                    {
                      if(!xmlStrcmp(pChildNode->name,BAD_CAST "primary"))
                        {
                          MaskShape primaryMaskShape{};
                          std::uint64_t u64BandwidthHz{};

                          for(xmlNodePtr pPrimaryChildNode = pChildNode->children;
                              pPrimaryChildNode;
                              pPrimaryChildNode = pPrimaryChildNode->next)
                            {
                              if(!xmlStrcmp(pPrimaryChildNode->name,BAD_CAST "width"))
                                {
                                  xmlChar * pWidthHz{xmlGetProp(pPrimaryChildNode,BAD_CAST "hz")};

                                  std::uint64_t u64WidthHz =
                                    EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pWidthHz)).toUINT64();

                                  xmlFree(pWidthHz);

                                  xmlChar * pdBr{xmlGetProp(pPrimaryChildNode,BAD_CAST "dBr")};

                                  double ddBr =
                                    EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pdBr)).toDouble();

                                  xmlFree(pdBr);

                                  primaryMaskShape.push_back(std::make_tuple(u64WidthHz,Utils::DB_TO_MILLIWATT(ddBr)));

                                  u64BandwidthHz += u64WidthHz;
                                }
                            }

                          spectralMasks.push_back(std::make_tuple(0,u64BandwidthHz,std::move(primaryMaskShape)));
                        }

                      else if(!xmlStrcmp(pChildNode->name,BAD_CAST "spurs"))
                        {
                          for(xmlNodePtr pSpursChildNode = pChildNode->children;
                              pSpursChildNode;
                              pSpursChildNode = pSpursChildNode->next)
                            {
                              if(!xmlStrcmp(pSpursChildNode->name,BAD_CAST "spur"))
                                {
                                  xmlChar * pOffsetHz{xmlGetProp(pSpursChildNode,BAD_CAST "offset_from_center_hz")};

                                  std::int64_t i64OffsetHz =
                                    EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pOffsetHz)).toINT64();

                                  xmlFree(pOffsetHz);

                                  MaskShape spurMaskShape{};
                                  std::uint64_t u64BandwidthHz{};

                                  for(xmlNodePtr pSpurChildNode = pSpursChildNode->children;
                                      pSpurChildNode;
                                      pSpurChildNode = pSpurChildNode->next)
                                    {
                                      if(!xmlStrcmp(pSpurChildNode->name,BAD_CAST "width"))
                                        {
                                          xmlChar * pWidthHz{xmlGetProp(pSpurChildNode,BAD_CAST "hz")};

                                          std::uint64_t u64WidthHz =
                                            EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pWidthHz)).toUINT64();

                                          xmlFree(pWidthHz);

                                          xmlChar * pdBr{xmlGetProp(pSpurChildNode,BAD_CAST "dBr")};

                                          double ddBr =
                                            EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pdBr)).toDouble();

                                          xmlFree(pdBr);

                                          spurMaskShape.push_back(std::make_tuple(u64WidthHz,Utils::DB_TO_MILLIWATT(ddBr)));

                                          u64BandwidthHz += u64WidthHz;
                                        }
                                    }

                                  spectralMasks.push_back(std::make_tuple(i64OffsetHz,u64BandwidthHz,std::move(spurMaskShape)));
                                }
                            }
                        }
                    }
                }
              spectralMaskStore_.insert({u16Id,std::move(spectralMasks)});
            }
        }
    }

  xmlFreeDoc(pDoc);
}


EMANE::SpectralMaskManager::MaskOverlap
EMANE::SpectralMaskManager::getSpectralOverlap(std::uint64_t u64TxFrequencyHz,
                                               std::uint64_t u64RxFrequencyHz,
                                               std::uint64_t u64RxBandwidthHz,
                                               std::uint64_t u64TxBandwidthHz,
                                               std::uint16_t u16SpectalMaskId) const
{
  MaskOverlap maskOverlap{};

  if(u16SpectalMaskId)
    {
      const auto iter = spectralMaskStore_.find(u16SpectalMaskId);

      SpectralOverlaps spectralOverlaps{};

      if(iter != spectralMaskStore_.end())
        {
          std::uint64_t u64LowerMaskOverlapFrequencyHz{std::numeric_limits<std::uint64_t>::max()};
          std::uint64_t u64UpperMaskOverlapFrequencyHz{};
          std::uint64_t u64Total{};

          for(const auto & maskEntry : iter->second)
            {
              std::int64_t i64OffsetFromCenterHz{std::get<0>(maskEntry)};
              std::uint64_t u64MaskBandwidthHz{std::get<1>(maskEntry)};

              if(!u64RxBandwidthHz)
                {
                  u64RxBandwidthHz = u64MaskBandwidthHz;
                }

              std::uint64_t u64LowerSpurOverlapFrequencyHz{std::numeric_limits<std::uint64_t>::max()};
              std::uint64_t u64UpperSpurOverlapFrequencyHz{};
              SpectralSegments spectralSegments{};

              std::uint64_t u64TxLowerFrequencyHz{u64TxFrequencyHz +
                i64OffsetFromCenterHz -
                u64MaskBandwidthHz / 2};

              for(const auto & maskSegmentEntry : std::get<2>(maskEntry))
                {
                  std::uint64_t u64SegmentWidthHz{std::get<0>(maskSegmentEntry)};
                  double dSegmentModifierMilliWatt{std::get<1>(maskSegmentEntry)};

                  std::uint64_t u64TxUpperFrequencyHz{u64TxLowerFrequencyHz +
                    u64SegmentWidthHz};

                  //percent in band, defaults to no coverage
                  double dRatio{};
                  std::uint64_t u64LowerSegmentOverlapFrequencyHz{};
                  std::uint64_t u64UpperSegmentOverlapFrequencyHz{};

                  std::tie(dRatio,
                           u64LowerSegmentOverlapFrequencyHz,
                           u64UpperSegmentOverlapFrequencyHz) =
                    frequencyOverlapRatio(u64RxFrequencyHz,
                                          u64RxBandwidthHz,
                                          u64TxLowerFrequencyHz + u64SegmentWidthHz / 2,
                                          u64SegmentWidthHz);

                  if(dRatio > 0)
                    {
                      spectralSegments.emplace_back(std::make_tuple(dRatio,
                                                                    dSegmentModifierMilliWatt,
                                                                    u64TxLowerFrequencyHz,
                                                                    u64TxUpperFrequencyHz));

                      if(u64LowerSpurOverlapFrequencyHz > u64LowerSegmentOverlapFrequencyHz)
                        {
                          u64LowerSpurOverlapFrequencyHz = u64LowerSegmentOverlapFrequencyHz;
                        }

                      if(u64UpperSpurOverlapFrequencyHz < u64UpperSegmentOverlapFrequencyHz)
                        {
                          u64UpperSpurOverlapFrequencyHz = u64UpperSegmentOverlapFrequencyHz;
                        }

                      ++u64Total;
                    }

                  // update for next segment start
                  u64TxLowerFrequencyHz += u64SegmentWidthHz;
                }

              if(!spectralSegments.empty())
                {
                  if(u64LowerMaskOverlapFrequencyHz > u64LowerSpurOverlapFrequencyHz)
                    {
                      u64LowerMaskOverlapFrequencyHz = u64LowerSpurOverlapFrequencyHz;
                    }

                  if(u64UpperMaskOverlapFrequencyHz < u64UpperSpurOverlapFrequencyHz)
                    {
                      u64UpperMaskOverlapFrequencyHz = u64UpperSpurOverlapFrequencyHz;
                    }

                  spectralOverlaps.emplace_back(std::move(spectralSegments),
                                                u64LowerSpurOverlapFrequencyHz,
                                                u64UpperSpurOverlapFrequencyHz);
                }
            }

          if(!spectralOverlaps.empty())
            {
              maskOverlap = std::make_tuple(std::move(spectralOverlaps),
                                            u64LowerMaskOverlapFrequencyHz,
                                            u64UpperMaskOverlapFrequencyHz,
                                            u64Total);
            }
        }
    }
  else
    {
      double dOverlapRatio{};
      std::uint64_t u64LowerOverlapFrequencyHz{};
      std::uint64_t u64UpperOverlapFrequencyHz{};

      std::tie(dOverlapRatio,
               u64LowerOverlapFrequencyHz,
               u64UpperOverlapFrequencyHz) =
        frequencyOverlapRatio(u64RxFrequencyHz,
                              u64RxBandwidthHz,
                              u64TxFrequencyHz,
                              u64TxBandwidthHz);

      SpectralSegments spectralSegments{SpectralSegment{dOverlapRatio,
          1,
          u64LowerOverlapFrequencyHz,
          u64UpperOverlapFrequencyHz}};

      SpectralOverlaps spectralOverlaps{SpectralOverlap{std::move(spectralSegments),
          u64LowerOverlapFrequencyHz,
          u64UpperOverlapFrequencyHz}};

      maskOverlap = std::make_tuple(std::move(spectralOverlaps),
                                    u64LowerOverlapFrequencyHz,
                                    u64UpperOverlapFrequencyHz,
                                    1);
    }

  return maskOverlap;
}
