/*
 * Copyright (c) 2015,2023 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "pcrmanager.h"
#include "emane/configurationexception.h"
#include "emane/utils/parameterconvert.h"

#include <cmath>
#include <cstdlib>
#include <limits>

#include <libxml/parser.h>
#include <libxml/xmlschemas.h>

namespace
{
  const char * pzSchema="\
<xs:schema xmlns:xs='http://www.w3.org/2001/XMLSchema'>\
  <xs:simpleType name='SINRType'>\
    <xs:restriction base='xs:token'>\
      <xs:pattern value='[+-]?(0|[1-9][0-9]*)(.[0-9]{1,2})?'/>\
    </xs:restriction>\
  </xs:simpleType>\
  <xs:simpleType name='PORType'>\
    <xs:restriction base='xs:token'>\
      <xs:pattern value='(0|[1-9][0-9]*)(.[0-9]{1,2})?'/>\
    </xs:restriction>\
  </xs:simpleType>\
  <xs:element name='bentpipe-model-pcr'>\
    <xs:complexType>\
      <xs:sequence>\
        <xs:element name='curve' maxOccurs='unbounded'>\
          <xs:complexType>\
            <xs:sequence>\
               <xs:element name='entry' maxOccurs='unbounded'>\
                 <xs:complexType>\
                   <xs:attribute name='sinr' type='SINRType' use='optional'/>\
                   <xs:attribute name='por' type='PORType' use='optional'/>\
                 </xs:complexType>\
               </xs:element>\
            </xs:sequence>\
            <xs:attribute name='index' type='xs:unsignedShort' use='required'/>\
          </xs:complexType>\
        </xs:element>\
      </xs:sequence>\
      <xs:attribute name='packetsize' type='xs:unsignedShort' use='required'/>\
    </xs:complexType>\
  </xs:element>\
</xs:schema>";

  std::string scaleFloatToInteger(const std::string & sValue)
  {
    const int iScaleFactor{2};
    std::string sTmpParameter{sValue};

    // location of decimal point, if exists
    std::string::size_type indexPoint =  sTmpParameter.find(".",0);

    if(indexPoint != std::string::npos)
      {
        std::string::size_type numberOfDigitsAfterPoint =
          sTmpParameter.size() - indexPoint - 1;

        if(numberOfDigitsAfterPoint > iScaleFactor)
          {
            // need to move the decimal point, enough digits are present
            sTmpParameter.insert(sTmpParameter.size() - (numberOfDigitsAfterPoint - iScaleFactor),
                                 ".");
          }
        else
          {
            // need to append 0s
            sTmpParameter.append(iScaleFactor - numberOfDigitsAfterPoint,'0');
          }

        // remove original decimal point
        sTmpParameter.erase(indexPoint,1);
      }
    else
      {
        // need to append 0s
        sTmpParameter.append(iScaleFactor,'0');
      }

    return sTmpParameter;
  }
}

EMANE::Models::BentPipe::PCRManager::PCRManager():
  modifierLengthBytes_{}{}

void EMANE::Models::BentPipe::PCRManager::load(const std::string & sPCRFileName)
{
  xmlDocPtr pSchemaDoc{xmlReadMemory(pzSchema,
                                     strlen(pzSchema),
                                     "file:///bentpipe-model-pcr.xsd",
                                     NULL,
                                     0)};
  if(!pSchemaDoc)
    {
      throw makeException<ConfigurationException>("unable to open schema");
    }

  xmlSchemaParserCtxtPtr pParserContext{xmlSchemaNewDocParserCtxt(pSchemaDoc)};

  if(!pParserContext)
    {
      throw makeException<ConfigurationException>("bad schema context");
    }

  xmlSchemaPtr pSchema{xmlSchemaParse(pParserContext)};

  if(!pSchema)
    {
      throw makeException<ConfigurationException>("bad schema parser");
    }

  xmlSchemaValidCtxtPtr pSchemaValidCtxtPtr{xmlSchemaNewValidCtxt(pSchema)};

  if(!pSchemaValidCtxtPtr)
    {
      throw makeException<ConfigurationException>("bad schema valid context");
    }

  xmlSchemaSetValidOptions(pSchemaValidCtxtPtr,XML_SCHEMA_VAL_VC_I_CREATE);

  xmlDocPtr pDoc = xmlReadFile(sPCRFileName.c_str(),nullptr,0);

  if(xmlSchemaValidateDoc(pSchemaValidCtxtPtr, pDoc))
    {
      throw makeException<ConfigurationException>("invalid document: %s",
                                                  sPCRFileName.c_str());
    }

  xmlNodePtr pRoot = xmlDocGetRootElement(pDoc);

  xmlChar * pPacketSize = xmlGetProp(pRoot,BAD_CAST "packetsize");

  modifierLengthBytes_ = Utils::ParameterConvert(reinterpret_cast<const char *>(pPacketSize)).toUINT16();

  xmlFree(pPacketSize);

  for(xmlNodePtr pCurveNode = pRoot->children;
      pCurveNode != nullptr;
      pCurveNode = pCurveNode->next)
    {
      if(!xmlStrcmp(pCurveNode->name, BAD_CAST "curve"))
        {
          xmlChar * pIndex = xmlGetProp(pCurveNode,BAD_CAST "index");

          PCRCurveIndex index =
            Utils::ParameterConvert(reinterpret_cast<const char *>(pIndex)).toUINT16();

          xmlFree(pIndex);

          Curve curve{};
          std::int32_t i32MinScaledSINR{std::numeric_limits<std::int32_t>::max()};
          std::int32_t i32MaxScaledSINR{std::numeric_limits<std::int32_t>::lowest()};

          for(xmlNodePtr pEntryNode = pCurveNode->children;
              pEntryNode != nullptr;
              pEntryNode = pEntryNode->next)
            {
              if(!xmlStrcmp(pEntryNode->name, BAD_CAST "entry"))
                {
                  xmlChar * pSINR = xmlGetProp(pEntryNode,BAD_CAST "sinr");
                  xmlChar * pPOR = xmlGetProp(pEntryNode,BAD_CAST "por");

                  std::int32_t i32ScaledSINR =
                    Utils::ParameterConvert(scaleFloatToInteger(reinterpret_cast<const char *>(pSINR))).toINT32();

                  auto ret =
                    curve.insert(std::make_pair(i32ScaledSINR,
                                                Utils::ParameterConvert(reinterpret_cast<const char *>(pPOR)).toFloat()/100));

                  if(!ret.second)
                    {
                      throw makeException<ConfigurationException>("duplicate PCR SINR value for index: %hu sinr: %s in %s",
                                                                  index,
                                                                  reinterpret_cast<const char *>(pSINR),
                                                                  sPCRFileName.c_str());
                    }

                  xmlFree(pSINR);
                  xmlFree(pPOR);

                  i32MinScaledSINR = std::min(i32ScaledSINR,i32MinScaledSINR);
                  i32MaxScaledSINR = std::max(i32ScaledSINR,i32MaxScaledSINR);
                }
            }

          Curve interpolation{};

          auto iter = curve.begin();

          while(iter != curve.end())
            {
              auto x0 = iter->first;
              auto y0 = iter->second;

              ++iter;

              if(iter != curve.end())
                {
                  auto x1 = iter->first;
                  auto y1 = iter->second;

                  auto slope = (y1 - y0) / (x1 - x0);

                  for(auto i = x0; i < x1; ++i)
                    {
                      interpolation.insert({i,y1 - (x1 - i) * slope});
                    }
                }
            }

          curve.insert(interpolation.begin(),interpolation.end());

          auto ret = curveTable_.insert(std::make_pair(index,
                                                       std::make_tuple(i32MinScaledSINR,
                                                                       i32MaxScaledSINR,
                                                                       curve)));

          if(!ret.second)
            {
              throw makeException<ConfigurationException>("duplicate PCR curve: %zu in %s",
                                                          index,
                                                          sPCRFileName.c_str());
            }

        }
    }

  xmlFreeDoc(pSchemaDoc);

  xmlFreeDoc(pDoc);
}


std::optional<float> EMANE::Models::BentPipe::PCRManager::getPOR(PCRCurveIndex index,
                                                                 float fSINR,
                                                                 size_t packetLengthBytes) const
{
  auto iter = curveTable_.find(index);

  if(iter == curveTable_.end())
    {
      return {};
    }

  std::int32_t i32MinScaledSINR = std::get<0>(iter->second);
  std::int32_t i32MaxScaledSINR = std::get<1>(iter->second);
  const Curve & curve = std::get<2>(iter->second);

  std::int32_t i32Scaled{static_cast<std::int32_t>(fSINR * 100)};

  if(i32Scaled < i32MinScaledSINR)
    {
      return 0;
    }

  if(i32Scaled > i32MaxScaledSINR)
    {
      return 1;
    }

  auto curveIter = curve.find(i32Scaled);

  if(curveIter != curve.end())
    {
      auto por = curveIter->second;

      if(modifierLengthBytes_)
        {
          por = powf(por, packetLengthBytes / static_cast<float>(modifierLengthBytes_));
        }

      return por;
    }

  return 0;
}

const EMANE::Models::BentPipe::PCRManager::CurveTable &
EMANE::Models::BentPipe::PCRManager::PCRManager::getCurveTable() const
{
  return curveTable_;
}
