/*
 * Copyright (c) 2013,2016 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2009-2010 - DRS CenGen, LLC, Columbia, Maryland
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
 */

#include "filterreader.h"
#include "ipprotocoludprule.h"
#include "ipprotocolsimplerule.h"
#include "ethernetprotocolipv4rule.h"

#include "emane/net.h"
#include "emane/utils/parameterconvert.h"

#include <arpa/inet.h>

void
EMANE::Models::CommEffect::FilterReader::getFilters(xmlNode * cur, Filters & filters)
{
  while(cur)
    {
      if((!xmlStrcmp(cur->name, XMLCHAR "filter")))
        {
          Target * pTarget = getTarget(cur->xmlChildrenNode);

          if(pTarget)
            {
              auto optionalEffect = getEffect(cur->xmlChildrenNode);

              if(optionalEffect.second)
                {
                  filters.push_back({pTarget,optionalEffect.first});
                }
              else
                {
                  delete pTarget;
                }
            }
        }
      cur = cur->next;
    }
}



EMANE::Models::CommEffect::Target *
EMANE::Models::CommEffect::FilterReader::getTarget(xmlNode * cur)
{
  while(cur)
    {
      if((!xmlStrcmp(cur->name, XMLCHAR "target")))
        {
          EthernetProtocolRules rules =
            getEthernetProtocolRules(cur->xmlChildrenNode);

          return new Target(rules);
        }
      cur = cur->next;
    }

  return NULL;
}



std::pair<EMANE::Events::CommEffect,bool>
EMANE::Models::CommEffect::FilterReader::getEffect(xmlNode * cur)
{
  while(cur)
    {
      if((!xmlStrcmp(cur->name, XMLCHAR "effect")))
        {
          return std::make_pair(Events::CommEffect{0,
                getDuration(cur->xmlChildrenNode, "latency"),
                getDuration(cur->xmlChildrenNode, "jitter"),
                getProbability(cur->xmlChildrenNode, "loss"),
                getProbability(cur->xmlChildrenNode, "duplicate"),
                getBitRate(cur->xmlChildrenNode, "unicastbitrate"),
                getBitRate(cur->xmlChildrenNode, "broadcastbitrate")},true);
        }
      cur = cur->next;
    }

  return std::make_pair(EMANE::Events::CommEffect{0,{},{},0,0,0,0},false);
}



EMANE::Models::CommEffect::EthernetProtocolRules
EMANE::Models::CommEffect::FilterReader::getEthernetProtocolRules(xmlNode * cur)
{
  EthernetProtocolRules rules;

  while(cur)
    {
      if((!xmlStrcmp(cur->name, XMLCHAR "ipv4")))
        {
          std::string sSrc = getAttribute(cur, XMLCHAR "src");
          std::string sDst = getAttribute(cur, XMLCHAR "dst");
          std::string sLen = getAttribute(cur, XMLCHAR "len");
          std::string sTTL = getAttribute(cur, XMLCHAR "ttl");
          std::string sTOS = getAttribute(cur, XMLCHAR "tos");

          EthernetProtocolIPv4Rule * ethProto =
            new EthernetProtocolIPv4Rule{static_cast<uint32_t>(inet_addr(sSrc.c_str())),
                                         static_cast<uint32_t>(inet_addr(sDst.c_str())),
                                         EMANE::HTONS(Utils::ParameterConvert(sLen).toUINT16()),
                                         Utils::ParameterConvert(sTOS).toUINT8(),
                                         Utils::ParameterConvert(sTTL).toUINT8(),
                                         getIPProtocolRules(cur->xmlChildrenNode)};

          rules.push_back(ethProto);
        }

      cur = cur->next;
    }

  return rules;
}



EMANE::Models::CommEffect::IPProtocolRules
EMANE::Models::CommEffect::FilterReader::getIPProtocolRules(xmlNodePtr cur)
{
  IPProtocolRules rules;

  while(cur)
    {
      if((!xmlStrcmp(cur->name, XMLCHAR "udp")))
        {
          std::string sSrc = getAttribute(cur, XMLCHAR "sport");
          std::string sDst = getAttribute(cur, XMLCHAR "dport");

          rules.push_back(new IPProtocolUDPRule{EMANE::HTONS(Utils::ParameterConvert(sSrc).toUINT16()),
                EMANE::HTONS(Utils::ParameterConvert(sDst).toUINT16())});
        }
      else if((!xmlStrcmp(cur->name, XMLCHAR "protocol")))
        {
          std::string type = getAttribute(cur, XMLCHAR "type");

          rules.push_back(new IPProtocolSimpleRule{Utils::ParameterConvert(type).toUINT8()});
        }

      cur = cur->next;
    }

  return rules;
}



EMANE::Microseconds
EMANE::Models::CommEffect::FilterReader::getDuration(xmlNode * cur, const char *id)
{
  while(cur)
    {
      if((!xmlStrcmp(cur->name, XMLCHAR id)))
        {
          std::string sSec  = getAttribute(cur, XMLCHAR "sec");
          std::string sUSec = getAttribute(cur, XMLCHAR "usec");

          return Microseconds{std::chrono::duration_cast<Microseconds>(std::chrono::seconds{Utils::ParameterConvert(sSec).toUINT32()}) +
              Microseconds{Utils::ParameterConvert(sUSec).toUINT32()}};
        }
      cur = cur->next;
    }

  return EMANE::Microseconds::zero();
}



float
EMANE::Models::CommEffect::FilterReader::getProbability(xmlNode * cur, const char * id)
{
  while(cur)
    {
      if((!xmlStrcmp(cur->name, XMLCHAR id)))
        {
          std::string sValue = getContent(cur);

          return Utils::ParameterConvert(sValue).toFloat();
        }
      cur = cur->next;
    }

  return 0;
}


std::uint64_t
EMANE::Models::CommEffect::FilterReader::getBitRate(xmlNode * cur, const char * id)
{
  while(cur)
    {
      if((!xmlStrcmp(cur->name, XMLCHAR id)))
        {
          std::string sValue = getContent(cur);

          return Utils::ParameterConvert(sValue).toUINT64();
        }
      cur = cur->next;
    }

  return 0;
}



std::string
EMANE::Models::CommEffect::FilterReader::getAttribute(xmlNodePtr cur, const xmlChar * id)
{
  // default. don't care
  std::string str = "0";

  if(id)
    {
      xmlChar *attr = xmlGetProp(cur, id);

      if(attr)
        {
          str =(const char *) attr;
        }
      xmlFree(attr);
    }

  return str;
}


std::string
EMANE::Models::CommEffect::FilterReader::getContent(xmlNodePtr cur)
{
  // default. don't care
  std::string str = "0";

  xmlChar *attr = xmlNodeGetContent(cur);

  if(attr)
    {
      str =(const char *) attr;
    }
  xmlFree(attr);

  return str;
}

EMANE::Models::CommEffect::Filters
EMANE::Models::CommEffect::FilterReader::load(const char * pzFilename)
{
  xmlDoc  * pDoc{};
  xmlNode * pRoot{};
  xmlParserCtxtPtr pContext{};
  Filters filters;

  if(pzFilename)
    {
      pContext = xmlNewParserCtxt();

      if(pContext)
        {
          // Allocate the document tree
          if(!(pDoc = xmlCtxtReadFile(pContext,
                                      pzFilename,
                                      0,
                                      XML_PARSE_DTDVALID|XML_PARSE_NOERROR)))
            {
              xmlFreeParserCtxt(pContext);

              throw FilterLoadFailure{};
            }

          if((pContext)->valid)
            {
              if(!(pRoot = xmlDocGetRootElement(pDoc)))
                {
                  xmlFreeParserCtxt(pContext);

                  throw FilterLoadFailure{};
                }
            }
          else
            {
              xmlFreeParserCtxt(pContext);

              xmlFreeDoc(pDoc);

              throw FilterLoadFailure{};
            }
        }
      else
        {
          throw FilterLoadFailure{};
        }
    }
  else
    {
      throw FilterLoadFailure{};;
    }

  xmlNodePtr pCurrent = pRoot;

  while(pCurrent)
    {
      if((!xmlStrcmp(pCurrent->name, XMLCHAR "commeffect")))
        {
          getFilters(pCurrent->xmlChildrenNode, filters);
        }

      pCurrent = pCurrent->next;
    }

  xmlFreeParserCtxt(pContext);

  xmlFreeDoc(pDoc);

  return filters;
}
