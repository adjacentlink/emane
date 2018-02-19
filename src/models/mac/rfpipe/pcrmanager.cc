/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2010 - DRS CenGen, LLC, Columbia, Maryland
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

#include "pcrmanager.h"
#include "emane/utils/parameterconvert.h"

#include <libxml/parser.h>

#include <sstream>

#include <cmath>

namespace 
{
  const int PRECISION_FACTOR = 100;

  xmlChar* toXmlChar(const char * arg)
   {
     return reinterpret_cast<xmlChar*>(const_cast<char *>(arg));
   }
}


EMANE::Models::RFPipe::PCRManager::PCRManager(EMANE::NEMId id, EMANE::PlatformServiceProvider * pPlatformService) :
  id_{id},
  pPlatformService_{pPlatformService},
  tablePacketSize_{}
{}



EMANE::Models::RFPipe::PCRManager::~PCRManager()
{}


void 
EMANE::Models::RFPipe::PCRManager::Open(const std::string & uri, xmlParserCtxtPtr * ppContext, xmlDoc ** ppDocument, xmlNode ** ppRoot)
{
  if((*ppContext = xmlNewParserCtxt()) == NULL)
    {
      std::stringstream excString;
      excString << "EMANE::Models::RFPipe::PCRManager::Open: Failed to allocate parser context" << std::ends;

      throw StartException(excString.str());
    }
  else if((*ppDocument = xmlCtxtReadFile(*ppContext, uri.c_str(), NULL, XML_PARSE_DTDVALID)) == NULL)
    {
      std::stringstream excString;
      excString << "EMANE::Models::RFPipe::PCRManager::Open: Failed to parse document " << uri << std::ends;

      throw StartException(excString.str());
    }
  else if((*ppContext)->valid == false)
    {          
      std::stringstream excString;
      excString << "EMANE::Models::RFPipe::PCRManager::Open: Document in " << uri << " is not valid" << std::ends;

      throw StartException(excString.str());
    }
  else if((*ppRoot = xmlDocGetRootElement(*ppDocument)) == NULL) 
    {
      std::stringstream excString;
      excString << "EMANE::Models::RFPipe::PCRManager::Open: Could not get root element" << std::ends;

      throw StartException(excString.str());
    }
}


void 
EMANE::Models::RFPipe::PCRManager::Close(xmlParserCtxtPtr * ppContext, xmlDoc ** ppDocument)
{
  if(ppContext)
    {
      xmlFreeParserCtxt(*ppContext);      
      *ppContext = NULL;
    }

  if(ppDocument) 
    {
      xmlFreeDoc(*ppDocument);
      *ppDocument = NULL;
    }

  xmlCleanupParser();
}



void EMANE::Models::RFPipe::PCRManager::load(const std::string & uri)
{
  xmlDoc  * doc{};
  xmlNode * root{};
  xmlParserCtxtPtr pContext{};

  if(uri.empty())
    {
      std::stringstream excString;
      excString << "EMANE::Models::RFPipe::PCRManager::load: must supply curve file name" << std::ends;

      throw StartException(excString.str());
    }

  // open doc
  Open(uri, &pContext, &doc, &root);

  // root
  xmlNodePtr cur{root};

  // clear pcr entries
  pcrEntryVector_.clear();

  // clear por entries
  porVector_.clear();

  // get table
  while(cur) 
    {
      if((!xmlStrcmp(cur->name, toXmlChar("pcr")))) 
        {
          getTable(cur->xmlChildrenNode);
        }

      cur = cur->next;
    }

  // close doc
  Close(&pContext, &doc);

  // need at least 1 point
  if(pcrEntryVector_.size() < 1)
    {
      std::stringstream excString;
      excString << "EMANE::Models::RFPipe::PCRManager::getRows: need at least 1 point to define a pcr curve " << std::ends;

      throw StartException(excString.str());
    }

  // fill in points
  interpolate();
}



void
EMANE::Models::RFPipe::PCRManager::getTable(xmlNodePtr cur)
{
  while(cur) 
    {
      if((!xmlStrcmp(cur->name, toXmlChar("table"))))
        {
          tablePacketSize_ = Utils::ParameterConvert(getAttribute(cur, toXmlChar("pktsize"))).toUINT32();

          getRows(cur->xmlChildrenNode);
        }

      cur = cur->next;
    }
}



void
EMANE::Models::RFPipe::PCRManager::getRows(xmlNodePtr cur)
{
  while(cur) 
    {
      if((!xmlStrcmp(cur->name, toXmlChar("row")))) 
        {
          // get sinr value
          const float fSINR{Utils::ParameterConvert(getAttribute(cur, toXmlChar("sinr"))).toFloat(-255.0f, 255.0f)};

          // get por value, convert from percent to fraction
          const float fPOR{Utils::ParameterConvert(getAttribute(cur, toXmlChar("por"))).toFloat(0.0f, 100.0f) / 100.0f};

          for(const auto & entry : pcrEntryVector_)
            {
              if(fSINR == entry.fSINR_)
                {
                  std::stringstream excString;
                  excString << "EMANE::Models::RFPipe::PCRManager::getRows: duplicate sinr value " << fSINR << std::ends;
                  throw StartException(excString.str());
                }
              else if(fSINR < entry.fSINR_)
                {
                  std::stringstream excString;
                  excString << "EMANE::Models::RFPipe::PCRManager::getRows: out of order sinr value, must be in increasing value " << fSINR << std::ends;
                  throw StartException(excString.str());
                }
            }
          
          pcrEntryVector_.push_back(EMANE::Models::RFPipe::PCRManager::PCREntry(fSINR, fPOR));
        }
 
      cur = cur->next;
    }
}


void 
EMANE::Models::RFPipe::PCRManager::interpolate()
{
  // for each sinr/pcr entry
  for(size_t i = 0; i < (pcrEntryVector_.size() - 1); ++i)
    {
      // x1
      const int x1{static_cast<int>(pcrEntryVector_[i].fSINR_ * PRECISION_FACTOR)};

      // y1
      const float y1{pcrEntryVector_[i].fPOR_};

      // x2
      const int x2{static_cast<int>(pcrEntryVector_[i + 1].fSINR_ * PRECISION_FACTOR)};

      // y2
      const float y2{pcrEntryVector_[i + 1].fPOR_};

      // slope
      const float slope{(y2 - y1) / (x2 - x1)};

      // fill in between points
      for(int dx = x1; dx < x2; ++dx)
        {
          // y = mx + b
          porVector_.push_back((dx - x1) * slope + y1);
        }
    }

  porVector_.push_back(pcrEntryVector_[pcrEntryVector_.size()-1].fPOR_);
}


float 
EMANE::Models::RFPipe::PCRManager::getPCR(float fSINR, size_t packetLen)
{
  // check low end
  if(fSINR < pcrEntryVector_.front().fSINR_)
    {
      // full loss
      return 0.0f; 
    }
  // check high end
  else if(fSINR > pcrEntryVector_.back().fSINR_)
    {
      // no loss
      return 1.0f;
    }
  // lookup
  else
    {
      // por value
      float fPOR{};

      // single point
      if(fSINR == pcrEntryVector_.front().fSINR_ && fSINR == pcrEntryVector_.back().fSINR_)
        {
          fPOR = pcrEntryVector_.front().fPOR_;
        }
      else
        {
          // sinr adjusted
          const int sinrScaled{static_cast<int>(fSINR * PRECISION_FACTOR)};

          // sinr offset from the front
          const int sinrOffset{static_cast<int>(pcrEntryVector_.front().fSINR_ * PRECISION_FACTOR)};

          // get the table index
          int idx{sinrScaled - sinrOffset};

          // cap max index, low end check covers min index
          if(idx >= static_cast<int>(porVector_.size()))
            {
              idx = porVector_.size() - 1;
            }

          // direct lookup
          fPOR = porVector_[idx];
        }

      // adjust for pkt size is given in the table
      if(tablePacketSize_ > 0)
        {
          fPOR = pow(fPOR,(float) packetLen /(float) tablePacketSize_);
        }

      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                             DEBUG_LEVEL, 
                             "MACI %03hu PCRManager::%s: sinr %3.2f, len %zu, por %3.2f",
                             id_, 
                             __func__, 
                             fSINR, 
                             packetLen, 
                             fPOR);
      return fPOR;
    }
}




std::string 
EMANE::Models::RFPipe::PCRManager::getAttribute(xmlNodePtr cur, const xmlChar * id)
{
  std::string str{"0"};

  if(id) 
    {
      xmlChar *attr{xmlGetProp(cur, id)};

      if(attr) 
        {
          str = reinterpret_cast<const char *>(attr);
        }
      xmlFree(attr);
    }

  return str;
}



std::string 
EMANE::Models::RFPipe::PCRManager::getContent(xmlNodePtr cur)
{
  std::string str{"0"};

  xmlChar *attr{xmlNodeGetContent(cur)};

  if(attr) 
    {
      str = reinterpret_cast<const char *>(attr);
    }

  xmlFree(attr);

  return str;
}
