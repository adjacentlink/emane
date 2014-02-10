/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2010-2012 - DRS CenGen, LLC, Columbia, Maryland
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

#include "emane/utils/parameterconvert.h"

#include "pcrmanager.h"

#include <libxml/parser.h>
#include <math.h>

#include <sstream>


namespace
{
  const int PRECISION_FACTOR{100};

  inline xmlChar * toXMLChar(const char * p)
   {
      return reinterpret_cast<xmlChar *>(const_cast<char *>(p));
   }
}


EMANE::Models::IEEE80211ABG::PCRManager::PCRManager(EMANE::NEMId id, EMANE::PlatformServiceProvider * pPlatformService):
id_{id}, 
pPlatformService_{pPlatformService}, 
tablePacketSize_{}
{ }



EMANE::Models::IEEE80211ABG::PCRManager::~PCRManager()
{ }


void
EMANE::Models::IEEE80211ABG::PCRManager::openDoc(const std::string & uri,
                                              xmlParserCtxtPtr * ppContext,
                                              xmlDoc ** ppDocument, xmlNode ** ppRoot)
{
  if((*ppContext = xmlNewParserCtxt()) == NULL)
    {
      std::stringstream excString;
      excString << "IEEE80211ABG::PCRManager::openDoc: Failed to allocate parser context" << std::ends;

      throw EMANE::ConfigurationException(excString.str());
    }

  else if((*ppDocument = xmlCtxtReadFile(*ppContext, uri.c_str(), NULL, XML_PARSE_DTDVALID)) == NULL)
    {
      std::stringstream excString;
      excString << "IEEE80211ABG::PCRManager::openDoc: Failed to parse document " << uri << std::ends;

      throw EMANE::ConfigurationException(excString.str());
    }

  else if((*ppContext)->valid == false)
    {
      std::stringstream excString;
      excString << "IEEE80211ABG::PCRManager::openDoc: Document in " << uri << " is not valid" << std::ends;

      throw EMANE::ConfigurationException(excString.str());
    }

  else if((*ppRoot = xmlDocGetRootElement(*ppDocument)) == NULL)
    {
      std::stringstream excString;
      excString << "IEEE80211ABG::PCRManager::openDoc: Could not get root element" << std::ends;

      throw EMANE::ConfigurationException(excString.str());
    }
}


void
EMANE::Models::IEEE80211ABG::PCRManager::closeDoc(xmlParserCtxtPtr * ppContext, xmlDoc ** ppDocument)
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



void
EMANE::Models::IEEE80211ABG::PCRManager::load(const std::string & uri)
{
  xmlDoc * doc{};
  xmlNode * root{};
  xmlParserCtxtPtr pContext{};

  if(uri.empty())
    {
      std::stringstream excString;
      excString << "IEEE80211ABG::PCRManager::load: must supply curve file name" << std::ends;

      throw EMANE::ConfigurationException(excString.str());
    }

  // open doc
  openDoc(uri, &pContext, &doc, &root);

  // root
  xmlNodePtr cur {root};

  // clear pcr entries
  pcrPorMap_.clear();

  // get table
  while(cur)
    {
      if((!xmlStrcmp(cur->name, toXMLChar("pcr"))))
        {
          getTable(cur->xmlChildrenNode, pcrPorMap_);
        }

      cur = cur->next;
    }

  // close doc
  closeDoc(&pContext, &doc);

  // need at least 1 point
  for(auto & iter : pcrPorMap_)
    {
      if(iter.second.pcr_.size() < 1)
        {
          std::stringstream excString;
          excString << "IEEE80211ABG::PCRManager::load: need at least 1 point to define a pcr curve " << std::ends;
          throw EMANE::ConfigurationException(excString.str());
        }
    }

  // fill in points
  interpolate();
}



void
EMANE::Models::IEEE80211ABG::PCRManager::getTable(xmlNodePtr cur, PCRPORMap & map)
{
  while(cur)
    {
      if((!xmlStrcmp(cur->name, toXMLChar("table"))))
        {
          // save table packet size
          tablePacketSize_ = Utils::ParameterConvert(getAttribute(cur, toXMLChar("pktsize"))).toUINT32();

          // get each data rate
          getDataRate(cur->xmlChildrenNode, map);
        }

      cur = cur->next;
    }
}


void
EMANE::Models::IEEE80211ABG::PCRManager::getDataRate(xmlNodePtr cur, PCRPORMap & map)
{
  while(cur)
    {
      if((!xmlStrcmp(cur->name, toXMLChar("datarate"))))
        {
          // get data rate index
          const std::uint16_t u16DataRateIndex{Utils::ParameterConvert(getAttribute(cur, toXMLChar("index"))).toUINT16()};

          // entry 
          PCRPOR entry;

          // get pcr each row 
          getRows(cur->xmlChildrenNode, entry.pcr_);

          if(map.insert(std::make_pair(u16DataRateIndex, entry)).second == false)
            {
              std::stringstream excString;
              excString << "IEEE80211ABG::PCRManager::getDataRate: duplicate datarate index value " << u16DataRateIndex << std::ends;
              throw EMANE::ConfigurationException(excString.str());
            }
        }

      cur = cur->next;
    }
}


void
EMANE::Models::IEEE80211ABG::PCRManager::getRows(xmlNodePtr cur, PCREntryVector & vec)
{
  while(cur)
    {
      if((!xmlStrcmp(cur->name, toXMLChar("row"))))
        {
          // get sinr value
          const float fSINR{Utils::ParameterConvert(getAttribute(cur, toXMLChar("sinr"))).toFloat(-255.0f, 255.0f)};

          // get por value, convert from percent to fraction
          const float
            fPOR{Utils::ParameterConvert(getAttribute(cur, toXMLChar("por"))).toFloat(0.0f, 100.0f) / 100.0f};

          for(const auto & iter : vec)
            {
              if(fSINR == iter.fSINR_)
                {
                  std::stringstream excString;
                  excString << "IEEE80211ABG::PCRManager::getRows: duplicate sinr value " << fSINR << std::ends;
                  throw EMANE::ConfigurationException(excString.str());
                }
              else if(fSINR < iter.fSINR_)
                {
                  std::stringstream excString;
                  excString << "IEEE80211ABG::PCRManager::getRows: out of order sinr value, must be in increasing value " 
                            << fSINR << std::ends;
                  throw EMANE::ConfigurationException(excString.str());
                }
            }

          // append entry 
          vec.push_back(PCREntry(fSINR, fPOR));
        }

      cur = cur->next;
    }
}


void
EMANE::Models::IEEE80211ABG::PCRManager::interpolate()
{
  // for each datarate
  for(auto & iter : pcrPorMap_)
    {
      // for each sinr/pcr entry
      for(size_t i = 0; i < (iter.second.pcr_.size() - 1); ++i)
        {
          // x1
          const int x1{static_cast<int>(iter.second.pcr_[i].fSINR_ * PRECISION_FACTOR)};

          // y1
          const float y1{iter.second.pcr_[i].fPOR_};

          // x2
          const int x2{static_cast<int>(iter.second.pcr_[i + 1].fSINR_ * PRECISION_FACTOR)};

          // y2
          const float y2{iter.second.pcr_[i + 1].fPOR_};

          // slope(m)
          const float slope{(y2 - y1) / (x2 - x1)};

          // fill in between points
          for(int dx = x1; dx < x2; ++dx)
            {
              // y = mx + b
              const float por{(dx - x1) * slope + y1};

              // save value to end of vector
              iter.second.por_.push_back(por);
            }
        }

      iter.second.por_.push_back(iter.second.pcr_[iter.second.pcr_.size() - 1].fPOR_);
    }
}




float
EMANE::Models::IEEE80211ABG::PCRManager::getPCR(float fSINR, size_t packetLen, std::uint16_t u16DataRateIndex)
{
  const auto iter = pcrPorMap_.find(u16DataRateIndex);

  if(iter != pcrPorMap_.end())
    {
      // check low end
      if(fSINR < iter->second.pcr_.front().fSINR_)
        {
          LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                 DEBUG_LEVEL,
                                 "MACI %03hu PCRManager::%s: sinr %3.2f, for datarate index %hu, "
                                 "is below the low limit sinr of %3.2f",
                                 id_,
                                 __func__,
                                 fSINR, 
                                 u16DataRateIndex,
                                 iter->second.pcr_.front().fSINR_);

          // full loss
          return 0.0f;
        }
      // check high end
      else if(fSINR > iter->second.pcr_.back().fSINR_)
        {
          LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                 DEBUG_LEVEL,
                                 "MACI %03hu PCRManager::%s: sinr %3.2f, for datarate index %hu, "
                                 "is above the high limit sinr of %3.2f",
                                 id_, 
                                 __func__, 
                                 fSINR, 
                                 u16DataRateIndex,
                                 iter->second.pcr_.back().fSINR_);
          // no loss
          return 1.0f;
        }
      // lookup
      else
        {
          float fPOR{};

          // single point(front == back)
          if(fSINR == iter->second.pcr_.front().fSINR_ && fSINR == iter->second.pcr_.back().fSINR_)
            {
              // get por
              fPOR = iter->second.pcr_.front().fPOR_;
            }
          else
            {
              // sinr adjusted
              const int sinrScaled{static_cast<int>(fSINR * PRECISION_FACTOR)};

              // sinr offset from the front
              const int sinrOffset{static_cast<int>(iter->second.pcr_.front().fSINR_ * PRECISION_FACTOR)};

              // get the table index
              int idx{sinrScaled - sinrOffset};

              // cap max index, low end check covers min index
              if(idx >= static_cast<int>(iter->second.por_.size()))
               {
                  idx = iter->second.por_.size() - 1;
               }

              // direct por lookup
              fPOR = iter->second.por_[idx];

              LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                     DEBUG_LEVEL, 
                                     "MACI %03hu PCRManager::%s: lookup por %3.2f at table index %d",
                                     id_,
                                     __func__,
                                     fPOR, 
                                     idx);
            }

          // adjust por for packet length
          if(tablePacketSize_ > 0)
           {
             fPOR = powf(fPOR, static_cast<float>(packetLen) / tablePacketSize_);
           }

          LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                 DEBUG_LEVEL,
                                 "MACI %03hu PCRManager::%s: sinr %3.2f, for datarate index %hu, por %3.2f",
                                 id_, 
                                 __func__, 
                                 fSINR, 
                                 u16DataRateIndex,
                                 fPOR);

          // return por
          return fPOR;
        }
    }
  else
    {
      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                             DEBUG_LEVEL,
                             "MACI %03hu PCRManager::%s: unsupported datarate index %hu, por %3.2f",
                             id_,
                             __func__,
                             u16DataRateIndex,
                             0.0f);

      // full loss
      return 0.0;
    }
}




std::string EMANE::Models::IEEE80211ABG::PCRManager::getAttribute(xmlNodePtr cur, const xmlChar * id)
{
  std::string str{"0"};

  if(id)
    {
      xmlChar * attr {xmlGetProp(cur, id)};

      if(attr)
        {
          str = reinterpret_cast<const char *>(attr);
        }

      xmlFree(attr);
    }

  return str;
}



std::string EMANE::Models::IEEE80211ABG::PCRManager::getContent(xmlNodePtr cur)
{
  std::string str{"0"};

  xmlChar * attr{xmlNodeGetContent(cur)};

  if(attr)
    {
      str = reinterpret_cast<const char *>(attr);
    }

  xmlFree(attr);

  return str;
}
