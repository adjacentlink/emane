/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
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


#ifndef EMANEMODELSIEEE802ABGIEEE80211MACPCRMANAGER_HEADER_
#define EMANEMODELSIEEE802ABGIEEE80211MACPCRMANAGER_HEADER_

#include "emane/types.h"
#include "emane/configurationexception.h"
#include "emane/platformserviceprovider.h"

#include <libxml/parser.h>

#include <string>
#include <vector>
#include <map>

namespace EMANE
 {
  namespace Models
   {
    namespace IEEE80211ABG
     {
      /**
       *
       * @class PCRManager
       *
       * @brief provides access to the pcr curves
       *
       */
        class PCRManager
        {
        /**
         *
         * @struct PCREntry
         *
         * @brief proides a direct lookup mapping of packet completion to probability of reception
         *
         */
          struct PCREntry
          {
            float fSINR_;
            float fPOR_;

          /**
           *
           * default constructor
           *
           */
            PCREntry(): 
             fSINR_{}, 
             fPOR_{}
             { } 

          /**
           *
           * initialized constructor
           *
           * @param fSinr the singnal to noise ratio
           * @param por  the probability of reception
           *
           */
            PCREntry(float fSinr, float por):
              fSINR_{fSinr}, 
              fPOR_{por}
            { }
          }; 

          /**
           *
           * @brief vector of PCREntry
           *
           */
          typedef std::vector <PCREntry> PCREntryVector;

          /**
           *
           * @brief vector of POR (probability of reception) values
           *
           */
          typedef std::vector <float> PORVector;

         /**
          *
          * @struct PCRPOR
          *
          * @brief proides a direct lookup mapping of packet completion to probability of reception over a range of entries
          *
          */
          struct PCRPOR
          {
            PCREntryVector pcr_;
            PORVector      por_;
          };

          /**
           *
           * @brief map of data rate index to PCRPOR
           *
           */
          typedef std::map <std::uint16_t, PCRPOR> PCRPORMap;

        public:
          /**
           * initialized constructor
           * 
           * @param id the NEM id
           *
           * @param pPlatformService a reference to the platform service(s)
           *
           */
          PCRManager(NEMId id, PlatformServiceProvider * pPlatformService);

          /**
           *  
           * destructor
           *
           */
          ~PCRManager();

          /**
           *  
           * loads the pcr curve
           *
           * @param uri the location of the pcr curve file
           *
           * @throw ConfigurationException
           */
          void load(const std::string & uri);

          /**
           *  
           * provides the pcr for a given sinr, packet size and data rate index
           *
           * @param fSinr the signal to noise ratio
           * @param size the packet size in bytes
           * @param DataRateIndex the data rate index
           *
           * @return retunrs the pcr
           *
           */
          float getPCR(float fSinr, size_t size, std::uint16_t DataRateIndex);

        private:
          void openDoc(const std::string & uri, xmlParserCtxtPtr * ppContext,
                     xmlDoc ** ppDocument, xmlNode ** ppRoot);

          void closeDoc(xmlParserCtxtPtr * ppContext, xmlDoc ** ppDocument);

          std::string getAttribute(xmlNodePtr cur, const xmlChar * id);

          std::string getContent(xmlNodePtr cur);

          void getTable(xmlNodePtr cur, PCRManager::PCRPORMap & map);

          void getDataRate(xmlNodePtr cur, PCRManager::PCRPORMap & map);

          void getRows(xmlNodePtr cur, PCRManager::PCREntryVector & vec);

          void interpolate();

          const NEMId id_;

          PlatformServiceProvider * pPlatformService_;

          PCRPORMap pcrPorMap_;

          size_t tablePacketSize_;
        };
      }
   }
}


#endif //EMANEMODELSIEEE802ABGIEEE80211MACPCRMANAGER_HEADER_
