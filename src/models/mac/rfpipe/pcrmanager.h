/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
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


#ifndef RFPIPEMAC_PCRMANAGER_HEADER_
#define RFPIPEMAC_PCRMANAGER_HEADER_

#include "emane/types.h"
#include "emane/startexception.h"
#include "emane/platformserviceprovider.h"

#include <libxml/parser.h>

#include <string>
#include <vector>
#include <map>

namespace EMANE
{
  namespace Models
  {
    namespace RFPipe
    {
 
      /**
       *
       * @class  PCRManager 
       *
       * @brief  Manages the PCR curves
       *
       */
      class PCRManager 
      {

        /**
         *
         * @brief Defines a PCR entry (sinr and por)
         *
         */
        struct PCREntry {
          float fSINR_;
          float fPOR_;

          PCREntry () :
            fSINR_{},
            fPOR_{}
          { }

          PCREntry (float fSinr, float fPor) :
            fSINR_{fSinr},
            fPOR_{fPor}
          { }
        };

        typedef std::vector<PCREntry>    PCREntryVector;
        typedef PCREntryVector::iterator PCREntryVectorIter;

        typedef std::vector<float>  PORVector;
        typedef PORVector::iterator PORVectorIter;


      public:
        PCRManager(EMANE::NEMId id, EMANE::PlatformServiceProvider * pPlatformService);

        ~PCRManager();

        /**
         *
         * loads the PCR curve entries
         *
         * @param uri  location of the PCR file
         *
         */
        void load(const std::string &uri);

        /**
         *  get the PCR for a given sinr and pkt size
         * 
         *  @param fSinr the signal to noise ratio
         *  @param size the packet size in bytes, units (bytes) must match the units provided in the pcr curve file.
         *
         *  @return the PCR value
         */
        float getPCR(float fSinr, size_t size);
       
      private:
        void Open(const std::string & uri, xmlParserCtxtPtr * ppContext, xmlDoc ** ppDocument, xmlNode ** ppRoot);

        void Close(xmlParserCtxtPtr * ppContext, xmlDoc ** ppDocument);

        std::string getAttribute (xmlNodePtr cur, const xmlChar * id);

        std::string getContent (xmlNodePtr cur);

        void getTable (xmlNodePtr cur);

        void getRows (xmlNodePtr cur);

        void interpolate();

        const EMANE::NEMId id_;

        EMANE::PlatformServiceProvider * pPlatformService_;

        PCREntryVector pcrEntryVector_;

        PORVector porVector_;

        size_t tablePacketSize_;
      };
    }
  }
}


#endif //RFPIPEMAC_PCRMANAGER_HEADER_
