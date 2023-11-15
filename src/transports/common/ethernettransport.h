/*
 * Copyright (c) 2013,2016,2023 - Adjacent Link LLC, Bridgewater,
 *  New Jersey
 * Copyright (c) 2009-2012 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef ETHERNETTRANSPORT_HEADER_
#define ETHERNETTRANSPORT_HEADER_

#include "emane/transport.h"
#include "emane/utils/netutils.h"

#include <mutex>
#include <map>

namespace EMANE
{
  namespace Transports
  {
    namespace Ethernet
    {
      /**
       * @class EthernetTransport
       *
       * @brief EMANE Ethernet Transport
       *
       */
      class EthernetTransport : public EMANE::Transport
      {
      public:
        EthernetTransport(EMANE::NEMId id, EMANE::PlatformServiceProvider *pPlatformService);

        ~EthernetTransport();

      protected:
        virtual int parseFrame(const Utils::EtherHeader *pEthHeader,
                               EMANE::NEMId & dst,
                               std::uint8_t & dscp);

        virtual int verifyFrame(const void * buf, size_t len);

        void updateArpCache(const Utils::EtherHeader *pEthHeader, EMANE::NEMId nemId);

        void addEntry(const Utils::EtherAddr& addr, EMANE::NEMId nemId);

        EMANE::NEMId lookupArpCache(const Utils::EtherAddr *pEthAddr);

        bool bBroadcastMode_;
        bool bArpCacheMode_;
        std::uint8_t u8EtherTypeARPPriority_;

        using UnknownEtherTypePriorityMap = std::map<std::uint16_t,std::uint8_t>;
        UnknownEtherTypePriorityMap unknownEtherTypePriorityMap_;

      private:
        std::mutex mutex_;

        /**
         * @brief EtherAddr compare operator
         *
         */
        struct ltmacaddr
        {
          /**
           * @param a1  ether addr 1
           * @param a2  ether addr 2
           *
           * @return returns true if a1 is less than a2, else returns false
           *
           */

          bool operator()(const Utils::EtherAddr& a1, const Utils::EtherAddr& a2) const
          {
            return memcmp(&a1, &a2, Utils::ETH_ALEN) < 0;
          }
        };

        using EthAddrMap = std::map<Utils::EtherAddr, EMANE::NEMId, ltmacaddr>;

        EthAddrMap macCache_;
      };
    }
  }
}

#endif //ETHERNETTRANSPORT_HEADER_
