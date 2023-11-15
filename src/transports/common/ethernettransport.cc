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

#include "ethernettransport.h"
#include "emane/componenttypes.h"


EMANE::Transports::Ethernet::EthernetTransport::EthernetTransport(NEMId id,
                                                                  PlatformServiceProvider *pPlatformService):
  Transport(id, pPlatformService),
  bBroadcastMode_(false),
  bArpCacheMode_(true),
  u8EtherTypeARPPriority_{}
{ }


EMANE::Transports::Ethernet::EthernetTransport::~EthernetTransport()
{ }


int EMANE::Transports::Ethernet::EthernetTransport::verifyFrame(const void * buf, size_t len)
{
   // check min header len
   if(len < Utils::ETH_HEADER_LEN)
     {
        LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                ERROR_LEVEL,
                               "TRANSPORTI %03d EthernetTransport::%s len %zd < min eth header len %d",
                                id_,
                                __func__,
                                len,
                                Utils::ETH_HEADER_LEN);

        // error
        return -1;
     }
   else
     {
       // eth header
       const Utils::EtherHeader *pEthHeader = (Utils::EtherHeader *) buf;

       // eth protocol
       const std::uint16_t u16ethProtocol = Utils::get_protocol(pEthHeader);

       // reduce length by eth hdr len
       len -= Utils::ETH_HEADER_LEN;

       switch(u16ethProtocol)
        {
          // eth ipv4
          case Utils::ETH_P_IPV4:
          {
            // check min len
            if(len < Utils::IPV4_HEADER_LEN)
             {
               LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                       ERROR_LEVEL,
                                       "TRANSPORTI %03d EthernetTransport::%s ipv4, len %zu < min len %d",
                                       id_,
                                       __func__,
                                       len,
                                       Utils::IPV4_HEADER_LEN);

               // error
               return -1;
             }
            else
             {
               // success
               return 0;
             }
          }

         // eth ipv6
         case Utils::ETH_P_IPV6:
          {
            // check min len
            if(len < Utils::IPV6_HEADER_LEN)
             {
                LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                        ERROR_LEVEL,
                                        "TRANSPORTI %03d EthernetTransport::%s ipv6, len %zu < min len %d",
                                        id_,
                                        __func__,
                                        len,
                                        Utils::IPV6_HEADER_LEN);

                // error
                return -1;
             }
           else
             {
               // success
               return 0;
             }
          }

         // eth arp
         case Utils::ETH_P_ARP:
          {
            // check min len
            if(len < Utils::ETHARP_HEADER_LEN)
             {
               LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                       ERROR_LEVEL, "TRANSPORTI %03d EthernetTransport::%s arp, len %zu < len %d",
                                       id_,
                                       __func__,
                                       len,
                                       Utils::ETHARP_HEADER_LEN);

               // error
               return -1;
             }
            else
             {
               // success
               return 0;
             }
          }

        // unknown protocol
        default:
          LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                 DEBUG_LEVEL, "TRANSPORTI %03d EthernetTransport::%s allow unknown protocol %02X",
                                 id_,
                                 __func__,
                                 u16ethProtocol);

          // but not an error
          return 1;
      }
   }
}



int EMANE::Transports::Ethernet::EthernetTransport::parseFrame(const Utils::EtherHeader *pEthHeader,
                                                               NEMId & rNemDestination,
                                                               std::uint8_t & rDspc)
{
   // eth protocol
  const std::uint16_t u16ethProtocol = Utils::get_protocol(pEthHeader);

   switch(u16ethProtocol)
   {
     // eth ipv4
     case Utils::ETH_P_IPV4:
       {
         // ipv4 header
         const Utils::Ip4Header *pIpHeader = (Utils::Ip4Header*) ((Utils::EtherHeader*) pEthHeader + 1);

         // broadcast always mode
         if(bBroadcastMode_)
           {
             rNemDestination = NEM_BROADCAST_MAC_ADDRESS;
           }
         // check arp cache
         else if (bArpCacheMode_)
           {
             rNemDestination = lookupArpCache(&pEthHeader->dst);
           }
         // use ether dst
         else
           {
             rNemDestination = Utils::ethaddr4_to_id(&pEthHeader->dst);
           }

         // set the dscp based on ip header
         rDspc = Utils::get_dscp(pIpHeader);

         // success
         return 0;
       }

     // eth ipv6
     case Utils::ETH_P_IPV6:
       {
         // ipv6 header
         const Utils::Ip6Header *pIpHeader = (Utils::Ip6Header*) ((Utils::EtherHeader*) pEthHeader + 1);

         // broadcast always mode
         if(bBroadcastMode_)
           {
             rNemDestination = NEM_BROADCAST_MAC_ADDRESS;
           }
         // check arp cache
         else if (bArpCacheMode_)
           {
             rNemDestination = lookupArpCache(&pEthHeader->dst);
           }
         // use ether dst
         else
           {
             rNemDestination = Utils::ethaddr6_to_id(&pEthHeader->dst);
           }

         // set the dscp based on ip header
         rDspc = Utils::get_dscp(pIpHeader);

         // success
         return 0;
       }

     // eth arp
     case Utils::ETH_P_ARP:
       {
         // broadcast always mode
         if(bBroadcastMode_)
           {
             rNemDestination = NEM_BROADCAST_MAC_ADDRESS;
           }
         // check arp cache
         else if (bArpCacheMode_)
           {
             rNemDestination = lookupArpCache(&pEthHeader->dst);
           }
         // use ether dst
         else
           {
             rNemDestination = Utils::ethaddr4_to_id(&pEthHeader->dst);
           }

         // use configured arp priority value, default 0
         rDspc = u8EtherTypeARPPriority_;

         // success
         return 0;
       }

     // unknown protocol
     default:
       LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                              DEBUG_LEVEL,
                              "TRANSPORTI %03d EthernetTransport::%s allow unknown protocol %02X",
                              id_,
                              __func__,
                              u16ethProtocol);

       // broadcast always mode
       if(bBroadcastMode_)
         {
           rNemDestination = NEM_BROADCAST_MAC_ADDRESS;
         }
       // check arp cache
       else if (bArpCacheMode_)
         {
           rNemDestination = lookupArpCache(&pEthHeader->dst);
         }
       // use the last 2 bytes of the ethernet destination
       else
         {
           rNemDestination = ntohs(pEthHeader->dst.words.word3);
         }

       // check if unknown ethertype has a configured priority
       if(auto iter = unknownEtherTypePriorityMap_.find(u16ethProtocol);
          iter != unknownEtherTypePriorityMap_.end())
         {
           rDspc = iter->second;
         }
       else
         {
           // set dscp to 0
           rDspc = 0;
         }

       // but not an error
       return 1;
   }
}



void EMANE::Transports::Ethernet::EthernetTransport::updateArpCache(const Utils::EtherHeader *pEthHeader, NEMId nemId)
{
   // not needed in broadcast mode
   if(bBroadcastMode_)
    {
      return;
    }
   // not needed if arp cache disabled
   else if (! bArpCacheMode_)
    {
      return;
    }
   else
    {

      // eth protocol
      const std::uint16_t u16ethProtocol = Utils::get_protocol(pEthHeader);

      switch(u16ethProtocol)
      {
        // eth arp
        case Utils::ETH_P_ARP:
          {
            const Utils::EtherArpHeader *pEtherArpHeader = (Utils::EtherArpHeader*) ((Utils::EtherHeader*) pEthHeader + 1);

            const std::uint16_t u16code = Utils::get_code(pEtherArpHeader);

            // arp type reply or request
            if((u16code == Utils::ETH_ARPOP_REPLY) || (u16code == Utils::ETH_ARPOP_REQUEST))
             {
               addEntry(*Utils::get_srchwaddr(pEtherArpHeader), nemId);
             }
          }
        break;

        case Utils::ETH_P_IPV6:
          {
            const Utils::Ip6Header *pIp6Header = (Utils::Ip6Header*) ((Utils::EtherHeader*) pEthHeader + 1);

            // check for icmpv6
            if (pIp6Header->u8Ipv6next == Utils::IPV6_P_ICMP)
             {
               const Utils::IP6ICMPHeader *pICMP6Header = (Utils::IP6ICMPHeader*) ((Utils::Ip6Header*) pIp6Header + 1);

               const std::uint8_t icmpv6Type = pICMP6Header->u8Type;

               // icmpv6 neighbor solicitation or advertisement
               if ((icmpv6Type == Utils::IP6_ICMP_NEIGH_SOLICIT) || (icmpv6Type == Utils::IP6_ICMP_NEIGH_ADVERT))
                {
                  addEntry(pEthHeader->src, nemId);
                }
             }
          }
        break;
      }
   }
}

void EMANE::Transports::Ethernet::EthernetTransport::addEntry(const Utils::EtherAddr& addr, NEMId nemId)
{
  // lock mutex
  std::lock_guard<std::mutex> m(mutex_);

  const auto iter = macCache_.find(addr);

  // new entry
  if(iter == macCache_.end())
    {
      macCache_.insert(std::make_pair(addr, nemId));

      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                             DEBUG_LEVEL,
                             "TRANSPORTI %03d ARPCache::%s added cache entry %s to nem %hu",
                             id_,
                             __func__,
                             ethaddr_to_string(&addr).c_str(), nemId);
    }
  else
    {
      // entry found but different nem
      if(iter->second != nemId)
       {
         LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                DEBUG_LEVEL,
                                "TRANSPORTI %03d ARPCache::%s updated cache entry %s from nem %hu to nem %hu",
                                id_,
                                __func__,
                                ethaddr_to_string(&addr).c_str(),
                                iter->second, nemId);
         // updated nem id
         iter->second = nemId;
       }
    }
}


EMANE::NEMId EMANE::Transports::Ethernet::EthernetTransport::lookupArpCache(const Utils::EtherAddr *pEtherAddr)
{
   // lock mutex
  std::lock_guard<std::mutex> m(mutex_);

   const auto iter = macCache_.find(*pEtherAddr);

   // entry not found, most likely broadcast
   if(iter == macCache_.end())
    {
      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                             DEBUG_LEVEL,
                             "TRANSPORTI %03d EthernetTransport::%s no nem found for %s, using broadcast mac address",
                             id_,
                             __func__,
                             Utils::ethaddr_to_string(pEtherAddr).c_str());

      return NEM_BROADCAST_MAC_ADDRESS;
    }
   else
    {
      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                             DEBUG_LEVEL,
                             "TRANSPORTI %03d EthernetTransport::%s nem %hu found for %s, using %hu",
                             id_,
                             __func__,
                             iter->second,
                             Utils::ethaddr_to_string(pEtherAddr).c_str(),
                             iter->second);

      return iter->second;
    }
}
