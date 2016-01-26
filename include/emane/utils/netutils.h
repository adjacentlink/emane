/*
 * Copyright (c) 2013,2016 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2008-2010 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANEUTILSNETUTILS_HEADER_
#define EMANEUTILSNETUTILS_HEADER_

#include <sstream>
#include <vector>
#include <cstdint>
#include <cstring>
#include <arpa/inet.h>

namespace EMANE
{
  namespace Utils
  {
    /**
     * @struct UdpHeader
     *
     * @brief Definition of the UDP header
     *
     */

    struct UdpHeader
    {
      std::uint16_t u16Udpsrc;
      std::uint16_t u16Udpdst;
      std::uint16_t u16Udplen;
      std::uint16_t u16Udpcheck;
    } __attribute__((packed));


    /**
     *
     * @brief ip udp protocol
     *
     */
    const std::uint8_t IP_PROTO_UDP = 0x11;

    /**
     *
     * @brief udp header len
     *
     */
    const std::uint16_t UDP_HEADER_LEN = 8;


    /**
     * @param vhl ip version/header len value
     *
     * @return returns the ip header version
     *
     */
    inline std::uint8_t
    get_ip_version(std::uint8_t vhl)
    {
      return ((vhl >> 4) & 0x0F);
    }

    /**
     * @param vhl ip version/header len value
     *
     * @return returns the ip header length
     *
     */
    inline std::uint8_t
    get_ip_hdrlen(std::uint8_t vhl)
    {
      return ((vhl & 0x0F) << 2);
    }


    /**
     *
     * @param tos ip tos value
     *
     * @return returns ipv4 dscp value
     *
     */
    inline std::uint8_t
    get_ip_dscp(std::uint8_t tos)
    {
      return (tos >> 2);
    }

    /**
     *
     * @param ver ip version
     * @param clas ip traffic class
     *
     * @return returns ipv6 dscp value
     *
     */

    inline std::uint8_t
    get_ip_dscp(std::uint8_t ver, std::uint8_t clas)
    {
      return (((ver & 0x0F) << 2) | (clas >> 6));
    }


    /**
     *
     * @struct Ip4Header
     *
     * @brief Definition of the IPv4 header
     *
     */
    struct Ip4Header
    {
      std::uint8_t  u8Ipv4vhl;
      std::uint8_t  u8Ipv4tos;
      std::uint16_t u16Ipv4len;
      std::uint16_t u16Ipv4id;
      std::uint16_t u16Ipv4frag;
      std::uint8_t  u8Ipv4hops;
      std::uint8_t  u8Ipv4proto;
      std::uint16_t u16Ipv4check;
      std::uint32_t u32Ipv4src;
      std::uint32_t u32Ipv4dst;
    } __attribute__((packed));

    /**
     *
     * @brief ipv4 header len without options
     *
     */
    const std::uint16_t IPV4_HEADER_LEN = 20;


    /**
     *
     * @brief ipv4 addr len
     *
     */
    const std::uint16_t IPV4_ADDR_LEN = 4;


    /**
     *
     * @brief ipv6 addr len
     *
     */
    const std::uint16_t IPV6_ADDR_LEN = 16;

    /**
     *
     * @brief IPv6 ICMP Protocol
     *
     */
    const std::uint8_t IPV6_P_ICMP = 0x3A;

    /**
     *
     * @param ip pointer to an ipv4 header
     *
     * @return returns the ip header version (hopefully version 4)
     *
     */
    inline std::uint8_t
    get_version(const Ip4Header *ip)
    {
      return get_ip_version(ip->u8Ipv4vhl);
    }


    /**
     *
     * @param ip pointer to an ipv4 header
     *
     * @return returns the ipv4 header length with options in bytes
     *
     */
    inline std::uint8_t
    get_hdrlen(const Ip4Header *ip)
    {
      return get_ip_hdrlen(ip->u8Ipv4vhl);
    }


    /**
     *
     * @param ip pointer to an ipv4 header
     *
     * @return returns the total ipv4 packet length
     *
     */
    inline std::uint16_t
    get_len(const Ip4Header *ip)
    {
      return ntohs(ip->u16Ipv4len);
    }


    /**
     *
     * @param ip pointer to an ipv4 header
     *
     * @return returns the dscp value
     *
     */
    inline std::uint8_t
    get_dscp(const Ip4Header *ip)
    {
      return get_ip_dscp(ip->u8Ipv4tos);
    }

    /**
     *
     * @param  addr pointer to the address buffer
     * @param  addrlen length of the address buffer
     * @param  delim the requested delimeter
     * @param  buf the buffer to write the formatted output to
     * @param  buflen the length of the output buffer
     *
     */
    inline void
    addr_to_string(const std::uint8_t * addr, size_t addrlen, const char *delim, char *buf, size_t buflen)
    {
      size_t pos = 0;

      for(size_t i = 0; i < addrlen; ++i)
        {
          if(i < (addrlen - 1))
            {
              pos += snprintf(buf + pos, buflen - pos, "%02X%s", addr[i] & 0xFF, delim);
            }
          else
            {
              pos += snprintf(buf + pos, buflen - pos, "%02X", addr[i] & 0xFF);
            }
        }
    }


    /**
     *
     * @struct Ip6Header
     *
     * @brief Definition of the IPv6 header
     *
     */
    struct Ip6Header
    {
      struct {
        std::uint8_t   u8Ipv6Ver;
        std::uint8_t   u8Ipv6Class;
        std::uint16_t  u16Ipv6Flow;
      } vcf;

      std::uint16_t u16Ipv6len;
      std::uint8_t  u8Ipv6next;
      std::uint8_t  u8Ipv6hops;
      std::uint8_t  Ipv6src[IPV6_ADDR_LEN];
      std::uint8_t  Ipv6dst[IPV6_ADDR_LEN];
    } __attribute__((packed));

    /**
     *
     * @brief ipv6 header len
     *
     */
    const std::uint16_t IPV6_HEADER_LEN = 40;

    /**
     *
     * @brief ipv6 max addr len
     *
     */
    const std::uint16_t IPV6_MAX_BIT_LEN = 128;



    inline std::uint8_t
    get_version(const Ip6Header *ip)
    {
      return get_ip_version(ip->vcf.u8Ipv6Ver);
    }

    inline std::uint8_t
    get_hdrlen(const Ip6Header *ip)
    {
      return get_ip_hdrlen(ip->vcf.u8Ipv6Ver);
    }

    inline std::uint16_t
    get_len(const Ip6Header *ip)
    {
      return (ntohs(ip->u16Ipv6len) + IPV6_HEADER_LEN);
    }

    inline std::uint8_t
    get_dscp(const Ip6Header *ip)
    {
      return get_ip_dscp(ip->vcf.u8Ipv6Ver, ip->vcf.u8Ipv6Class);
    }



    /**
     *
     * @brief Ethernet hardware address length
     *
     */
    const std::uint16_t ETH_ALEN = 6;


    /**
     *
     * @brief Ethernet header length.
     *
     */
    const std::uint16_t ETH_HEADER_LEN = 14;


    /**
     *
     * @brief Ethernet Arp header length.
     *
     */
    const std::uint16_t ETHARP_HEADER_LEN = 28;

    /**
     *
     * @brief Max ip packet len
     *
     */
    const std::uint16_t IP_MAX_PACKET = 0xFFFF;


    /**
     *
     * @brief Ethernet hardware type
     *
     */
    const std::uint16_t ARPHRD_ETHER = 0x0001;


    /**
     *
     * @brief Ethernet ipv4 protocol
     *
     */
    const std::uint16_t ETH_P_IPV4 = 0x0800;


    /**
     *
     * @brief Ethernet arp protocol
     *
     */
    const std::uint16_t ETH_P_ARP = 0x0806;

    /**
     *
     * @brief Ethernet ipv6 protocol
     *
     */
    const std::uint16_t ETH_P_IPV6 = 0x86DD;

    /**
     *
     * @struct EtherAddrBytes
     *
     * @brief Definition of the ethernet frame address as an array of 6 bytes
     *
     */
    struct  EtherAddrBytes{
      std::uint8_t  buff[ETH_ALEN];
    } __attribute__((packed));

    /**
     *
     * @struct EtherAddrWords
     *
     * @brief Definition of the ethernet frame address as a set of 3 words
     *
     */
    struct EtherAddrWords{
      std::uint16_t word1;
      std::uint16_t word2;
      std::uint16_t word3;
    } __attribute__((packed));

    /**
     *
     * @union EtherAddr
     *
     * @brief Definition of the ethernet frame address as an array of 6 bytes or set of 3 words
     *
     */
    union EtherAddr {
      struct EtherAddrBytes bytes;
      struct EtherAddrWords words;
    } __attribute__((packed));

    /**
     *
     * @struct EtherHeader
     *
     * @brief Definition of the ethernet frame header
     *
     */
    struct  EtherHeader{
      union  EtherAddr dst;
      union  EtherAddr src;
      std::uint16_t u16proto;
    } __attribute__((packed));


    /**
     *
     * @param eth a pointer to an  EtherHeader
     *
     * @return returns a pointer the src EtherAddr
     *
     */
    inline const EtherAddr*
    get_srcaddr(const EtherHeader *eth)
    {
      return (&eth->src);
    }

    /**
     *
     * @param eth a pointer to an  EtherHeader
     *
     * @return returns a pointer the dst EtherAddr
     *
     */
    inline const EtherAddr*
    get_dstaddr(const EtherHeader *eth)
    {
      return (&eth->dst);
    }

    /**
     *
     * @param eth a pointer to an  EtherHeader
     *
     * @return returns the ether protocol id in host byte order
     *
     */
    inline std::uint16_t
    get_protocol(const EtherHeader *eth)
    {
      return ntohs(eth->u16proto);
    }



    /**
     *
     * @param addr a pointer to an  EtherAddr
     *
     * @return returns the formatted address string
     *
     */
    inline const char *
    addr_to_string(const EtherAddr * addr)
    {
      static char buf[64];

      addr_to_string((std::uint8_t*) addr, ETH_ALEN, ":", buf, sizeof(buf));

      return buf;
    }


    /**
     *
     * @param addr a pointer to an  EtherAddr
     *
     * @return returns the formatted address string
     *
     */
    inline std::string
    ethaddr_to_string(const EtherAddr * addr)
    {
      char buf[64];

      addr_to_string((std::uint8_t*) addr, ETH_ALEN, ":", buf, sizeof(buf));

      return buf;
    }



    /**
     *
     * @struct EtherArpHeader
     *
     * @brief Definition of the ethernet arp header.
     *
     */
    struct EtherArpHeader
    {
      std::uint16_t        u16hwType;
      std::uint16_t        u16protocol;
      std::uint8_t         u8hwAddrLen;
      std::uint8_t         u8protoAddrLen;
      std::uint16_t        u16code;
      EtherAddr         srcHwAddr;
      std::uint32_t        u32srcProtoAddr;
      EtherAddr         dstHwAddr;
      std::uint32_t        u32dstProtoAddr;
    } __attribute__((packed));

    /**
     *
     * @param arp a pointer to an  EtherArpHeader
     *
     * @return returns the hardware type
     *
     */
    inline std::uint16_t
    get_hwtype(const EtherArpHeader *arp)
    {
      return ntohs(arp->u16hwType);
    }


    /**
     *
     * @param arp a pointer to an  EtherArpHeader
     *
     * @return returns the protocol id in host byte order
     *
     */
    inline std::uint16_t
    get_protocol(const EtherArpHeader *arp)
    {
      return ntohs(arp->u16protocol);
    }


    /**
     *
     * @param arp a pointer to an  EtherArpHeader
     *
     * @return returns the hardware address len
     *
     */
    inline std::uint8_t
    get_hwaddrlen(const EtherArpHeader *arp)
    {
      return (arp->u8hwAddrLen);
    }


    /**
     *
     * @param arp a pointer to an  EtherArpHeader
     *
     * @return returns the protocol addresss len
     *
     */
    inline std::uint8_t
    get_protoaddrlen(const EtherArpHeader *arp)
    {
      return (arp->u8protoAddrLen);
    }

    /**
     *
     * @param arp a pointer to an  EtherArpHeader
     *
     * @return returns the arp code in host byte order
     *
     */
    inline std::uint16_t
    get_code(const EtherArpHeader *arp)
    {
      return ntohs(arp->u16code);
    }


    /**
     *
     * @param arp a pointer to an  EtherArpHeader
     *
     * @return returns the src (requestor) hardware address
     *
     */
    inline const EtherAddr*
    get_srchwaddr(const EtherArpHeader *arp)
    {
      return (&arp->srcHwAddr);
    }


    /**
     *
     * @param arp a pointer to an  EtherArpHeader
     *
     * @return returns the destination (responder) hardware address
     *
     */
    inline const EtherAddr*
    get_dsthwaddr(const EtherArpHeader *arp)
    {
      return (&arp->dstHwAddr);
    }

    /**
     *
     * @param arp a pointer to an  EtherArpHeader
     *
     * @return returns the src (requestor) protocol address
     *
     */
    inline std::uint32_t
    get_srcprotoaddr(const EtherArpHeader *arp)
    {
      return (arp->u32srcProtoAddr);
    }


    /**
     *
     * @param arp a pointer to an  EtherArpHeader
     *
     * @return returns the (responder) destination protocol address
     *
     */
    inline std::uint32_t
    get_dstprotoaddr(const EtherArpHeader *arp)
    {
      return (arp->u32dstProtoAddr);
    }


    /**
     *
     * @brief Ethernet arp request
     *
     */
    const std::uint16_t ETH_ARPOP_REQUEST = 0x0001;


    /**
     *
     * @brief Ethernet arp reply
     *
     */
    const std::uint16_t ETH_ARPOP_REPLY = 0x0002;



    /**
     *
     * @param addr a pointer to an ipv4 EtherAddr
     *
     * @return returns the derived NEM id
     *
     */
    inline std::uint16_t
    ethaddr4_to_id (const EtherAddr *addr)
    {
      // multicast mac
      static std::uint8_t MulticastMacAddr[3] = {0x01, 0x00, 0x5E};

      // broadcast mac
      static std::uint8_t BroadcastMacAddr[ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

      // multicast
      if(memcmp(addr->bytes.buff, MulticastMacAddr, 3) == 0)
        {
          return ntohs(0xFFFF);
        }
      // broadcast
      else if(memcmp(addr->bytes.buff, BroadcastMacAddr, 6) == 0)
        {
          return ntohs(0xFFFF);
        }
      // unicast
      else
        {
          return ntohs(addr->words.word3);
        }
    }


    /**
     *
     * @param addr a pointer to an ipv6 EtherAddr
     *
     * @return returns the derived NEM id
     *
     */
    inline std::uint16_t
    ethaddr6_to_id (const EtherAddr *addr)
    {
      // multicast mac
      static std::uint8_t MulticastMacAddr[2] = {0x33, 0x33};

      // broadcast mac
      static std::uint8_t BroadcastMacAddr[ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

      // multicast
      if(memcmp(addr->bytes.buff, MulticastMacAddr, 2) == 0)
        {
          return ntohs(0xFFFF);
        }
      // broadcast
      else if(memcmp(addr->bytes.buff, BroadcastMacAddr, 6) == 0)
        {
          return ntohs(0xFFFF);
        }
      // unicast
      else
        {
          return ntohs(addr->words.word3);
        }
    }

    /**
     *
     * @param prefix a pointer to a inet 6 address
     *
     * @return returns the prefix length in bits
     *
     */
    inline std::uint8_t
    get_prefixlen (const in6_addr *prefix)
    {
      std::uint8_t len = 0;
      std::uint8_t *u8ptr = (std::uint8_t *) prefix;

      while ((*u8ptr == 0xff) && len < IPV6_MAX_BIT_LEN)
        {
          len += 8;
          u8ptr++;
        }

      if (len < IPV6_MAX_BIT_LEN)
        {
          std::uint8_t byte = *u8ptr;
          while (byte)
            {
              len++;
              byte <<= 1;
            }
        }

      return len;
    }


    /**
     *
     * @param buf pointer to data buffer
     * @param buflen length of buffer
     * @param prev running sum
     *
     * @return returns the 16 bit one’s complement of the one’s complement
     *         sum of all 16 bit words in the buffer
     *
     */

    inline std::uint16_t
    inet_cksum (const void * buf, int buflen, std::uint16_t prev = 0)
    {
      std::uint32_t sum = 0;
      std::uint16_t *w = (std::uint16_t *) buf;

      while (buflen > 1) {
        sum += *w;
        buflen -= 2;
        w++;
      }

      if (buflen) {
        std::uint8_t *byte = (std::uint8_t *) w;
        sum += *byte;
      }

      sum += prev;
      sum = (sum >> 16) + (sum & 0xFFFF);
      return (sum + (sum >> 16));
    }

    /**
     * @struct IP6ICMPHeader
     *
     * @brief Definition of the IPv6 ICMP header.
     *
     */
    struct IP6ICMPHeader {
      std::uint8_t    u8Type;
      std::uint8_t    u8Code;
      std::uint16_t   u16Checksum;
      std::uint32_t   u32Flags;
      sockaddr_in6 targetAddr;
    } __attribute__((packed));

    /**
     *
     * @brief IPv6 ICMP Neighbor Soliciation
     *
     */
    const std::uint8_t IP6_ICMP_NEIGH_SOLICIT = 135;

    /**
     *
     * @brief IPv6 ICMP Neighbor Advertisement
     *
     */
    const std::uint8_t IP6_ICMP_NEIGH_ADVERT = 136;

    /**
     * @param val the frequency in Hz
     * @param precision the format precision
     *
     * @brief  returns a sting with the formatted value and suffix
     *
     */

    inline std::string formatFrequency(const std::uint64_t val, const int precision = 6)
    {
      std::stringstream ss;

      ss.precision(precision);

      const float fFrequency = val;

      if(fFrequency < 1e+3)
        {
          ss << fFrequency << " Hz";
        }
      else if (fFrequency < 1e+6)
        {
          ss << (fFrequency / 1e+3) << " KHz";
        }
      else if (fFrequency < 1e+9)
        {
          ss << (fFrequency / 1e+6) << " MHz";
        }
      else if (fFrequency < 1e+12)
        {
          ss << (fFrequency / 1e+9) << " GHz";
        }
      else
        {
          ss << (fFrequency / 1e+12) << " THz";
        }

      return ss.str();
    }

    inline std::vector<std::string> getTokens(const std::string & sInput, const char * pzDelimeter)
    {
      char *pzToken = NULL;

      int iTokenCount = 0;

      std::string str = sInput;

      std::vector<std::string> strVector;

      while ((pzToken = strtok (iTokenCount == 0 ? &str[0] : NULL, pzDelimeter)) != NULL)
        {
          ++iTokenCount;

          strVector.push_back(pzToken);
        }

      return strVector;
    }
  }
}

#endif // EMANEUTILSNETUTILS_HEADER_
