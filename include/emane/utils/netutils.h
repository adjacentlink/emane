/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include <ace/INET_Addr.h>
#include <ace/OS_NS_string.h>
#include <sstream>
#include <vector>

namespace EMANE
{
  namespace Utils
  {
    /**
     * @union INET_Addr
     *
     * @brief Definition of an ipv4/ipv6 internet address 
     *
     */
    union INET_Addr
    {
      sockaddr_in  in4_;
#if defined (ACE_HAS_IPV6)
      sockaddr_in6 in6_;
#endif /* ACE_HAS_IPV6 */
    };

    // Return @c true if the IP address is IPv4/IPv6 multicast address.
    // Internal version of is_multicast due to bug in at least ACE version 5.6.2
    // See doc directory for available patches.
    inline bool
    is_multicast (const ACE_INET_Addr & addr)
    {
      INET_Addr *p = (INET_Addr*) addr.get_addr();
#if defined (ACE_HAS_IPV6)
      if (addr.get_type() == AF_INET6)
        return p->in6_.sin6_addr.s6_addr[0] == 0xFF;
#endif /* ACE_HAS_IPV6 */
      return
        ACE_NTOHL(p->in4_.sin_addr.s_addr) >= 0xE0000000 &&  // 224.0.0.0
        ACE_NTOHL(p->in4_.sin_addr.s_addr) <= 0xEFFFFFFF;    // 239.255.255.255
    }

    /**
     * @struct UdpHeader
     *
     * @brief Definition of the UDP header
     *
     */

    struct UdpHeader
    {
      ACE_UINT16 u16Udpsrc;
      ACE_UINT16 u16Udpdst;
      ACE_UINT16 u16Udplen;
      ACE_UINT16 u16Udpcheck;
    } __attribute__((packed));


    /**
     *
     * @brief ip udp protocol
     *
     */
    const ACE_UINT8 IP_PROTO_UDP = 0x11;

    /**
     *
     * @brief udp header len
     *
     */
    const ACE_UINT16 UDP_HEADER_LEN = 8;


    /**
     * @param vhl ip version/header len value
     *
     * @return returns the ip header version
     *
     */
    inline ACE_UINT8 
    get_ip_version(ACE_UINT8 vhl)
    {
      return ((vhl >> 4) & 0x0F);
    }

    /**
     * @param vhl ip version/header len value
     *
     * @return returns the ip header length
     *
     */
    inline ACE_UINT8 
    get_ip_hdrlen(ACE_UINT8 vhl)
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
    inline ACE_UINT8 
    get_ip_dscp(ACE_UINT8 tos)
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

    inline ACE_UINT8 
    get_ip_dscp(ACE_UINT8 ver, ACE_UINT8 clas)
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
      ACE_UINT8  u8Ipv4vhl;
      ACE_UINT8  u8Ipv4tos;
      ACE_UINT16 u16Ipv4len;
      ACE_UINT16 u16Ipv4id;
      ACE_UINT16 u16Ipv4frag;
      ACE_UINT8  u8Ipv4hops;
      ACE_UINT8  u8Ipv4proto;
      ACE_UINT16 u16Ipv4check;
      ACE_UINT32 u32Ipv4src;
      ACE_UINT32 u32Ipv4dst;
    } __attribute__((packed));

    /**
     *
     * @brief ipv4 header len without options
     *
     */
    const ACE_UINT16 IPV4_HEADER_LEN = 20;


    /**
     *
     * @brief ipv4 addr len 
     *
     */
    const ACE_UINT16 IPV4_ADDR_LEN = 4;


    /**
     *
     * @brief ipv6 addr len 
     *
     */
    const ACE_UINT16 IPV6_ADDR_LEN = 16;

    /**
     *
     * @brief IPv6 ICMP Protocol
     *
     */
    const ACE_UINT8 IPV6_P_ICMP = 0x3A;

    /**
     *
     * @param ip pointer to an ipv4 header
     *
     * @return returns the ip header version (hopefully version 4)
     *
     */
    inline ACE_UINT8
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
    inline ACE_UINT8
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
    inline ACE_UINT16
    get_len(const Ip4Header *ip)
    {
      return ACE_NTOHS(ip->u16Ipv4len); 
    }


    /**
     *
     * @param ip pointer to an ipv4 header
     *
     * @return returns the dscp value
     *
     */
    inline ACE_UINT8 
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
    addr_to_string(const ACE_UINT8 * addr, size_t addrlen, const char *delim, char *buf, size_t buflen)
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
        ACE_UINT8   u8Ipv6Ver;
        ACE_UINT8   u8Ipv6Class;
        ACE_UINT16  u16Ipv6Flow;
      } vcf;

      ACE_UINT16 u16Ipv6len;
      ACE_UINT8  u8Ipv6next;
      ACE_UINT8  u8Ipv6hops;
      ACE_UINT8  Ipv6src[IPV6_ADDR_LEN];
      ACE_UINT8  Ipv6dst[IPV6_ADDR_LEN];
    } __attribute__((packed));

    /**
     *
     * @brief ipv6 header len
     *
     */
    const ACE_UINT16 IPV6_HEADER_LEN = 40;

    /**
     *
     * @brief ipv6 max addr len
     *
     */
    const ACE_UINT16 IPV6_MAX_BIT_LEN = 128;



    inline ACE_UINT8
    get_version(const Ip6Header *ip)
    {
      return get_ip_version(ip->vcf.u8Ipv6Ver);
    }

    inline ACE_UINT8
    get_hdrlen(const Ip6Header *ip)
    {
      return get_ip_hdrlen(ip->vcf.u8Ipv6Ver);
    }

    inline ACE_UINT16
    get_len(const Ip6Header *ip)
    {
      return (ACE_NTOHS(ip->u16Ipv6len) + IPV6_HEADER_LEN);
    }

    inline ACE_UINT8 
    get_dscp(const Ip6Header *ip)
    {
      return get_ip_dscp(ip->vcf.u8Ipv6Ver, ip->vcf.u8Ipv6Class);
    }



    /**
     *
     * @brief Ethernet hardware address length
     *
     */
    const ACE_UINT16 ETH_ALEN = 6;


    /**
     *
     * @brief Ethernet header length.
     *
     */
    const ACE_UINT16 ETH_HEADER_LEN = 14;


    /**
     *
     * @brief Ethernet Arp header length.
     *
     */
    const ACE_UINT16 ETHARP_HEADER_LEN = 28;

    /**
     *
     * @brief Max ip packet len
     *
     */
    const ACE_UINT16 IP_MAX_PACKET = 0xFFFF;


    /**
     *
     * @brief Ethernet hardware type
     *
     */
    const ACE_UINT16 ARPHRD_ETHER = 0x0001;


    /**
     *
     * @brief Ethernet ipv4 protocol
     *
     */
    const ACE_UINT16 ETH_P_IPV4 = 0x0800;


    /**
     *
     * @brief Ethernet arp protocol
     *
     */
    const ACE_UINT16 ETH_P_ARP = 0x0806;

    /**
     *
     * @brief Ethernet ipv6 protocol
     *
     */
    const ACE_UINT16 ETH_P_IPV6 = 0x86DD;

    /**
     *
     * @struct EtherAddrBytes
     *
     * @brief Definition of the ethernet frame address as an array of 6 bytes
     *
     */
    struct  EtherAddrBytes{
      ACE_UINT8  buff[ETH_ALEN];
    } __attribute__((packed));

    /**
     *
     * @struct EtherAddrWords
     *
     * @brief Definition of the ethernet frame address as a set of 3 words
     *
     */
    struct EtherAddrWords{
      ACE_UINT16 word1;
      ACE_UINT16 word2;
      ACE_UINT16 word3;
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
      ACE_UINT16 u16proto;
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
    inline ACE_UINT16
    get_protocol(const EtherHeader *eth)
    {
      return ACE_NTOHS(eth->u16proto);
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

      addr_to_string((ACE_UINT8*) addr, ETH_ALEN, ":", buf, sizeof(buf));

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

      addr_to_string((ACE_UINT8*) addr, ETH_ALEN, ":", buf, sizeof(buf));

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
      ACE_UINT16        u16hwType;
      ACE_UINT16        u16protocol;
      ACE_UINT8         u8hwAddrLen;
      ACE_UINT8         u8protoAddrLen;
      ACE_UINT16        u16code; 
      EtherAddr         srcHwAddr;
      ACE_UINT32        u32srcProtoAddr;
      EtherAddr         dstHwAddr;
      ACE_UINT32        u32dstProtoAddr;
    } __attribute__((packed));

    /**
     *
     * @param arp a pointer to an  EtherArpHeader 
     *
     * @return returns the hardware type
     *
     */
    inline ACE_UINT16
    get_hwtype(const EtherArpHeader *arp)
    {
      return ACE_NTOHS(arp->u16hwType);
    }


    /**
     *
     * @param arp a pointer to an  EtherArpHeader 
     *
     * @return returns the protocol id in host byte order
     *
     */
    inline ACE_UINT16
    get_protocol(const EtherArpHeader *arp)
    {
      return ACE_NTOHS(arp->u16protocol);
    }


    /**
     *
     * @param arp a pointer to an  EtherArpHeader 
     *
     * @return returns the hardware address len
     *
     */
    inline ACE_UINT8
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
    inline ACE_UINT8
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
    inline ACE_UINT16
    get_code(const EtherArpHeader *arp)
    {
      return ACE_NTOHS(arp->u16code);
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
    inline ACE_UINT32
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
    inline ACE_UINT32
    get_dstprotoaddr(const EtherArpHeader *arp)
    {
      return (arp->u32dstProtoAddr);
    }


    /**
     *
     * @brief Ethernet arp request
     *
     */
    const ACE_UINT16 ETH_ARPOP_REQUEST = 0x0001;


    /**
     *
     * @brief Ethernet arp reply
     *
     */
    const ACE_UINT16 ETH_ARPOP_REPLY = 0x0002;



    /**
     *
     * @param addr a pointer to an ipv4 EtherAddr
     *
     * @return returns the derived NEM id
     *
     */
    inline ACE_UINT16 
    ethaddr4_to_id (const EtherAddr *addr)
    {
      // multicast mac
      static ACE_UINT8 MulticastMacAddr[3] = {0x01, 0x00, 0x5E};

      // broadcast mac
      static ACE_UINT8 BroadcastMacAddr[ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

      // multicast
      if(ACE_OS::memcmp(addr->bytes.buff, MulticastMacAddr, 3) == 0) 
        {
          return ACE_NTOHS(0xFFFF);
        }
      // broadcast
      else if(ACE_OS::memcmp(addr->bytes.buff, BroadcastMacAddr, 6) == 0) 
        {
          return ACE_NTOHS(0xFFFF);
        }
      // unicast
      else 
        { 
          return ACE_NTOHS(addr->words.word3);
        }
    }


    /**
     *
     * @param addr a pointer to an ipv6 EtherAddr
     *
     * @return returns the derived NEM id
     *
     */
    inline ACE_UINT16 
    ethaddr6_to_id (const EtherAddr *addr)
    {
      // multicast mac
      static ACE_UINT8 MulticastMacAddr[2] = {0x33, 0x33};

      // broadcast mac
      static ACE_UINT8 BroadcastMacAddr[ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

      // multicast
      if(ACE_OS::memcmp(addr->bytes.buff, MulticastMacAddr, 2) == 0) 
        {
          return ACE_NTOHS(0xFFFF);
        }
      // broadcast
      else if(ACE_OS::memcmp(addr->bytes.buff, BroadcastMacAddr, 6) == 0) 
        {
          return ACE_NTOHS(0xFFFF);
        }
      // unicast
      else 
        { 
          return ACE_NTOHS(addr->words.word3);
        }
    }

    /**
     *
     * @param prefix a pointer to a inet 6 address
     *
     * @return returns the prefix length in bits
     *
     */
    inline ACE_UINT8
    get_prefixlen (const in6_addr *prefix)
    {
      ACE_UINT8 len = 0;
      ACE_UINT8 *u8ptr = (ACE_UINT8 *) prefix;

      while ((*u8ptr == 0xff) && len < IPV6_MAX_BIT_LEN)
        {
          len += 8;
          u8ptr++;
        }

      if (len < IPV6_MAX_BIT_LEN)
        {
          ACE_UINT8 byte = *u8ptr;
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

    inline ACE_UINT16
    inet_cksum (const void * buf, int buflen, ACE_UINT16 prev = 0)
    {
      ACE_UINT32 sum = 0;
      ACE_UINT16 *w = (ACE_UINT16 *) buf;

      while (buflen > 1) {
        sum += *w;
        buflen -= 2;
        w++;
      }

      if (buflen) {
        ACE_UINT8 *byte = (ACE_UINT8 *) w;
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
      ACE_UINT8    u8Type;
      ACE_UINT8    u8Code;
      ACE_UINT16   u16Checksum;
      ACE_UINT32   u32Flags;
      sockaddr_in6 targetAddr;
    } __attribute__((packed));

    /**
     *
     * @brief IPv6 ICMP Neighbor Soliciation
     *
     */
    const ACE_UINT8 IP6_ICMP_NEIGH_SOLICIT = 135;
  
    /**
     *
     * @brief IPv6 ICMP Neighbor Advertisement
     *
     */
    const ACE_UINT8 IP6_ICMP_NEIGH_ADVERT = 136;

    /**
     * @param val the frequency in Hz
     * @param precision the format precision
     *
     * @brief  returns a sting with the formatted value and suffix
     *
     */

    inline std::string formatFrequency(const ACE_UINT64 val, const int precision = 6)
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
 
      while ((pzToken = ACE_OS::strtok (iTokenCount == 0 ? &str[0] : NULL, pzDelimeter)) != NULL) 
        {
          ++iTokenCount;

          strVector.push_back(pzToken);
        }

      return strVector;
    }
  }
}

#endif // EMANEUTILSNETUTILS_HEADER_
