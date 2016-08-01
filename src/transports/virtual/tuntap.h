/*
 * Copyright (c) 2014-2016 - Adjacent Link LLC, Bridgewater, New
 * Jersey
 * Copyright (c) 2008 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef TUNTAP_HEADER_
#define TUNTAP_HEADER_

#include "emane/inetaddr.h"
#include <string>

#include "emane/platformserviceprovider.h"
#include "emane/utils/netutils.h"


#include <linux/if_tun.h>

#undef ETH_ALEN
#undef ETH_P_ARP
#undef ETH_P_IPV6

#define NETWORK_DEVICE_PATH  "/dev/net/tun"

// local sockaddr
struct sockaddr_in_t
{
  short            sin_family;   // e.g. AF_INET, AF_INET6
  unsigned short   sin_port;     // e.g. htons(3490)
  struct in_addr   sin_addr;     // see struct in_addr, below
  char             sin_zero[8];  // zero this if you want to
} __attribute__((__may_alias__));



namespace EMANE
{
  namespace Transports
  {
    namespace Virtual
    {
      class TunTap
      {
      private:

        PlatformServiceProvider * pPlatformService_;
        int                   tunHandle_;
        std::string           tunName_;
        std::string           tunPath_;
        std::string           tunGuid_;
        int                   tunIndex_;
        INETAddr              tunAddr_;
        INETAddr              tunMask_;


        int set_flags(int, int);
        int get_flags();

      public:
        TunTap(PlatformServiceProvider * pPlatformService);

        ~TunTap();

        /**
         * Opens tuntap handle
         *
         * @param sDevicePath path to device
         * @param sDeviceName name of device
         *
         * @return 0 on success, -1 on error
         */
        int open(const char *, const char *);

        /**
         * Closes tuntap handle
         *
         * @return 0
         */
        int close();

        /**
         * Activates tuntap
         *
         * @param arpEnabled enable or disable arp
         * @return 0 on success, -1 on error.
         *
         * @note sets the device to the UP state.
         */
        int activate(bool);

        /**
         * Deactivates tuntap
         *
         * @return 0 on success, -1 on error.
         *
         * @note sets the device to the DOWN state.
         */
        int deactivate();

        /**
         * Gets tuntap handle
         *
         * @return tuntap handle.
         */
        int get_handle();

        /**
         * Reads from tuntap
         *
         * @param iov iovector
         * @param iov_len number of elements in the iovector
         *
         * @return total number of bytes read
         */
        int readv(struct iovec*, size_t);

        /**
         * Writes to tuntap
         *
         * @param iov iovector
         * @param iov_len number of elements in the iovector
         *
         * @return total number of bytes written
         */
        int writev(const struct iovec*, size_t);

        /**
         * Sets tuntap interface address and netmask
         *
         * ipv4 addr "192.168.1.1"    mask "255.255.255.0"
         * ipv6 addr "::192.168.1.1"  mask "ffff:ffff:ffff:ffff::"
         *
         * @param addr tun address ipv4 or ipv6 address
         * @param mask tun netmask used as an ipv4 or ipv6 netmask
         *
         * @return 0 on success, -1 on error.
         */
        int set_addr(const INETAddr &, const INETAddr &);

        /**
         * Sets tuntap eth address
         *
         * @param ethAddr eth address
         *
         * @return 0 on success, -1 on error.
         */
        int set_ethaddr(const Utils::EtherAddr &);

        /**
         * Sets tuntap eth address
         *
         * @param id nem id
         *
         * @return 0 on success, -1 on error.
         */
        int set_ethaddr(NEMId);

        /**
         * Gets the tuntap ip address
         *
         * @return tuntap ip address
         */
        INETAddr & get_addr();

        /**
         * Gets the tuntap netmask
         *
         * @return tuntap netmask
         */
        INETAddr & get_mask();
      };
    }
  }
}

#endif // TUNTAP_HEADER_
