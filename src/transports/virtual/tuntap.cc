/*
 * Copyright (c) 2013-2016 - Adjacent Link LLC, Bridgewater, New Jersey
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


#include "tuntap.h"

#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include <unistd.h>
#include <fcntl.h>

/*
  Create device node: mknod /dev/net/tun c 10 200
  Add following line to the /etc/modules.conf: alias char-major-10-200 tun
  Run: depmod -a
  Driver will be automatically loaded when application access /dev/net/tun.
*/


/**
 *
 * @class TunTap
 *
 * @brief Tuntap networkdevice service provider.
 *
 */
EMANE::Transports::Virtual::TunTap::TunTap(PlatformServiceProvider * pPlatformService):
  pPlatformService_(pPlatformService),
  tunHandle_(-1),
  tunName_(""),
  tunPath_(""),
  tunIndex_(-1)
{}


/**
 *
 * @brief destructor
 *
 */
EMANE::Transports::Virtual::TunTap::~TunTap()
{}



int
EMANE::Transports::Virtual::TunTap::open(const char *sDevicePath, const char *sDeviceName)
{
  int result;

  // open tuntap device
  if((tunHandle_ = ::open(sDevicePath, O_RDWR)) == -1)
    {
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              ABORT_LEVEL,
                              "TunTap::%s:open:error %s",
                              __func__,
                              strerror(errno));

      // fail
      return -1;
    }

  // interface info
  struct ifreq ifr;

  // clear ifr
  memset(&ifr, 0, sizeof(ifr));

  // copy dev name
  strncpy(ifr.ifr_name, sDeviceName, sizeof(ifr.ifr_name)-1);

  // set flags no proto info and tap mode
  ifr.ifr_flags = IFF_NO_PI | IFF_TAP;

  // set tun flags
  if(ioctl(tunHandle_, TUNSETIFF, &ifr) < 0)
    {
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              ABORT_LEVEL,
                              "TunTap::%s:ioctl:error %s",
                              __func__,
                              strerror(errno));

      // fail
      return -1;
    }

  // clear ifr
  memset(&ifr, 0, sizeof(ifr));

  // copy dev name
  strncpy(ifr.ifr_name, sDeviceName, sizeof(ifr.ifr_name)-1);

  int ctrlsock = socket(AF_INET,SOCK_DGRAM,0);

  // get iff index
  if(ioctl(ctrlsock, SIOCGIFINDEX, &ifr) < 0)
    {
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              ABORT_LEVEL,
                              "TunTap::%s:getindex:error %s",
                              __func__,
                              strerror(errno));

      // fail
      return -1;
    }

  // save the dev path, name, guid and index
  tunPath_  = sDevicePath;
  tunName_  = sDeviceName;
  tunGuid_  = "n/a";
  tunIndex_ = ifr.ifr_ifindex;

  // close control socket
  ::close(ctrlsock);

  // success
  result = 0;


  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "TunTap::%s, path %s, name %s, guid %s, index %d",
                          __func__,
                          tunPath_.c_str(),
                          tunName_.c_str(),
                          tunGuid_.c_str(),
                          tunIndex_);

  // return result
  return result;
}


int EMANE::Transports::Virtual::TunTap::close()
{
  ::close(tunHandle_);

  // return 0
  return 0;
}


int EMANE::Transports::Virtual::TunTap::get_handle()
{
  return tunHandle_;
}


int
EMANE::Transports::Virtual::TunTap::activate(bool arpEnabled)
{
  // set if flags
  int flags = IFF_UP;

  if(arpEnabled == false)
    {
      flags |= IFF_NOARP;
    }

  return set_flags(flags, 1);
}


int
EMANE::Transports::Virtual::TunTap::deactivate()
{
  return set_flags(IFF_UP, -1);
}


int
EMANE::Transports::Virtual::TunTap::set_addr(const INETAddr & addr, const INETAddr & mask)
{
  // save of copy of the addr and mask in string format
  std::string sAddress{addr.str(false)};
  std::string sNetMask{mask.str(false)};

  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "TunTap::%s, type %s, addr %s, mask %s", __func__,
                          addr.isIPv4() ? "ipv4" :
                          addr.isIPv6() ? "ipv6" : "?",
                          addr.str().c_str(),
                          mask.str().c_str());

  // addr type ipv4
  if(addr.isIPv4())
    {
      // interface info
      struct ifreq ifr;

      // clear ifr
      memset(&ifr, 0, sizeof(ifr));

      // copy dev name
      strncpy(ifr.ifr_name, tunName_.c_str(), sizeof(ifr.ifr_name)-1);

      // copy addr and family
      ((struct sockaddr_in_t *) &ifr.ifr_addr)->sin_family = AF_INET;

      ((struct sockaddr_in_t *) &ifr.ifr_addr)->sin_addr.s_addr =
        reinterpret_cast<sockaddr_in *>(addr.getSockAddr())->sin_addr.s_addr;

      // open ipv4 control socket
      int ctrlsock = socket(AF_INET,SOCK_DGRAM,0);

      // set addr
      if(ioctl(ctrlsock, SIOCSIFADDR, &ifr) < 0)
        {
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  ABORT_LEVEL,
                                  "TunTap::%s:setaddr:error %s",
                                  __func__,
                                  strerror(errno));
          return -1;
        }
      else
        {
          // save tun addr
          tunAddr_ = addr;
        }

      // clear ifr
      memset(&ifr, 0, sizeof(ifr));

      // copy dev name
      strncpy(ifr.ifr_name, tunName_.c_str(), sizeof(ifr.ifr_name)-1);

      // copy addr and family
      ((struct sockaddr_in_t *) &ifr.ifr_addr)->sin_family = AF_INET;
      ((struct sockaddr_in_t *) &ifr.ifr_addr)->sin_addr.s_addr =
        reinterpret_cast<sockaddr_in *>(mask.getSockAddr())->sin_addr.s_addr;

      // set netmask
      if(ioctl(ctrlsock, SIOCSIFNETMASK, &ifr) < 0)
        {
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  ABORT_LEVEL,
                                  "TunTap::%s:setmask:error %s",
                                  __func__,
                                  strerror(errno));

          // fail
          return -1;
        }
      else
        {
          // save tun mask
          tunMask_ = mask;
        }

      // close control socket
      ::close(ctrlsock);
    }
  // addr type ipv6
  else if(addr.isIPv6())
    {
      // interface info ipv6
      struct in6_ifreq
      {
        struct in6_addr ifr6_addr;
        std::uint32_t ifr6_prefixlen;
        std::uint32_t ifr6_ifindex;
      } ifr6;

      // ipv6 prefix
      struct in6_addr in6_prefix;

      // clear ifr6
      memset(&ifr6, 0, sizeof(ifr6));

      // copy index
      ifr6.ifr6_ifindex = tunIndex_;

      // copy mask
      if(inet_pton(AF_INET6, sNetMask.c_str(), &in6_prefix) < 0)
        {
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  ABORT_LEVEL,
                                  "TunTap::%s:copyprefix:error %s",
                                  __func__,
                                  strerror(errno));
          return -1;
        }

      // copy prefix len
      ifr6.ifr6_prefixlen = Utils::get_prefixlen(&in6_prefix);

      // copy address
      if(inet_pton(AF_INET6, sAddress.c_str(), &ifr6.ifr6_addr) < 0)
        {
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  ABORT_LEVEL,
                                  "TunTap::%s:copyaddr:error %s",
                                  __func__,
                                  strerror(errno));
          return -1;
        }

      int ctrlsock = socket(AF_INET6,SOCK_DGRAM,0);

      // open ipv6 control socket
      // set addr
      if(ioctl(ctrlsock, SIOCSIFADDR, &ifr6) < 0)
        {
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  ABORT_LEVEL,
                                  "TunTap::%s:setaddr:error %s",
                                  __func__,
                                  strerror(errno));
          return -1;
        }
      else
        {
          // save tun addr
          tunAddr_ = addr;

          // save tun mask
          tunMask_ = mask;
        }

      // close control socket
      ::close(ctrlsock);
    }


  // success
  return 0;
}


int EMANE::Transports::Virtual::TunTap::set_ethaddr(NEMId id)
{
  // eth addr
  Utils::EtherAddr ethAddr;

  // locally administered 02:02:00:00:XX:XX
  ethAddr.words.word1 = htons(0x0202);
  ethAddr.words.word2 = htons(0x0000);
  ethAddr.words.word3 = htons(id);

  return set_ethaddr(ethAddr);
}

int EMANE::Transports::Virtual::TunTap::set_ethaddr(const Utils::EtherAddr & ethAddr)
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "TunTap::%s:%s",
                          __func__,
                          addr_to_string(&ethAddr));

  // interface info
  struct ifreq ifr;

  // clear ifr
  memset(&ifr, 0, sizeof(ifr));

  // copy dev name
  strncpy(ifr.ifr_name, tunName_.c_str(), sizeof(ifr.ifr_name));

  // copy hwaddr
  memcpy(ifr.ifr_hwaddr.sa_data, &ethAddr, Utils::ETH_ALEN);

  // copy hwaddr family
  ifr.ifr_hwaddr.sa_family = Utils::ARPHRD_ETHER;

  // open ipv4 control socket
  int ctrlsock = socket(AF_INET,SOCK_DGRAM,0);

  // set hw addr
  if(ioctl(ctrlsock, SIOCSIFHWADDR, &ifr) < 0)
    {
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              ABORT_LEVEL,
                              "TunTap::%s:sethwaddr:error %s",
                              __func__,
                              strerror(errno));

      // fail
      return -1;
    }

  // close control socket
  ::close(ctrlsock);


  // success
  return 0;
}



int EMANE::Transports::Virtual::TunTap::writev(const struct iovec *iov, size_t iov_len)
{
  int result;

  result = ::writev(tunHandle_, iov, iov_len);

  // check result
  if(result < 0)
    {
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              ABORT_LEVEL,
                              "TunTap::%s:write:error %s",
                              __func__,
                              strerror(errno));
    }

  // return result
  return result;
}



int EMANE::Transports::Virtual::TunTap::readv(struct iovec *iov, size_t iov_len)
{
  int result;

  result = ::readv(tunHandle_, iov, iov_len);

  // check result
  if(result < 0)
    {
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              ABORT_LEVEL,
                              "TunTap::%s:read:error %s",
                              __func__,
                              strerror(errno));
    }

  // return result
  return result;
}


EMANE::INETAddr & EMANE::Transports::Virtual::TunTap::get_addr()
{
  // return tun address
  return tunAddr_;
}


EMANE::INETAddr & EMANE::Transports::Virtual::TunTap::get_mask()
{
  // return tun address
  return tunMask_;
}



// private methods
int EMANE::Transports::Virtual::TunTap::set_flags(int newflags, int cmd)
{
  // interface info
  struct ifreq ifr;

  // clear ifr
  memset(&ifr, 0, sizeof(ifr));

  // copy dev name
  strncpy(ifr.ifr_name, tunName_.c_str(), IFNAMSIZ-1);

  // add flags
  if(cmd > 0)
    {
      ifr.ifr_flags =(get_flags() | newflags);
    }
  // del flags
  else if(cmd < 0)
    {
      ifr.ifr_flags =(get_flags() & ~newflags);
    }
  // set flags
  else
    {
      ifr.ifr_flags = newflags;
    }

  // open ipv4 control socket
  int ctrlsock = socket(AF_INET,SOCK_DGRAM,0);

  // set flags
  if(ioctl(ctrlsock, SIOCSIFFLAGS, &ifr) < 0)
    {
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              ABORT_LEVEL,
                              "TunTap::%s:setflags:error %s",
                              __func__,
                              strerror(errno));

      // fail
      return -1;
    }

  // close control socket
  ::close(ctrlsock);

  // success
  return 0;
}



int EMANE::Transports::Virtual::TunTap::get_flags()
{
  // interface info
  struct ifreq ifr;

  // clear ifr
  memset(&ifr, 0, sizeof(ifr));

  // copy dev name
  strncpy(ifr.ifr_name, tunName_.c_str(), sizeof(ifr.ifr_name)-1);

  // open ipv4 control socket
  int ctrlsock = socket(AF_INET,SOCK_DGRAM,0);

  // get flags
  if(ioctl(ctrlsock, SIOCGIFFLAGS, &ifr) < 0)
    {
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              ABORT_LEVEL,
                              "TunTap::%s:getflags:error %s",
                              __func__,
                              strerror(errno));

      // fail
      return -1;
    }

  // close control socket
  ::close(ctrlsock);

  // return flags
  return ifr.ifr_flags;
}
