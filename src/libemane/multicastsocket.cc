/*
 * Copyright (c) 2015 - Adjacent Link LLC, Bridgewater, New Jersey
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
 * * Neither the name of Adjacent Link LLC nor the names of its
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

#include "multicastsocket.h"
#include "socketexception.h"
#include <cstring>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>

EMANE::MulticastSocket::MulticastSocket(){}

EMANE::MulticastSocket::MulticastSocket(const INETAddr & address,
                                        bool bReuseAddress,
                                        const std::string & sDevice,
                                        std::uint8_t u8TTL,
                                        bool bLoop)
{
  open(address,
       bReuseAddress,
       sDevice,
       u8TTL,
       bLoop);
}

void EMANE::MulticastSocket::open(const INETAddr & address,
                                  bool bReuseAddress,
                                  const std::string & sDevice,
                                  std::uint8_t u8TTL,
                                  bool bLoop)
{
  if(iSock_ != -1)
    {
      close();
    }

  addr_ = address;

  if((iSock_ = socket(address.getFamily(),
                      SOCK_DGRAM,
                      0)) == -1)
    {
      throw SocketException(strerror(errno));
    }

  // set ipv4 or ipv6 options
  if(address.isIPv4())
    {
      if(setsockopt(iSock_,
                    IPPROTO_IP,IP_MULTICAST_TTL,
                    (void*)&u8TTL,
                    sizeof(u8TTL)) < 0)
        {
          throw makeException<SocketException>("setsockopt IP_MULTICAST_TTL: %s",
                                               strerror(errno));
        }


      if(bLoop)
        {
          std::uint8_t u8Option = 1;

          // set the multicast loopback
          if(setsockopt(iSock_,IPPROTO_IP,IP_MULTICAST_LOOP,
                        &u8Option,
                        sizeof(u8Option)) < 0)
            {
              throw makeException<SocketException>("setsockopt IP_MULTICAST_LOOP: %s",
                                                   strerror(errno));
            }
        }

      // multicast group info used for join
      ip_mreq mreq;

      memset(&mreq,0,sizeof(mreq));

      memcpy(&mreq.imr_multiaddr,
             &reinterpret_cast<sockaddr_in *>(address.getSockAddr())->sin_addr,
             sizeof(mreq.imr_multiaddr));

      if(!sDevice.empty())
        {
          ifreq ifr;
          memset(&ifr,0,sizeof(ifr));
          strncpy(ifr.ifr_name,sDevice.c_str(),IFNAMSIZ);

          // get the ip address
          if(ioctl(iSock_,SIOCGIFADDR,&ifr) < 0)
            {
              throw makeException<SocketException>("ioctl SIOCGIFADDR: %s",
                                                   strerror(errno));
            }

          // set the multicast interface
          if(setsockopt(iSock_,
                        IPPROTO_IP,
                        IP_MULTICAST_IF,
                        &reinterpret_cast<sockaddr_in*>(&ifr.ifr_addr)->sin_addr.s_addr,
                        sizeof(in_addr)) < 0)
            {
              throw makeException<SocketException>("setsockopt IP_MULTICAST_IF: %s",
                                                   strerror(errno));
            }

          mreq.imr_interface.s_addr =
            reinterpret_cast<sockaddr_in*>(&ifr.ifr_addr)->sin_addr.s_addr;
        }
      else
        {
          mreq.imr_interface.s_addr = INADDR_ANY;
        }

      if(setsockopt(iSock_,
                    IPPROTO_IP,
                    IP_ADD_MEMBERSHIP,
                    reinterpret_cast<void*>(&mreq),
                    sizeof(mreq)) < 0)
        {
          throw makeException<SocketException>("setsockopt IP_ADD_MEMBERSHIP: %s",
                                               strerror(errno));
        }
    }

  else if(address.isIPv6())
    {
      int iOption{u8TTL};

      if(setsockopt(iSock_,
                    IPPROTO_IPV6,
                    IPV6_MULTICAST_HOPS,
                    &iOption,
                    sizeof(iOption)) < 0)
        {
          throw makeException<SocketException>("setsockopt IPV6_MULTICAST_HOPS: %s",
                                               strerror(errno));
        }

      if(bLoop)
        {
          iOption = 1;

          if(setsockopt(iSock_,
                        IPPROTO_IPV6,
                        IPV6_MULTICAST_LOOP,
                        &iOption,
                        sizeof(iOption)) < 0)
            {
              throw makeException<SocketException>("setsockopt IPV6_MULTICAST_LOOP: %s",
                                                   strerror(errno));
            }
        }

      ipv6_mreq mreq6;

      memset(&mreq6,0,sizeof(mreq6));

      memcpy(&mreq6.ipv6mr_multiaddr,
             &reinterpret_cast<sockaddr_in6 *>(address.getSockAddr())->sin6_addr,
             sizeof(mreq6.ipv6mr_multiaddr));

      mreq6.ipv6mr_interface = 0;

      if(!sDevice.empty())
        {
          unsigned int iIndex{if_nametoindex(sDevice.c_str())};

          if(setsockopt(iSock_,
                        IPPROTO_IPV6,
                        IPV6_MULTICAST_IF,
                        &iIndex,
                        sizeof(iIndex)) < 0)
            {
              throw makeException<SocketException>("setsockopt IPV6_MULTICAST_IF: %s",
                                                   strerror(errno));
            }

          mreq6.ipv6mr_interface = iIndex;
        }

      if(setsockopt(iSock_,
                    IPPROTO_IPV6,
                    IPV6_ADD_MEMBERSHIP,
                    reinterpret_cast<void *>(&mreq6),
                    sizeof(mreq6)) < 0)
        {
          throw makeException<SocketException>("setsockopt IPV6_ADD_MEMBERSHIP: %s",
                                               strerror(errno));
        }
    }
  else
    {
      throw makeException<SocketException>("Unknown address family");
    }

  if(bReuseAddress)
    {
      int iOption{1};

      if(setsockopt(iSock_,
                    SOL_SOCKET,
                    SO_REUSEADDR,
                    reinterpret_cast<void*>(&iOption),
                    sizeof(iOption)) < 0)
        {
          throw makeException<SocketException>("setsockopt SO_REUSEADDR: %s",
                                               strerror(errno));

        }
    }


  if(bind(iSock_,addr_.getSockAddr(),addr_.getAddrLength()) < 0)
    {
      throw makeException<SocketException>("bind: %s",
                                           strerror(errno));
    }

}

EMANE::MulticastSocket::~MulticastSocket(){}


ssize_t EMANE::MulticastSocket::send(const iovec *iov, int iovcnt, int flags) const
{
  msghdr msg;
  memset(&msg,0,sizeof(msg));

  msg.msg_iov = const_cast<iovec *>(iov);
  msg.msg_iovlen = iovcnt;

  msg.msg_name = addr_.getSockAddr();
  msg.msg_namelen =  addr_.getAddrLength();

  return sendmsg(iSock_,&msg,flags);
}

ssize_t EMANE::MulticastSocket::recv(void * buf,
                                    size_t len,
                                    int flags)
{
  return ::recv(iSock_,buf,len,flags);
}
