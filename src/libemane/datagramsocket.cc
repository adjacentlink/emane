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

#include "datagramsocket.h"
#include "socketexception.h"
#include <cstring>
#include <sys/socket.h>

EMANE::DatagramSocket::DatagramSocket(){}

EMANE::DatagramSocket::DatagramSocket(const INETAddr & address,
                                      bool bReuseAddress)
{
  open(address,
       bReuseAddress);
}

void EMANE::DatagramSocket::open(const INETAddr & address,
                                 bool bReuseAddress)
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

EMANE::DatagramSocket::~DatagramSocket(){}

int EMANE::DatagramSocket::getHandle()
{
  return iSock_;
}


ssize_t EMANE::DatagramSocket::send(const iovec *iov,
                                    int iovcnt,
                                    const INETAddr & remoteAddress,
                                    int flags) const
{
  msghdr msg;
  memset(&msg,0,sizeof(msg));

  msg.msg_iov = const_cast<iovec *>(iov);
  msg.msg_iovlen = iovcnt;

  msg.msg_name = remoteAddress.getSockAddr();
  msg.msg_namelen = remoteAddress.getAddrLength();

  return sendmsg(iSock_,&msg,flags);
}

ssize_t EMANE::DatagramSocket::recv(void * buf,
                                    size_t len,
                                    int flags)
{
  return ::recv(iSock_,buf,len,flags);
}

EMANE::INETAddr EMANE::DatagramSocket::getLocalAddress() const
{
  if(addr_.isIPv4())
    {
      sockaddr_in addr;
      socklen_t len = sizeof(addr);

      if(getsockname(iSock_, reinterpret_cast<sockaddr*>(&addr), &len))
        {
          throw makeException<SocketException>("getsockname: %s",
                                               strerror(errno));
        }

      return INETAddr{addr};
    }
  else
    {
      sockaddr_in6 addr;
      socklen_t len = sizeof(addr);

      if(getsockname(iSock_, reinterpret_cast<sockaddr*>(&addr), &len))
        {
          throw makeException<SocketException>("getsockname: %s",
                                               strerror(errno));
        }

      return INETAddr{addr};
    }
}
