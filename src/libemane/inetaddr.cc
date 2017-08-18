/*
 * Copyright (c) 2015-2016 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "emane/inetaddr.h"
#include "emane/inetaddrexception.h"
#include "emane/utils/parameterconvert.h"

#include <string>
#include <memory>
#include <algorithm>

#include <netdb.h>
#include <net/if.h>
#include <arpa/inet.h>

class EMANE::INETAddr::Implementation
{
public:
  Implementation():
    Implementation{"0.0.0.0",0}{}

  Implementation(const std::string & sEndPoint):
    length_{},
    iFamily_{AF_UNSPEC},
    u16Port_{}
  {
    set(sEndPoint);
  }

  Implementation(const std::string & sAddress,
                 std::uint16_t u16Port):
    pSockAddr_{},
    length_{},
    iFamily_{AF_UNSPEC},
    u16Port_{}
  {
    set(sAddress,u16Port);
  }

  Implementation(const sockaddr_in & sockaddr):
    pSockAddr_{reinterpret_cast<struct sockaddr*>(new sockaddr_in)},
    length_{sizeof(sockaddr_in)},
    iFamily_{AF_INET},
    u16Port_{}
  {
    memcpy(pSockAddr_.get(),
           &sockaddr,
           sizeof(sockaddr));

    u16Port_ = reinterpret_cast<sockaddr_in*>(pSockAddr_.get())->sin_port;
  }

  Implementation(const sockaddr_in6 & sockaddr):
    pSockAddr_{reinterpret_cast<struct sockaddr*>(new sockaddr_in6)},
    length_{sizeof(sockaddr_in6)},
    iFamily_{AF_INET6},
    u16Port_{}
  {
    memcpy(pSockAddr_.get(),
           &sockaddr,
           sizeof(sockaddr));

    u16Port_ = reinterpret_cast<sockaddr_in6*>(pSockAddr_.get())->sin6_port;
  }

  void set(const std::string & sAddress,std::uint16_t u16Port)
  {
    addrinfo * pAddrInfo{};

    addrinfo hints;
    memset(&hints,0,sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = 0;

    int iRet{};

    // lookup the ipv4 or ipv6 address info
    if((iRet = getaddrinfo(sAddress.c_str(),
                           std::to_string(u16Port).c_str(),
                           &hints,
                           &pAddrInfo)) != 0)
      {
        throw INETAddrException(gai_strerror(iRet));
      }

    if(pAddrInfo->ai_family == AF_INET)
      {
        pSockAddr_.reset(reinterpret_cast<sockaddr*>(new sockaddr_in));
        memcpy(pSockAddr_.get(),
               reinterpret_cast<sockaddr_in*>(pAddrInfo->ai_addr),
               sizeof(sockaddr_in));
      }
    else if(pAddrInfo->ai_family == AF_INET6)
      {
        pSockAddr_.reset(reinterpret_cast<sockaddr*>(new sockaddr_in6));
        memcpy(pSockAddr_.get(),
               reinterpret_cast<sockaddr_in6*>(pAddrInfo->ai_addr),
               sizeof(sockaddr_in6));
      }
    else
      {
        freeaddrinfo(pAddrInfo);
        throw INETAddrException("Unknown family");
      }

    iFamily_ = pAddrInfo->ai_family;
    length_ = pAddrInfo->ai_addrlen;
    u16Port_ = u16Port;

    freeaddrinfo(pAddrInfo);
  }

  void set(const std::string & sEndPoint)
  {
    if(sEndPoint.empty())
      {
        throw INETAddrException("Empty endpoint string");
      }

    size_t pos = sEndPoint.rfind("/");

    if(pos == std::string::npos)
      {
        // only used ':' as a port seperator for
        // hostname or IPv4 addresses
        if(std::count(sEndPoint.begin(),
                      sEndPoint.end(),
                      ':') == 1 ||
           sEndPoint.front() == '[')
          {
            pos  = sEndPoint.rfind(":");
          }
      }

    std::string sAddress{sEndPoint};
    std::uint16_t u16Port{};

    if(pos != std::string::npos)
      {
        sAddress = sEndPoint.substr(0,pos);
        u16Port = Utils::ParameterConvert(sEndPoint.substr(pos+1)).toUINT16();
      }


    if(!sAddress.empty())
      {
        if(sAddress.front() == '[' &&
           sAddress.back() == ']')
          {
            sAddress = sAddress.substr(1,sAddress.size() - 2);
          }

      }

    set(sAddress,u16Port);
  }


  Implementation(const Implementation & address)
  {
    copy(address);
  }

  Implementation & operator=(const Implementation & address)
  {
    copy(address);
    return *this;
  }

  bool isIPv6() const
  {
    return iFamily_ == AF_INET6;
  }

  bool isIPv4() const
  {
    return iFamily_ == AF_INET;
  }

  int getFamily() const
  {
    return iFamily_;
  }

  bool isAny() const
  {
    if(isIPv4())
      {
        return reinterpret_cast<sockaddr_in*>(pSockAddr_.get())->sin_addr.s_addr == INADDR_ANY;
      }
    else
      {
        return IN6_IS_ADDR_UNSPECIFIED(&reinterpret_cast<sockaddr_in6 *>(pSockAddr_.get())->sin6_addr);
      }
  }


  bool isMulticast() const
  {
    if(isIPv6())
      {
        return reinterpret_cast<sockaddr_in6 *>(pSockAddr_.get())->sin6_addr.s6_addr[0] == 0xFF;
      }
    else
      {
        return ntohl(reinterpret_cast<sockaddr_in*>(pSockAddr_.get())->sin_addr.s_addr) >= 0xE0000000 &&  // 224.0.0.0
          ntohl(reinterpret_cast<sockaddr_in*>(pSockAddr_.get())->sin_addr.s_addr) <= 0xEFFFFFFF;    // 239.255.255.255
      }
  }

  sockaddr * getSockAddr() const
  {
    return pSockAddr_.get();
  }

  socklen_t getAddrLength() const
  {
    return length_;
  }

  std::uint16_t getPort() const
  {
    return u16Port_;
  }

  std::string str(bool bWithPort) const
  {
    std::string sCharacterString{};

    if(isIPv4())
      {
        char buf[INET_ADDRSTRLEN];

        inet_ntop(AF_INET,
                  &reinterpret_cast<sockaddr_in*>(pSockAddr_.get())->sin_addr,
                  buf,
                  sizeof(buf));

        if(bWithPort)
          {
            sCharacterString = std::string{buf} + ":" + std::to_string(u16Port_);
          }
        else
          {
            sCharacterString = std::string{buf};
          }
      }
    else
      {
        char buf[INET6_ADDRSTRLEN];

        inet_ntop(AF_INET6,
                  &reinterpret_cast<sockaddr_in6 *>(pSockAddr_.get())->sin6_addr,
                  buf,
                  sizeof(buf));

        if(bWithPort)
          {
            sCharacterString =  std::string{"["} + buf + "]:" + std::to_string(u16Port_);
          }
        else
          {
            sCharacterString = std::string{buf};
          }
      }

    return sCharacterString;
  }

private:
  std::unique_ptr<sockaddr> pSockAddr_;
  socklen_t length_;
  int iFamily_;
  std::uint16_t u16Port_;

  void copy(const Implementation & address)
  {
    iFamily_ = address.iFamily_;
    length_ = address.length_;
    u16Port_ = address.u16Port_;

    if(address.iFamily_ == AF_INET)
      {
        pSockAddr_.reset(reinterpret_cast<sockaddr*>(new sockaddr_in));
        memcpy(pSockAddr_.get(),
               reinterpret_cast<sockaddr_in*>(address.pSockAddr_.get()),
               sizeof(sockaddr_in));
      }
    else
      {
        pSockAddr_.reset(reinterpret_cast<sockaddr*>(new sockaddr_in6));
        memcpy(pSockAddr_.get(),
               reinterpret_cast<sockaddr_in6*>(address.pSockAddr_.get()),
               sizeof(sockaddr_in6));
      }
  }
};

EMANE::INETAddr::INETAddr(const sockaddr_in & sockaddr):
  pImpl_{new Implementation{sockaddr}}{}

EMANE::INETAddr::INETAddr(const sockaddr_in6 & sockaddr):
  pImpl_{new Implementation{sockaddr}}{}

EMANE::INETAddr::INETAddr():
  pImpl_{new Implementation{}}{}

EMANE::INETAddr::INETAddr(const std::string & sEndPoint):
  pImpl_{new Implementation{sEndPoint}}{}

EMANE::INETAddr::INETAddr(const std::string & sAddress,
                          std::uint16_t u16Port):
  pImpl_{new Implementation{sAddress,u16Port}}{}

EMANE::INETAddr::INETAddr(const INETAddr & address)
{
  pImpl_.reset(new Implementation(*address.pImpl_.get()));
}

EMANE::INETAddr & EMANE::INETAddr::operator=(const INETAddr & address)
{
  pImpl_.reset(new Implementation(*address.pImpl_.get()));
  return *this;
}

EMANE::INETAddr::INETAddr(INETAddr && address)
{
  pImpl_.swap(address.pImpl_);
}

EMANE::INETAddr & EMANE::INETAddr::operator=(INETAddr && address)
{
  pImpl_.swap(address.pImpl_);
  return *this;
}

EMANE::INETAddr::~INETAddr(){}

void EMANE::INETAddr::set(const std::string & sAddress,
                          std::uint16_t u16Port)
{
  pImpl_->set(sAddress,u16Port);
}

void EMANE::INETAddr::set(const std::string & sEndPoint)
{
  pImpl_->set(sEndPoint);
}

bool EMANE::INETAddr::isIPv6() const
{
  return pImpl_->isIPv6();
}

bool EMANE::INETAddr::isIPv4() const
{
  return pImpl_->isIPv4();
}

bool EMANE::INETAddr::isAny() const
{
  return pImpl_->isAny();
}

int EMANE::INETAddr::getFamily() const
{
  return pImpl_->getFamily();
}

sockaddr * EMANE::INETAddr::getSockAddr() const
{
  return pImpl_->getSockAddr();
}

socklen_t EMANE::INETAddr::getAddrLength() const
{
  return pImpl_->getAddrLength();
}

std::string EMANE::INETAddr::str(bool bWithPort) const
{
  return pImpl_->str(bWithPort);
}

std::uint16_t EMANE::INETAddr::getPort() const
{
  return pImpl_->getPort();
}
