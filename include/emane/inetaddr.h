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

#ifndef EMANEINETADDR_HEADER_
#define EMANEINETADDR_HEADER_

#include <cstdint>
#include <string>
#include <memory>
#include <net/if.h>
#include <netinet/in.h>

namespace EMANE
{
  class INETAddr
  {
  public:
    INETAddr();

    INETAddr(const std::string & sEndPoint);

    INETAddr(const std::string & sAddr,
             std::uint16_t u16Port);

    INETAddr(const sockaddr_in & sockaddr);

    INETAddr(const sockaddr_in6 & sockaddr);

    /**
     * Creates an INETAddr instance by copying another instance
     */
    INETAddr(const INETAddr & address);

    /**
     * Creates an INETAddr instance by moving another instance
     */
    INETAddr(INETAddr && address);


    ~INETAddr();

    /**
     * Sets an INETAddr instance to a copy of another instance
     */
    INETAddr & operator=(const INETAddr & address);


    /**
     * Sets and INETAddr instance by moving another instance
     */
    INETAddr & operator=(INETAddr && address);

    void set(const std::string & sAddress, std::uint16_t u16Port);

    void set(const std::string & sEndPoint);

    bool isIPv6() const;

    bool isIPv4() const;

    bool isAny() const;

    bool isMulticast() const;

    int getFamily() const;

    sockaddr * getSockAddr() const;

    socklen_t getAddrLength() const;

    std::string str(bool bWithPort = true) const;

    std::uint16_t getPort() const;

  private:
    class Implementation;
    std::unique_ptr<Implementation>  pImpl_;
  };
}

#endif // EMANEINETADDR_HEADER_
