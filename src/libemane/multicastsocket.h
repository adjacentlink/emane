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

#ifndef EMANEMULTICASTSOCKET_HEADER_
#define EMANEMULTICASTSOCKET_HEADER_

#include "socket.h"
#include "emane/inetaddr.h"

#include <string>
#include <cstdint>
#include <sys/uio.h>

namespace EMANE
{
  class MulticastSocket: public Socket
  {
  public:
    MulticastSocket();

    MulticastSocket(const INETAddr & address,
                    bool bReuseAddress = false,
                    const std::string & sDevice = "",
                    std::uint8_t u8TTL = 1,
                    bool bLoop = false);

    ~MulticastSocket();

    void open(const INETAddr & address,
              bool bReuseAddress = false,
              const std::string & sDevice = "",
              std::uint8_t u8TTL = 1,
              bool bLoop = false);

    ssize_t send(const iovec *iov, int iovcnt, int flags=0) const;

    ssize_t recv(void * buf,
                 size_t len,
                 int flags=0);


  private:
    INETAddr addr_;
  };
}

#endif // EMANEMULTICASTSOCKET_HEADER_
