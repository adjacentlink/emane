/*
 * Copyright (c) 2013-2016 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "controlportsession.h"
#include "requestmessagehandler.h"
#include "remotecontrolportapi.pb.h"

#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/uio.h>

EMANE::ControlPort::Session::Session():
  u32MessageSizeBytes_{},
  u32Sequence_{}{}

int EMANE::ControlPort::Session::process(int iFd)
{
  char buf[1024];

  if(u32MessageSizeBytes_ == 0)
    {
      ssize_t length = 0; // number of bytes received

      // read at most 4 bytes from peer to determine message length
      length = recv(iFd,buf,4 - message_.size(),MSG_DONTWAIT);

      // if recv returns 0 or less (error), return -1 which will cause
      // the reactor to stop detecting input events and call handle_close().
      // This service does not specialize handle_close(), so the default
      // implementation (base class's) is used.
      if(length <= 0)
        {
          return -1;
        }

      // save the contents of the message (frame length encoding)
      message_.insert(message_.end(),&buf[0],&buf[length]);

      // is the entire frame length present
      if(message_.size() == 4)
        {
          u32MessageSizeBytes_ = ntohl(*reinterpret_cast<std::uint32_t *>(&message_[0]));

          message_.clear();

          // a message frame of 0 length is not allowed
          if(!u32MessageSizeBytes_)
            {
              return -1;
            }
        }
    }
  else
    {
      // attempt to read message length remaining or max buffer size
      ssize_t length{recv(iFd,
                          buf,
                          u32MessageSizeBytes_ - message_.size() > sizeof(buf) ?
                          sizeof(buf) : u32MessageSizeBytes_ - message_.size(),
                          MSG_DONTWAIT)};

      if(length <= 0)
        {
          return -1;
        }

      message_.insert(message_.end(),&buf[0],&buf[length]);

      // process message when full message is read
      if(message_.size() == u32MessageSizeBytes_)
        {
          EMANERemoteControlPortAPI::Request request{};

          if(!request.ParseFromArray(&message_[0],message_.size()))
            {
              // invalid message - terminate the connection
              return -1;
            }

          std::string sSerialization{RequestMessageHandler::process(request,
                                                                    ++u32Sequence_)};

          std::uint32_t u32MessageFrameLength = htonl(sSerialization.size());

          iovec iov[2] =
            {
              {reinterpret_cast<char *>(&u32MessageFrameLength),sizeof(u32MessageFrameLength)},
              {const_cast<char *>(sSerialization.c_str()),sSerialization.size()}
            };

          writev(iFd,iov,2);

          message_.clear();

          u32MessageSizeBytes_ = 0;
        }
    }

  return 0;
}
