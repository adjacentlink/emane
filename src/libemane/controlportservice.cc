/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "controlportservice.h"
#include "requestmessagehandler.h"
#include "remotecontrolportapi.pb.h"


EMANE::ControlPort::ControlPortService::ControlPortService():
  u32MessageSizeBytes_{},
  u32Sequence_{}{}

int EMANE::ControlPort::ControlPortService::handle_input(ACE_HANDLE)
{
  char buf[1024];
  
  if(u32MessageSizeBytes_ == 0)
    {
      ssize_t length = 0; // number of bytes received
      
      // read at most 4 bytes from peer to determine message length
      length = peer().recv(buf,4 - message_.size());
        
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
          u32MessageSizeBytes_ = ACE_NTOHL(*reinterpret_cast<std::uint32_t *>(&message_[0]));
            
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
      ssize_t length{peer_.recv(buf,
                                u32MessageSizeBytes_ - message_.size() > sizeof(buf) ? 
                                sizeof(buf) : u32MessageSizeBytes_ - message_.size())};

      if(length <= 0)
        {
          return -1;
        }

      message_.insert(message_.end(),&buf[0],&buf[length]);
        
      // process message when full message is read
      if(message_.size() == u32MessageSizeBytes_)
        {
          EMANERemoteControlPortAPI::Request request{};
          try
            {
              if(!request.ParseFromArray(&message_[0],message_.size()))
                {
                  // invalid message - terminate the connection
                  return -1;
                }
            }
          catch(google::protobuf::FatalException & exp)
            {
              // invalid message - terminate the connection
              return -1;
            }
          
          std::string sSerialization{RequestMessageHandler::process(request,
                                                                    ++u32Sequence_)};
          
          std::uint32_t u32MessageFrameLength = ACE_HTONL(sSerialization.size());

          iovec iov[2] =
            {
              {reinterpret_cast<char *>(&u32MessageFrameLength),sizeof(u32MessageFrameLength)},
              {const_cast<char *>(sSerialization.c_str()),sSerialization.size()}
            };
        
          peer_.sendv_n(iov,2);
          
          message_.clear();

          u32MessageSizeBytes_ = 0;
        }
    }

  return 0;
}

