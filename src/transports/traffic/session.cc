/*
 * Copyright (c) 2026 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "session.h"

#include <arpa/inet.h>

EMANE::Transports::Traffic::
Session::Session(NEMId id,PlatformServiceProvider * pPlatformService):
  id_{id},
  pPlatformService_{pPlatformService},
  u32MessageSizeBytes_{}
{}

void
EMANE::Transports::Traffic::Session::clear()
{
  streamBuffer_.clear();
  u32MessageSizeBytes_ = 0;
}

EMANE::Transports::Traffic::Session::StreamMessages
EMANE::Transports::Traffic::
Session::process(const void * buf,size_t bufLength)
{
  StreamMessages streamMessages{};

  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "/Session/%s",
                          __func__);

  // insert data at end of stream buffer
  streamBuffer_.insert(streamBuffer_.end(),
                       reinterpret_cast<const std::uint8_t *>(buf),
                       reinterpret_cast<const std::uint8_t *>(buf) + bufLength);

  while(1)
    {
      // are we currently looking for a length prefix frame
      if(u32MessageSizeBytes_ == 0)
        {
          // do we have enough bytes for the length prefix frame
          if(streamBuffer_.size() < 4)
            {
              // wait for more session data
              break;
            }
          else
            {
              // get the length prefix frame
              u32MessageSizeBytes_ =
                ntohl(*reinterpret_cast<std::uint32_t *>(streamBuffer_.data()));

              // pop the length prefix frame from stream buffer
              streamBuffer_.erase(streamBuffer_.begin(),streamBuffer_.begin() + 4);

              LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                      DEBUG_LEVEL,
                                      "TRANSPORTI %03hu Traffic::Transport::%s"
                                      " Traffic prefix length framing %u",
                                      id_,
                                      __func__,
                                      u32MessageSizeBytes_);
            }
        }
      else
        {
          // do we all the message bytes in the stream buffer
          if(streamBuffer_.size() >= u32MessageSizeBytes_)
            {
              LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                      DEBUG_LEVEL,
                                      "TRANSPORTI %03hu"
                                      " Traffic::Session::%s"
                                      " received message size %u",
                                      id_,
                                      __func__,
                                      u32MessageSizeBytes_);

              // pull the message from the buffer
              streamMessages.emplace_back(StreamMessage(streamBuffer_.begin(),
                                                        streamBuffer_.begin() +
                                                        u32MessageSizeBytes_));

              // pop off the message from stream buffer
              streamBuffer_.erase(streamBuffer_.begin(),
                                  streamBuffer_.begin() + u32MessageSizeBytes_);

              u32MessageSizeBytes_ = 0;
            }
          else
            {
              // wait for more session data
              break;
            }
        }
    }

  return streamMessages;
}
