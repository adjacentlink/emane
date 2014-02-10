/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "rfpipemacheadermessage.h"
#include "rfpipemacheader.pb.h"


class EMANE::Models::RFPipe::MACHeaderMessage::Implementation
{
public:
  Implementation(std::uint64_t u64DataRate) :
   u64DataRate_{u64DataRate}
    { }

   std::uint64_t getDataRate() const
    {
       return u64DataRate_;
    }

private:
  const std::uint64_t u64DataRate_; 
};


EMANE::Models::RFPipe::MACHeaderMessage::MACHeaderMessage(std::uint64_t u64DataRate) :
  pImpl_{new Implementation{u64DataRate}}
{ }


EMANE::Models::RFPipe::MACHeaderMessage::MACHeaderMessage(const void * p, size_t len) 
{
  EMANEMessage::RFPipeMACHeader message;

  try
    {
      if(!message.ParseFromArray(p, len))
        {
          throw SerializationException("unable to deserialize MACHeaderMessage");
        }

       pImpl_.reset(new Implementation{static_cast<std::uint64_t>(message.datarate())});
    }
  catch(google::protobuf::FatalException & exp)
    {
      throw SerializationException("unable to deserialize MACHeaderMessage");
    }
}



EMANE::Models::RFPipe::MACHeaderMessage::~MACHeaderMessage()
{ }


std::uint64_t EMANE::Models::RFPipe::MACHeaderMessage::getDataRate() const
{
  return pImpl_->getDataRate();
}


EMANE::Serialization EMANE::Models::RFPipe::MACHeaderMessage::serialize() const
{
  Serialization serialization;

  try
    {
      EMANEMessage::RFPipeMACHeader message;

      message.set_datarate(pImpl_->getDataRate());

      if(!message.SerializeToString(&serialization))
        {
          throw SerializationException("unable to serialize MACHeaderMessage");
        }
    }
  catch(google::protobuf::FatalException & exp)
    {
      throw SerializationException("unable to serialize MACHeaderMessage");
    }
  
  return serialization;
}
