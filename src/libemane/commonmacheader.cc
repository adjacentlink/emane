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

#include "emane/commonmacheader.h"
#include "commonmacheader.pb.h"

class EMANE::CommonMACHeader::Implementation
{
public:
  Implementation(RegistrationId registrationId,
                 std::uint64_t u64SequenceNumber):
    registrationId_{registrationId},
    u64SequenceNumber_{u64SequenceNumber}
    {}
    
  RegistrationId getRegistrationId() const
  {
    return registrationId_;
  }
  
  std::uint64_t getSequenceNumber() const
  {
    return u64SequenceNumber_;
  }
  
private:
  RegistrationId registrationId_;
  std::uint64_t u64SequenceNumber_;
};

EMANE::CommonMACHeader::CommonMACHeader(UpstreamPacket & pkt)
{
  if(pkt.length() >= sizeof(decltype(pkt.stripLengthPrefixFraming())))
    {
      std::uint16_t u16HeaderLength{pkt.stripLengthPrefixFraming()};
      
      if(pkt.length() >= u16HeaderLength)
        {
          EMANEMessage::CommonMACHeader msg;
          
          try
            {
              if(!msg.ParseFromArray(pkt.get(),u16HeaderLength))
                {
                  throw SerializationException("unable to deserialize CommonMACHeader");
                }
            }
          catch(google::protobuf::FatalException & exp)
            {
              throw SerializationException("unable to deserialize CommonMACHeader");
            }
          
          pImpl_.reset(new Implementation{static_cast<RegistrationId>(msg.registrationid()),
                msg.sequencenumber()});

        }
      else
        {
           throw SerializationException("CommonMACHeader not large enough for header data");
        }
      
      // strip common mac header from pkt
      pkt.strip(u16HeaderLength);
    }
  else
    {
      throw SerializationException("CommonMACHeader not large enough for header size data");
    }
}


EMANE::CommonMACHeader::CommonMACHeader(RegistrationId registrationId, 
                                        std::uint64_t u64SequenceNumber):
pImpl_{new Implementation{registrationId, 
                          u64SequenceNumber}}
{}


EMANE::CommonMACHeader::CommonMACHeader(CommonMACHeader&& rvalue):
  pImpl_(std::move(rvalue.pImpl_))
{}

EMANE::CommonMACHeader::~CommonMACHeader(){}

EMANE::RegistrationId EMANE::CommonMACHeader::getRegistrationId() const
{
  return pImpl_->getRegistrationId();
}
    
std::uint64_t EMANE::CommonMACHeader::getSequenceNumber() const
{
  return pImpl_->getSequenceNumber();
}

    
void EMANE::CommonMACHeader::prependTo(DownstreamPacket & pkt) const
{
  EMANEMessage::CommonMACHeader msg{};

  msg.set_registrationid(pImpl_->getRegistrationId());
  msg.set_sequencenumber(pImpl_->getSequenceNumber());

  std::string sSerialization;
  
  try
    {
      if(!msg.SerializeToString(&sSerialization))
        {
          throw SerializationException("unable to serialize CommonMACHeader");
        }
    }
  catch(google::protobuf::FatalException & exp)
    {
      throw SerializationException("unable to serialize CommonMACHeader");
    }
  
  // prepend order is important
  pkt.prepend(sSerialization.c_str(),sSerialization.size());
  
  pkt.prependLengthPrefixFraming(sSerialization.size());
}

EMANE::Strings EMANE::CommonMACHeader::format() const
{
  Strings sFormat{"regid",
      std::to_string(pImpl_->getRegistrationId()),
      "seq",
      std::to_string(pImpl_->getSequenceNumber())};
        
  return sFormat;
}
