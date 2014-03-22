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

#include "emane/controls/flowcontrolcontrolmessage.h"
#include "flowcontrol.pb.h"

class EMANE::Controls::FlowControlControlMessage::Implementation
{
public:
  Implementation(std::uint16_t u16Tokens):
    u16Tokens_{u16Tokens}{}

  std::uint16_t getTokens() const
  {
    return u16Tokens_;
  }
  
private:
  const std::uint16_t u16Tokens_;
};

EMANE::Controls::FlowControlControlMessage::
FlowControlControlMessage(const FlowControlControlMessage & msg):
  ControlMessage{IDENTIFIER},
  pImpl_{new Implementation{*msg.pImpl_}}
{}

EMANE::Controls::FlowControlControlMessage::FlowControlControlMessage(std::uint16_t u16Tokens):
  ControlMessage{IDENTIFIER},
  pImpl_{new Implementation{u16Tokens}}
{}

EMANE::Controls::FlowControlControlMessage::~FlowControlControlMessage()
{}

std::uint16_t EMANE::Controls::FlowControlControlMessage::getTokens() const
{
  return pImpl_->getTokens();
}


EMANE::Serialization EMANE::Controls::FlowControlControlMessage::serialize() const
{
  Serialization serialization;

  EMANEMessage::FlowControlControlMessage msg;
  msg.set_tokens(pImpl_->getTokens());

  try
    {
      if(!msg.SerializeToString(&serialization))
        {
          throw SerializationException("unable to serialize FlowControlControlMessage");
        }
    }
  catch(google::protobuf::FatalException & exp)
    {
      throw SerializationException("unable to serialize FlowControlControlMessage");
    }
  
  return serialization;
}

EMANE::Controls::FlowControlControlMessage *
EMANE::Controls::FlowControlControlMessage::create(std::uint16_t u16Tokens)
{
  return new FlowControlControlMessage{u16Tokens};
}

EMANE::Controls::FlowControlControlMessage *
EMANE::Controls::FlowControlControlMessage::create(const Serialization & serialization)
{
  EMANEMessage::FlowControlControlMessage msg;

  try
    {
      if(!msg.ParseFromString(serialization))
        {
          throw SerializationException("unable to deserialize : FlowControlControlMessage");
        }
    }
  catch(google::protobuf::FatalException & exp)
    {
      throw SerializationException("unable to deserialize : FlowControlControlMessage");
    }
  
  return new FlowControlControlMessage{static_cast<std::uint16_t>(msg.tokens())};
}

EMANE::Controls::FlowControlControlMessage *
EMANE::Controls::FlowControlControlMessage::clone() const
{
  return new FlowControlControlMessage{*this};
}
