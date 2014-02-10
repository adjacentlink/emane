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

#include "emane/controls/serializedcontrolmessage.h"

class EMANE::Controls::SerializedControlMessage::Implementation
{
public:
  Implementation(ControlMessageId id,
                 const void * pData,
                 size_t length):
    serializedId_(id),
    sSerialization_(reinterpret_cast<const char *>(pData),length){}
  
  ~Implementation(){}

  ControlMessageId getSerializedId() const
  {
    return serializedId_;
  }

  std::string getSerialization() const
  {
    return sSerialization_;
  }

private:
  const ControlMessageId serializedId_;
  const std::string sSerialization_;
};

EMANE::Controls::SerializedControlMessage::SerializedControlMessage(ControlMessageId id,
                                                          const void * pData,
                                                          size_t length):
  ControlMessage(IDENTIFIER),
  pImpl_(new Implementation(id,pData,length))
{}

EMANE::Controls::SerializedControlMessage::~SerializedControlMessage()
{}

EMANE::ControlMessageId EMANE::Controls::SerializedControlMessage::getSerializedId() const
{
  return pImpl_->getSerializedId();
}

  
std::string EMANE::Controls::SerializedControlMessage::getSerialization() const
{
  return pImpl_->getSerialization();
}


EMANE::Controls::SerializedControlMessage * 
EMANE::Controls::SerializedControlMessage::create(ControlMessageId id,
                                                  const void * pData,
                                                  size_t length)
{
  return new SerializedControlMessage(id,pData,length);
}

