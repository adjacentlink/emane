/*
 * Copyright (c) 2013-2014,2016 - Adjacent Link LLC, Bridgewater, New
 * Jersey
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

#include "controlmessageserializer.h"
#include "controlmessageserializerexception.h"
#include "emane/controls/serializedcontrolmessage.h"
#include <arpa/inet.h>

EMANE::ControlMessageSerializer::ControlMessageSerializer(const ControlMessages & msgs):
  serializations_(msgs.size(),std::string()),
  vectorIO_(msgs.size() * 2 + 1,iovec()),
  controlMessageHeaders_(msgs.size(),ControlMessageHeader()),
  length_(0)
{
  ControlMessages::const_iterator iter = msgs.begin();

  controlMessageSerializerHeader_.u16Count_ = htons(msgs.size());
  vectorIO_[0].iov_len = sizeof(ControlMessageSerializerHeader);
  vectorIO_[0].iov_base = &controlMessageSerializerHeader_;
  length_ += vectorIO_[0].iov_len;

  for(int i = 0; iter != msgs.end(); ++iter,++i)
    {
      (*iter)->serialize().swap(serializations_[i]);
      controlMessageHeaders_[i].u16Id_ = htons((*iter)->getId());
      controlMessageHeaders_[i].u16Length_ = htons(serializations_[i].length());

      vectorIO_[i*2+1].iov_len = sizeof(ControlMessageHeader);
      vectorIO_[i*2+1].iov_base = &controlMessageHeaders_[i];
      length_ += vectorIO_[i*2+1].iov_len;

      vectorIO_[i*2 + 2].iov_len = serializations_[i].length();
      vectorIO_[i*2 + 2].iov_base = const_cast<char *>(serializations_[i].c_str());
      length_ += vectorIO_[i*2+2].iov_len;
    }
}

EMANE::ControlMessageSerializer::~ControlMessageSerializer(){}

const EMANE::Utils::VectorIO &
EMANE::ControlMessageSerializer::getVectorIO() const
{
  return vectorIO_;
}

size_t EMANE::ControlMessageSerializer::getLength() const
{
  return length_;
}

EMANE::ControlMessages EMANE::ControlMessageSerializer::create(const void * pData, size_t length)
{
  ControlMessages controlMessages;

  if(length >= sizeof(ControlMessageSerializerHeader))
    {
      const ControlMessageSerializerHeader * pControlMessageSerializerHeader =
        reinterpret_cast<const ControlMessageSerializerHeader *>(pData);

      std::uint16_t u16ControlMessageCount =
        std::uint16_t(pControlMessageSerializerHeader->u16Count_);

      length -= sizeof(ControlMessageSerializerHeader);

      const ControlMessageHeader * pControlMessageHeader =
        reinterpret_cast<const ControlMessageHeader *>(pControlMessageSerializerHeader->data_);

      for(int i = 0; i < u16ControlMessageCount; ++i)
        {
          if(length >= sizeof(ControlMessageHeader))
            {
              length -= sizeof(ControlMessageHeader);

              std::uint16_t u16ControlMessageId = ntohs(pControlMessageHeader->u16Id_);
              std::uint16_t u16ControlMessageLength = ntohs(pControlMessageHeader->u16Length_);

              if(length >= u16ControlMessageLength)
                {
                  controlMessages.push_back(Controls::SerializedControlMessage::create(u16ControlMessageId,
                                                                                       pControlMessageHeader->data_,
                                                                                       u16ControlMessageLength));

                  length -= u16ControlMessageLength;
                }
              else
                {
                  throw ControlMessageSerializerException("Control Message header length mismatch");
                }


              pControlMessageHeader =
                reinterpret_cast<const ControlMessageHeader *>(pControlMessageHeader->data_ +
                                                               u16ControlMessageLength);
            }
          else
            {
              throw ControlMessageSerializerException("Control Message count mismatch");
            }


        }
    }
  else
    {
      throw ControlMessageSerializerException("Control message serializer header length mismatch");
    }

  return controlMessages;
}

EMANE::ControlMessages
EMANE::ControlMessageSerializer::create(const EMANE::Utils::VectorIO & vectorIO)
{
  ControlMessages controlMessages;

  if(!vectorIO.empty())
    {
      if(vectorIO[0].iov_len == sizeof(ControlMessageSerializerHeader))
        {
          const ControlMessageSerializerHeader * pControlMessageSerializerHeader =
            reinterpret_cast<const ControlMessageSerializerHeader *>(vectorIO[0].iov_base);

          std::uint16_t u16ControlMessageCount =
            ntohs(pControlMessageSerializerHeader->u16Count_);


          if(u16ControlMessageCount <= (vectorIO.size() - 1) / 2)
            {
              for(int i = 0; i < u16ControlMessageCount; ++i)
                {
                  if(vectorIO[i*2 +1].iov_len == sizeof(ControlMessageHeader))
                    {
                      const ControlMessageHeader * pControlMessageHeader =
                        reinterpret_cast<const ControlMessageHeader *>(vectorIO[i*2 +1].iov_base);

                      std::uint16_t u16ControlMessageId = ntohs(pControlMessageHeader->u16Id_);
                      std::uint16_t u16ControlMessageLength = ntohs(pControlMessageHeader->u16Length_);


                      if(vectorIO[i*2 + 2].iov_len == u16ControlMessageLength)
                        {
                          controlMessages.push_back(Controls::SerializedControlMessage::create(u16ControlMessageId,
                                                                                               vectorIO[i*2 + 2].iov_base,
                                                                                               u16ControlMessageLength));
                        }
                      else
                        {
                          throw ControlMessageSerializerException("Control Message data length mismatch");
                        }
                    }
                  else
                    {
                      throw ControlMessageSerializerException("Control Message header length mismatch");
                    }
                }
            }
          else
            {
              throw ControlMessageSerializerException("Control Message count mismatch");
            }
        }
      else
        {
          throw ControlMessageSerializerException("Control message serializer header length mismatch");
        }
    }

  return controlMessages;
}
