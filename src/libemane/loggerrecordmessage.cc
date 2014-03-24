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

#include "loggerlevelconvert.h"
#include "loggerrecordmessage.h"
#include "loggermessage.pb.h"

class EMANE::Messages::LoggerRecordMessage::Implementation
{
public:
  Implementation(std::string sLogRecord, LogLevel logLevel, uint32_t u32LogSequenceNumber) :
   sLogRecord_{sLogRecord},
   logLevel_{logLevel},
   u32LogSequenceNumber_{u32LogSequenceNumber}{}

  LogLevel getLogLevel() const
  {
    return logLevel_;
  }
 
  std::uint32_t getLogSequenceNumber() const
  {
    return u32LogSequenceNumber_;
  }
 
  const std::string & getLogRecord() const
  {
    return sLogRecord_;
  }

private:
  const std::string   sLogRecord_;
  const LogLevel      logLevel_;
  const std::uint32_t u32LogSequenceNumber_;
};

EMANE::Messages::LoggerRecordMessage::LoggerRecordMessage(const char * p,
                                              size_t len,
                                              LogLevel logLevel, 
                                              std::uint32_t u32LogSequenceNumber):
  pImpl_{new Implementation{std::string{p, len}, logLevel, u32LogSequenceNumber}}
{ }


EMANE::Messages::LoggerRecordMessage::LoggerRecordMessage(const void * p, size_t len) 
{
  EMANEMessage::LoggerMessage loggerMessage;

  try
    {
      if(!loggerMessage.ParseFromArray(p, len))
        {
          throw SerializationException("unable to deserialize LoggerRecordMessage");
        }

      // check for record rype
      if(loggerMessage.type() == EMANEMessage::LoggerMessage_Type_RECORD)
        {
          // check for record
          if(loggerMessage.has_record ())
            {   
               pImpl_.reset(new Implementation(loggerMessage.record().record(),
                                               convertLogLevel(loggerMessage.record().level()), 
                                               loggerMessage.record().sequencenumber()));
            }
          else
            {
              throw SerializationException("LoggerRecordMessage does NOT have record");
            }
        }
      else
        {
          throw SerializationException("LoggerRecordMessage type is not LoggerMessage_Type_RECORD");
        }
    }
  catch(google::protobuf::FatalException & exp)
    {
      throw SerializationException("unable to deserialize LoggerRecordMessage");
    }
}



EMANE::Messages::LoggerRecordMessage::~LoggerRecordMessage()
{ }


EMANE::LogLevel EMANE::Messages::LoggerRecordMessage::getLogLevel() const
{
  return pImpl_->getLogLevel();
}


const std::string & EMANE::Messages::LoggerRecordMessage::getLogRecord() const
{
  return pImpl_->getLogRecord();
}


std::uint32_t EMANE::Messages::LoggerRecordMessage::getLogSequenceNumber() const
{
  return pImpl_->getLogSequenceNumber();
}


EMANE::Serialization EMANE::Messages::LoggerRecordMessage::serialize() const
{
  Serialization serialization;

  try
    {
     // logger message
     EMANEMessage::LoggerMessage loggerMessage;

     // set type log record
     loggerMessage.set_type(EMANEMessage::LoggerMessage_Type_RECORD);

     // get ref to log record msg
     EMANEMessage::LoggerMessage_RecordMessage * pRecordMessage = loggerMessage.mutable_record();

     // set the log record fields
     pRecordMessage->set_level(convertLogLevel(pImpl_->getLogLevel()));

     pRecordMessage->set_record(pImpl_->getLogRecord());

     pRecordMessage->set_sequencenumber(pImpl_->getLogSequenceNumber());

     if(!loggerMessage.SerializeToString(&serialization))
      {
        throw SerializationException("unable to serialize LoggerRecordMessage");
      }
    }
  catch(google::protobuf::FatalException & exp)
    {
      throw SerializationException("unable to serialize LoggerRecordMessage");
    }
  
  return serialization;
}
