/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2008 - DRS CenGen, LLC, Columbia, Maryland
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
 * * Neither the name of DRS CenGen, LLC nor the names of its
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


#define _GLIBCXX_USE_NANOSLEEP // gets us sleep_for()

#include "logservice.h"

#include "loggerrecordmessage.h"
#include "loggerlevelconvert.h"

#include "loggermessage.pb.h"

#include "emane/utils/spawnmemberfunc.h"
#include "emane/utils/recvcancelable.h"

#include <cstdarg>
#include <cstdio>
#include <chrono>
#include <thread>

#include <ace/OS_NS_sys_time.h>
#include <ace/OS_NS_string.h>
#include <ace/OS_NS_Thread.h>
#include <ace/OS_NS_time.h>
#include <ace/Log_Msg.h>
#include <ace/streams.h>

namespace
{
  const char * LEVELSTRING[] = 
    { 
      "NONE",  // 0
      "ABORT", // 1
      "ERROR", // 2
      "INFO",  // 3
      "DEBUG", // 4
    };

  const unsigned short MAX_PACKET_LEN = 0xffff;
}

#define TOTAL_LEVELS (static_cast<int>(sizeof(LEVELSTRING) / sizeof(char *)))
#define MAX_LEVEL    (TOTAL_LEVELS - 1)

EMANE::LogService::LogService():
  controlThread_{},
  level_{ABORT_LEVEL},
  bOpenBackend_{false},
  bDecoupleLogging_{false},
  bACELogging_{false},
  u32LogSequenceNumber_{}
{ };

EMANE::LogService::~LogService()
{
  if(bDecoupleLogging_)
    {
      std::this_thread::sleep_for(DoubleSeconds(0.5));     

      ACE_OS::thr_cancel(controlThread_);

      ACE_OS::thr_join(controlThread_,0,0);
    }
};

void EMANE::LogService::log(LogLevel level, const char *fmt, ...)
{
  if(level <= level_) 
    {
      va_list ap;
      
      va_start(ap, fmt);
      
      vlog_i(level,fmt,ap);
      
      va_end(ap);
    }
}


void EMANE::LogService::log(LogLevel level, const Strings & strings)
{
  if(level <= level_) 
    {
      log_i(level, strings);
    }
}


void EMANE::LogService::vlog(LogLevel level, const char *fmt, va_list ap)
{
  if(level <= level_) 
    {
      vlog_i(level, fmt, ap);
    }
}


void EMANE::LogService::redirectLogsToSysLog(const ACE_TCHAR *program)
{
  if(ACE_LOG_MSG->open(program, ACE_Log_Msg::SYSLOG, NULL) == -1)
    {
      ACE_DEBUG((LM_ERROR, ACE_TEXT ("SystemLogging Failed: %s\n"),ACE_OS::strerror(errno)));
    }
  else
    {
      bACELogging_ = true;
    }
}


void EMANE::LogService::redirectLogsToFile(const char* file)
{
  ofstream *log_stream = new ofstream();

  log_stream->open(file, ios::out | ios::app);

  ACE_OSTREAM_TYPE * outputStream = ((ACE_OSTREAM_TYPE *)log_stream);

  ACE_LOG_MSG->msg_ostream(outputStream);

  ACE_LOG_MSG->clr_flags(ACE_Log_Msg::STDERR | ACE_Log_Msg::LOGGER);

  ACE_LOG_MSG->set_flags(ACE_Log_Msg::OSTREAM);

  bACELogging_ = true;
}


void EMANE::LogService::redirectLogsToRemoteLogger(const ACE_TCHAR *program, const char* addr)
{
  if(ACE_LOG_MSG->open(program, ACE_Log_Msg::LOGGER,ACE_TEXT (addr)) == -1)
    {
      ACE_DEBUG((LM_ERROR, ACE_TEXT ("RemoteLogging Failed: %s\n"),ACE_OS::strerror(errno)));
    }
  else
    {
      bACELogging_ = true;
    }  
}


void EMANE::LogService::setLogLevel(LogLevel level)
{
  level_ = level;
}


bool  EMANE::LogService::isLogAllowed(LogLevel level) const
{
  return level <= level_;
}


void EMANE::LogService::open()
{
  if(!bOpenBackend_)
    {
      bOpenBackend_ = true;
      
      if(udpLoggerTxSocket_.open(ACE_INET_Addr(static_cast<std::uint16_t>(0),"127.0.0.1"),
                                 ACE_PROTOCOL_FAMILY_INET,
                                 0,
                                 1) == -1)
        {
          writeLogString("!!! Unable to decomple logging !!!", ERROR_LEVEL); 
        }
      else
        {
          udpLoggerTxSocket_.get_local_addr(localSocketAddress_);
          
          bDecoupleLogging_ = true;

          EMANE::Utils::spawn(*this, &LogService::processControlMessages, &controlThread_);
        }
    }
}


ACE_THR_FUNC_RETURN EMANE::LogService::processControlMessages(void)
{
  char buf[MAX_PACKET_LEN] ={0};

  char errmsg[1024] = {0};

  int iRxLength{};
  
  std::uint16_t * pu16HeaderLength = reinterpret_cast<std::uint16_t *> (buf);

  int iMinExpectedLength = static_cast<int> (sizeof(*pu16HeaderLength));

  while(1)
    {
      // wait for message here, the header len is the fisrt 2 bytes in network byte order
      if((iRxLength = Utils::recvCancelable(udpLoggerTxSocket_,
                                            buf, 
                                            sizeof(buf), 
                                            addrFrom_)) > iMinExpectedLength)
        {
          EMANEMessage::LoggerMessage loggerMessage;
          
          try
            {
              if(!loggerMessage.ParseFromArray(buf + sizeof(*pu16HeaderLength),
                                               ntohs(*pu16HeaderLength)))
                {
                  snprintf(errmsg, sizeof(errmsg), "!!! logger message failed ParseFromArray !!!");
                  
                  writeLogString(errmsg, ERROR_LEVEL);
                }
              else
                {
                  if(loggerMessage.type() == EMANEMessage::LoggerMessage_Type_RECORD)
                    {
                      if(loggerMessage.has_record())
                        {   
                          writeLogString(loggerMessage.record().record().c_str(),
                                         convertLogLevel(loggerMessage.record().level()));
                        }
                      else
                        {
                          snprintf(errmsg, sizeof(errmsg), "!!! logger message type RECORD does NOT have record !!!");
                          
                          writeLogString(errmsg, ERROR_LEVEL);
                        }
                    }
                  else
                    {
                      snprintf(errmsg, sizeof(errmsg), "!!! unhandled logger message type %d !!!", loggerMessage.type());
                      
                      writeLogString(errmsg, ERROR_LEVEL);
                    }
                }
            }
          catch(google::protobuf::FatalException & exp)
            {
              snprintf(errmsg, sizeof(errmsg), "!!! caught exception (%s) !!!", exp.what());
              
              writeLogString(errmsg, ERROR_LEVEL);
            }
        }
      else if(iRxLength < 0)
        {
          // socket read error
          snprintf(errmsg, sizeof(errmsg), "!!! socket read error (%s) !!!", strerror(errno));
              
          writeLogString(errmsg, ERROR_LEVEL);
              
          break;
        }
      else if(iRxLength == 0)
        {
          // socket empty
          snprintf(errmsg, sizeof(errmsg), "socket empty, bye");
              
          writeLogString(errmsg, DEBUG_LEVEL);
              
          break;
        }
      else 
        {
          // socket read length short
          snprintf(errmsg, sizeof(errmsg), 
                   "!!! socket read len of %d, msg length must be greater than %d, ignore !!!",
                   iRxLength, iMinExpectedLength);
              
          writeLogString(errmsg, DEBUG_LEVEL);
        }
    }
  
  return 0;
}






void EMANE::LogService::log_i(LogLevel level, const char *fmt, ...)
{
  va_list ap;
      
  va_start(ap, fmt);
      
  vlog_i(level,fmt,ap);
      
  va_end(ap);
}



void EMANE::LogService::log_i(LogLevel level, const Strings & strings)
{
  std::string sMessage{};

  std::for_each(strings.begin(), 
                strings.end(),
                [&sMessage](const std::string & sPart){sMessage += sPart; sMessage.push_back(' ');});
      
  log_i(level,"%s",sMessage.c_str());
}


void EMANE::LogService::vlog_i(LogLevel level, const char *fmt, va_list ap)
{
  ACE_Time_Value tv = ACE_OS::gettimeofday();
      
  char buff[1024] = {0};

  struct tm ltm;
      
  time_t t = tv.sec();
      
  ACE_OS::localtime_r(&t, &ltm);
      
  std::uint16_t len = snprintf(buff, sizeof(buff),"%02d:%02d:%02d.%06lu %5s ",
                               ltm.tm_hour,
                               ltm.tm_min,
                               ltm.tm_sec,
                               tv.usec(),
                               level >= 0 && level < TOTAL_LEVELS ? LEVELSTRING[level] : "?");
      
  len += vsnprintf(buff + len, sizeof(buff) - len, fmt, ap);

  if(bDecoupleLogging_ == false)
    {
      writeLogString(buff, level);   
    }
  else
    {
      Utils::VectorIO outputVector(2);

      // set buff, len, level and seq num in msg hdr, len does not include terminationg byte
      EMANE::Messages::LoggerRecordMessage msg {buff, len, level, u32LogSequenceNumber_++};

      Serialization serialization{msg.serialize()};

      // the header len in net byte order
      std::uint16_t u16HeaderLen = htons(serialization.size());

      outputVector[0] = Utils::make_iovec((void *) &u16HeaderLen, sizeof(u16HeaderLen));
      outputVector[1] = Utils::make_iovec((void *) serialization.c_str(), serialization.size());

      udpLoggerTxSocket_.send(static_cast<const iovec*>(&outputVector[0]), 
                              static_cast<int>(outputVector.size()), 
                              localSocketAddress_);
    }
}


void EMANE::LogService::writeLogString(const char * pzLogMessage, LogLevel level)
{
  if(bACELogging_)
    {
      // Map the EMANE Log Level to the appropriate ACE Log Level
      switch(level)
        {
        case ABORT_LEVEL:   
          ACE_DEBUG((LM_CRITICAL, ACE_TEXT("%s\n"),pzLogMessage));
          break;
        case ERROR_LEVEL:
          ACE_DEBUG((LM_ERROR, ACE_TEXT("%s\n"),pzLogMessage));
          break;
        case INFO_LEVEL:
          ACE_DEBUG((LM_INFO, ACE_TEXT("%s\n"),pzLogMessage));
          break;
        case DEBUG_LEVEL:
          ACE_DEBUG((LM_DEBUG, ACE_TEXT("%s\n"),pzLogMessage));
          break;
        default:
          ACE_DEBUG((LM_ERROR, ACE_TEXT("Log Message wasn't a valid log level\n")));
          break;
        }
    }
  else
    {
      printf("%s\n",pzLogMessage);
    }
}
