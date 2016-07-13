/*
 * Copyright (c) 2013-2016 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include <cstdarg>
#include <cstdio>
#include <chrono>
#include <thread>
#include <iostream>
#include <algorithm>
#include <sys/eventfd.h>
#include <sys/epoll.h>
#include <unistd.h>

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
  thread_{},
  level_{ABORT_LEVEL},
  bOpenBackend_{false},
  u32LogSequenceNumber_{},
  pStream_{&std::cout},
  iEventFd_{},
  iepollFd_{}
{
  bOpenBackend_ = true;

  udpLoggerTxSocket_.open(INETAddr{"127.0.0.1",0});

  localSocketAddress_ = udpLoggerTxSocket_.getLocalAddress();

  iEventFd_ = eventfd(0,0);

  iepollFd_ = epoll_create1(0);

  // add the eventfd socket to the epoll instance
  struct epoll_event ev;
  ev.events = EPOLLIN;
  ev.data.fd = iEventFd_;
  epoll_ctl(iepollFd_,EPOLL_CTL_ADD,iEventFd_,&ev);

  // add the backend logger socket to the epoll instance
  ev.events = EPOLLIN;
  ev.data.fd =  udpLoggerTxSocket_.getHandle();
  epoll_ctl(iepollFd_,EPOLL_CTL_ADD,udpLoggerTxSocket_.getHandle(),&ev);

  thread_ = std::thread{&LogService::processControlMessages, this};
};

EMANE::LogService::~LogService()
{
  std::this_thread::sleep_for(DoubleSeconds(0.5));

  // signal thread procedure shutdown
  const uint64_t one{1};
  write(iEventFd_,&one,sizeof(one));

  thread_.join();

  close(iepollFd_);
  close(iEventFd_);
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


void EMANE::LogService::redirectLogsToFile(const std::string & file)
{
  ofs_.open(file,std::ofstream::trunc);
  pStream_  = &ofs_;
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

      udpLoggerTxSocket_.open(INETAddr{"127.0.0.1",0});

      localSocketAddress_ = udpLoggerTxSocket_.getLocalAddress();

      thread_ = std::thread{&LogService::processControlMessages, this};
    }
}


void EMANE::LogService::processControlMessages(void)
{
  char buf[MAX_PACKET_LEN] ={0};

  char errmsg[1024] = {0};

  int iRxLength{};

  std::uint16_t * pu16HeaderLength = reinterpret_cast<std::uint16_t *> (buf);

  int iMinExpectedLength = static_cast<int> (sizeof(*pu16HeaderLength));

  struct epoll_event events[2];

  int nfds{};

  bool bDone{};

  while(!bDone)
    {
      nfds = epoll_wait(iepollFd_,events,2,-1);

      if(nfds == -1)
        {
          break;
        }

      for(int n = 0; n < nfds; ++n)
        {
          if(events[n].data.fd == udpLoggerTxSocket_.getHandle())
            {
              // wait for message here, the header len is the fisrt 2 bytes in network byte order
              if((iRxLength = udpLoggerTxSocket_.recv(buf,sizeof(buf),0)) > iMinExpectedLength)
                {
                  EMANEMessage::LoggerMessage loggerMessage;

                  if(!loggerMessage.ParseFromArray(buf + sizeof(*pu16HeaderLength),
                                                   ntohs(*pu16HeaderLength)))
                    {
                      snprintf(errmsg,
                               sizeof(errmsg),
                               "!!! logger message failed ParseFromArray !!!");

                      writeLogString(errmsg);
                    }
                  else
                    {
                      if(loggerMessage.type() == EMANEMessage::LoggerMessage_Type_RECORD)
                        {
                          if(loggerMessage.has_record())
                            {
                              writeLogString(loggerMessage.record().record().c_str());
                            }
                          else
                            {
                              snprintf(errmsg,
                                       sizeof(errmsg),
                                       "!!! logger message type RECORD does NOT have record !!!");

                              writeLogString(errmsg);
                            }
                        }
                      else
                        {
                          snprintf(errmsg,
                                   sizeof(errmsg),
                                   "!!! unhandled logger message type %d !!!",
                                   loggerMessage.type());

                          writeLogString(errmsg);
                        }
                    }
                }
              else if(iRxLength < 0)
                {
                  // socket read error
                  snprintf(errmsg,
                           sizeof(errmsg),
                           "!!! socket read error (%s) !!!",
                           strerror(errno));

                  writeLogString(errmsg);

                  bDone = true;

                  break;
                }
              else if(iRxLength == 0)
                {
                  // socket empty
                  snprintf(errmsg, sizeof(errmsg), "socket empty, bye");

                  writeLogString(errmsg);

                  bDone = true;

                  break;
                }
              else
                {
                  // socket read length short
                  snprintf(errmsg, sizeof(errmsg),
                           "!!! socket read len of %d, msg length must be greater than %d, ignore !!!",
                           iRxLength, iMinExpectedLength);

                  writeLogString(errmsg);
                }
            }
          else
            {
              bDone = true;
              break;
            }
        }
    }
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
  auto now = Clock::now();

  std::time_t t{Clock::to_time_t(now)};

  std::tm ltm;

  localtime_r(&t, &ltm);

  // buffer size is MAX_LOG_LENGTH + timestamp + level
  const int iBuffSize{MAX_LOG_LENGTH + 22};
  char buff[iBuffSize] = {0};

  int iLen = snprintf(buff, sizeof(buff),"%02d:%02d:%02d.%06lu %5s ",
                      ltm.tm_hour,
                      ltm.tm_min,
                      ltm.tm_sec,
                      std::chrono::duration_cast<Microseconds>(now.time_since_epoch()).count()%1000000,
                      level >= 0 && level < TOTAL_LEVELS ? LEVELSTRING[level] : "?");

  if(iLen < 0 || iLen > iBuffSize)
    {
      // silently discard
      return;
    }

  int iAppended = vsnprintf(buff + iLen, iBuffSize - iLen, fmt, ap);

  if(iAppended > 0)
    {
      iLen += iAppended;

      if(iLen > iBuffSize)
        {
          iLen = iBuffSize;
        }
    }

  Utils::VectorIO outputVector(2);

  // set buff, len, level and seq num in msg hdr, len does not include terminationg byte
  EMANE::Messages::LoggerRecordMessage msg {buff,
      static_cast<size_t>(iLen),
      level,
      u32LogSequenceNumber_++};

  Serialization serialization{msg.serialize()};

  // the header len in net byte order
  std::uint16_t u16HeaderLen = htons(serialization.size());

  outputVector[0] = Utils::make_iovec((void *) &u16HeaderLen, sizeof(u16HeaderLen));
  outputVector[1] = Utils::make_iovec((void *) serialization.c_str(), serialization.size());

  udpLoggerTxSocket_.send(static_cast<const iovec*>(&outputVector[0]),
                          static_cast<int>(outputVector.size()),
                          localSocketAddress_);
}


void EMANE::LogService::writeLogString(const char * pzLogMessage)
{
  (*pStream_)<<pzLogMessage<<std::endl;
}
