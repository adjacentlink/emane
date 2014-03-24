/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2011 - DRS CenGen, LLC, Columbia, Maryland
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

#include "emane/application/logger.h"
#include "logservice.h"

#include <ace/INET_Addr.h>
#include <ace/ace_wchar.h>
#include <cstdarg>

EMANE::Application::Logger::Logger(){}


EMANE::Application::Logger::~Logger(){}


void 
EMANE::Application::Logger::log(LogLevel level, const char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  LogServiceSingleton::instance()->vlog(level, fmt, ap);
  va_end(ap);
}


void
EMANE::Application::Logger::setLogLevel(const LogLevel level)
{
  LogServiceSingleton::instance()->setLogLevel(level);
}


void
EMANE::Application::Logger::redirectLogsToSysLog(const char *program)
{
  const ACE_TCHAR *aceprogram = ACE_TEXT_CHAR_TO_TCHAR(program);
  LogServiceSingleton::instance()->redirectLogsToSysLog(aceprogram);
}


void 
EMANE::Application::Logger::redirectLogsToFile(const char* file)
{
  LogServiceSingleton::instance()->redirectLogsToFile(file);
}


void 
EMANE::Application::Logger::redirectLogsToRemoteLogger(const char *program, 
                                                       const char* addr)
{
  const ACE_TCHAR *aceprogram = ACE_TEXT_CHAR_TO_TCHAR(program);
  LogServiceSingleton::instance()->redirectLogsToRemoteLogger(aceprogram, 
                                                              addr);
}


void 
EMANE::Application::Logger::open()
{
  LogServiceSingleton::instance()->open();
}
