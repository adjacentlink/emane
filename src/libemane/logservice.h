/*
 * Copyright (c) 2013-2016 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2008-2012 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANELOGSERVICE_HEADER_
#define EMANELOGSERVICE_HEADER_

#include "emane/logserviceprovider.h"
#include "emane/componenttypes.h"
#include "emane/inetaddr.h"
#include "emane/utils/vectorio.h"
#include "emane/utils/singleton.h"
#include "datagramsocket.h"

#include <map>
#include <atomic>
#include <thread>
#include <fstream>

namespace EMANE
{
  /**
   * @class LogService
   *
   * @brief Platform log service
   *
   * @details Handles all logging tasks at the platform
   * level
   *
   */
  class LogService : public LogServiceProvider,
                     public Utils::Singleton<LogService>
  {
  public:
    ~LogService();

    void log(LogLevel level,const char *format, ...)
      __attribute__ ((format (printf, 3, 4)));

    void vlog(LogLevel level,const char *format,va_list ap);

    void log(LogLevel level, const Strings & strings);

    void setLogLevel(LogLevel level);

    void redirectLogsToFile(const std::string & file);

    void open();

  protected:
    LogService();

  private:
    std::thread thread_;

    DatagramSocket udpLoggerTxSocket_;

    INETAddr localSocketAddress_;

    std::atomic<LogLevel> level_;

    bool bOpenBackend_;

    std::uint32_t u32LogSequenceNumber_;

    std::ofstream ofs_;

    std::ostream * pStream_;

    int iEventFd_;

    int iepollFd_;

    void processControlMessages();

    bool isLogAllowed(LogLevel level) const;

    void vlog_i(LogLevel level,const char *format,va_list ap);

    void log_i(LogLevel level, const Strings & strings);

    void log_i(LogLevel level,const char *format, ...)
      __attribute__ ((format (printf, 3, 4)));

    void writeLogString(const char * pzLogMessage);
  };

  using LogServiceSingleton = LogService;
}

#endif //EMANELOGSERVICE_HEADER_
