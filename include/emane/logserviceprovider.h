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

#ifndef EMANELOGSERVICEPROVIDER_HEADER_
#define EMANELOGSERVICEPROVIDER_HEADER_

#include "emane/types.h"

#include <cstdarg>

namespace EMANE
{
  enum LogLevel
    {
      NOLOG_LEVEL = 0, /**< No Logging                                        */
      ABORT_LEVEL = 1, /**< Abort message - unrecoverable application failure */
      ERROR_LEVEL = 2, /**< Error message - recoverable application error     */
      INFO_LEVEL =  3, /**< Info message  - informational messages            */
      DEBUG_LEVEL = 4, /**< Debug message - general application debugging     */
    };

  class PlatformService;
  
  /**
   * @class LogServiceProvider
   *
   * @brief Log service provider interface
   */
  class LogServiceProvider
  {
  public:
    virtual ~LogServiceProvider(){}
    
    /**
     * Output a log message
     *
     * @param level Log level of the message
     * @param fmt format string (see printf)
     * @param ... Variable data (see printf)
     */
    virtual void log(LogLevel level, const char *fmt, ...)
      __attribute__ ((format (printf, 3, 4))) = 0;


    /**
     * Output a log message
     *
     * @param level Log level of the message
     * @param fmt format string (see printf)
     * @param ap Variable argument list data (see vprintf)
     */
    virtual void vlog(LogLevel level, const char *fmt, va_list ap) = 0;

    
    /**
     * Output a log message
     *
     * @param level Log level of the message
     * @param strings Strings to output
     */
    virtual void log(LogLevel level, const Strings & strings) = 0;


    /**
     * Output a log message using a callable
     *
     * @param level Log level of the message
     * @param fn Callable returning Strings
     */
    template <typename Function>
    void logfn(LogLevel level, Function fn);

    /**
     * Output a log message using a callable
     *
     * @param level Log level of the message
     * @param fn Callable returning Strings
     * @param fmt format string (see printf)
     * @param ap Variable argument list data (see vprintf)
     */
    template <typename Function>
    void logfn(LogLevel level, Function fn,const char *fmt, va_list ap);

    /**
     * Output a log message using a callable
     *
     * @param level Log level of the message
     * @param fn Callable returning Strings
     * @param fmt format string (see printf)
     */
    template <typename Function>
    void logfn(LogLevel level, Function fn,const char *fmt, ...)
      __attribute__ ((format (printf, 4, 5)));


  protected:
    LogServiceProvider(){}

    virtual bool isLogAllowed(LogLevel level) const = 0;

    virtual void log_i(LogLevel level, const Strings & strings) = 0;

  };
}

#ifdef VERBOSE_LOGGING

#define LOGGER_VERBOSE_LOGGING(logger,level,fmt,args...)  \
  (logger).log(level,fmt,## args)

#define LOGGER_VERBOSE_LOGGING_FN(logger,level,fn)  \
  (logger).logfn(level,fn)

#define LOGGER_VERBOSE_LOGGING_FN_VARGS(logger,level,fn,fmt,args...) \
  (logger).logfn(level,fn,fmt,## args)


#else

/**
 * Logs a printf style log message. Message my be compiled out
 * using ./configure --without-verbose-logging.
 *
 * @param logger @ref EMANE::LogServiceProvider "LogServiceProvider" reference
 * @param level Log level of the message
 * @param fmt format string (see printf)
 * @param args... Variable data (see printf)
 */
#define LOGGER_VERBOSE_LOGGING(logger,level,fmt,args...)

/**
 * Logs log message output from a callable. Message my be compiled out
 * using ./configure --without-verbose-logging.
 *
 * @param logger @ref EMANE::LogServiceProvider "LogServiceProvider" reference
 * @param level Log level of the message
 * @param fn Callable object returning Strings
 * 
 * @note Callable object is executed based on log level.
 */
#define LOGGER_VERBOSE_LOGGING_FN(logger,level,fn)

/**
 * Logs a printf style log message with appended message output from a
 * callable. Message my be compiled out using ./configure
 * --without-verbose-logging.
 *
 * @param logger @ref EMANE::LogServiceProvider "LogServiceProvider" reference
 * @param level Log level of the message
 * @param fn Callable object returning Strings
 * @param fmt format string (see printf)
 * @param args... Variable data (see printf)
 * 
 * @note Callable object is executed based on log level.
 */
#define LOGGER_VERBOSE_LOGGING_FN_VARGS(logger,level,fn,fmt,args...)

#endif

/**
 * Logs a printf style log message.
 *
 * @param logger @ref EMANE::LogServiceProvider "LogServiceProvider" reference
 * @param level Log level of the message
 * @param fmt format string (see printf)
 * @param args... Variable data (see printf)
 */
#define LOGGER_STANDARD_LOGGING(logger,level,fmt,args...) \
  (logger).log(level,fmt,## args)

/**
 * Logs log message output from a callable.
 *
 * @param logger @ref EMANE::LogServiceProvider "LogServiceProvider" reference
 * @param level Log level of the message
 * @param fn Callable object returning Strings
 * 
 * @note Callable object is executed based on log level.
 */
#define LOGGER_STANDARD_LOGGING_FN(logger,level,fn) \
  (logger).logfn(level,fn)

/**
 * Logs a printf style log message with appended message output from a
 * callable. Message my be compiled out using ./configure
 * --without-verbose-logging.
 *
 * @param logger @ref EMANE::LogServiceProvider "LogServiceProvider" reference
 * @param level Log level of the message
 * @param fn Callable object returning Strings
 * @param fmt format string (see printf)
 * @param args... Variable data (see printf)
 * 
 * @note Callable object is executed based on log level.
 */
#define LOGGER_STANDARD_LOGGING_FN_VARGS(logger,level,fn,fmt,args...) \
  (logger).logfn(level,fn,fmt,## args)

#include "emane/logserviceprovider.inl"

#endif //EMANELOGSERVICEPROVIDER_HEADER_

/**
 * @page LogService Log Service
 *
 * The @ref EMANE::LogServiceProvider "LogServiceProvider" is used by components to log output messages.
 *
 * @section IssuingALogMessage Issuing a Log Message
 *
 * The @ref EMANE::LogServiceProvider "LogServiceProvider" is accessed via the
 * @ref EMANE::PlatformServiceProvider "PlatformServiceProvider". All components are given a reference
 * to the @ref EMANE::PlatformServiceProvider "PlatformServiceProvider" when they are constructed.
 *
 * There are two types of logging macros: @a standard and @a verbose. The only difference between the standard
 * and verbose macro forms are that verbose macros can be compiled out using the
 * @a --without-verbose-logging configure option.
 *
 * - #LOGGER_STANDARD_LOGGING - Logs a printf style log message.
 * - #LOGGER_STANDARD_LOGGING_FN - Logs log message output from a callable.
 * - #LOGGER_STANDARD_LOGGING_FN_VARGS - Logs a printf style log message with appended message output from a
 * callable.
 *
 * - #LOGGER_VERBOSE_LOGGING - Logs a printf style log message.
 * - #LOGGER_VERBOSE_LOGGING_FN - Logs log message output from a callable.
 * - #LOGGER_VERBOSE_LOGGING_FN_VARGS - Logs a printf style log message with appended message output from a
 * callable.
 *
 * @snippet src/models/mac/rfpipe/maclayer.cc logservice-loggerfnvargs-snippet
 *
 * The Logger is designed to be lightweight especially
 * when the current log level does not permit a log message to be emitted. However, it may be useful to
 * analyze model performance without any @a go-path log statements.
 *
 * It is a best practice to use the standard form of log macros for all life cycle related logging and error
 * reporting. Use the verbose form of log macros for per-packet (go-path) debug logging.
 *
 * Use  #LOGGER_STANDARD_LOGGING_FN, #LOGGER_STANDARD_LOGGING_FN_VARGS, #LOGGER_VERBOSE_LOGGING_FN and 
 * #LOGGER_VERBOSE_LOGGING_FN_VARGS whenever you want to create a log message string using a function or method
 * and print the resulting string as a "%s". These macros are smart enough to not execute callables unless the
 * current log level is set to allow the message to be emitted.
 *
 * @section LogLevels Log Levels
 *
 * Log messages are assigned one of four log levels:
 * - @ref EMANE::ABORT_LEVEL "ABORT_LEVEL" - Used to emit an abort message prior to throwing an exception that
 * will terminate the emulator.
 * - @ref EMANE::ERROR_LEVEL "ERROR_LEVEL" - Used to emit an error message. Errors are considered worthy of
 *  reporting but not significant enough to terminate the emulator.
 * - @ref EMANE::INFO_LEVEL "INFO_LEVEL" - Used to emit informational messages concerning configuration item values.
 * - @ref EMANE::DEBUG_LEVEL "DEBUG_LEVEL" - Used to emit the kitchen sink.
 *
 * It is a best practice to only log @ref EMANE::INFO_LEVEL "INFO_LEVEL" messages from
 * @ref EMANE::Component::configure "Component::configure" and @ref EMANE::RunningStateMutable::processConfiguration
 * "RunningStateMutable::processConfiguration". @ref EMANE::INFO_LEVEL "INFO_LEVEL" log messages should contain
 * configuration item name and value information.
 *
 * @snippet src/models/mac/rfpipe/maclayer.cc logservice-infolog-snippet
 *
 * Logging, more specifically over logging, can have a significant impact on emulation model performance. It is never
 * advisable to run the emulator with a log level greater than @ref EMANE::INFO_LEVEL "INFO_LEVEL" (command line level
 * 3) unless you are debugging.
 *
 @verbatim
 $ emane platform.xml -l 3
 10:35:22.591338  INFO application: emane -l 3 platform.xml 
 10:35:22.591526  INFO application uuid: fcdf297b-38b7-49b2-903e-b3412f20b051
 10:35:22.591544 ERROR Please consider using the realtime scheduler to improve fidelity.
 10:35:22.607130  INFO MACI 001 RFPipeMACLayer::configure datarate = 10000
 10:35:22.607206  INFO MACI 001 RFPipeMACLayer::configure delay = 1.500000
 10:35:22.607245  INFO MACI 001 RFPipeMACLayer::configure enablepromiscuousmode = off
 10:35:22.607275  INFO MACI 001 RFPipeMACLayer::configure flowcontrolenable = off
 10:35:22.607301  INFO MACI 001 RFPipeMACLayer::configure flowcontroltokens = 10
 10:35:22.607333  INFO MACI 001 RFPipeMACLayer::configure jitter = 1.000000
 10:35:22.607350  INFO MACI 001 RFPipeMACLayer::configure pcrcurveuri = rfpipepcr.xml
 10:35:22.607363  INFO MACI 001 RFPipeMACLayer::configure neighbormetricdeletetime = 60.000000
 10:35:22.607383  INFO MACI 001 RFPipeMACLayer::configure radiometricenable = off
 10:35:22.607398  INFO MACI 001 RFPipeMACLayer::configure radiometricreportinterval = 1.000000
 10:35:22.607716  INFO PHYI 001 FrameworkPHY::configure: bandwidth = 1000000 Hz
 10:35:22.607760  INFO PHYI 001 FrameworkPHY::configure: fixedantennagain = 0.00 dBi
 10:35:22.607780  INFO PHYI 001 FrameworkPHY::configure: fixedantennagainenable = on
 10:35:22.607794  INFO PHYI 001 FrameworkPHY::configure: noisebinsize = 20 usec
 10:35:22.607816  INFO PHYI 001 FrameworkPHY::configure: noisemaxmessagepropagation = 200000 usec
 10:35:22.607829  INFO PHYI 001 FrameworkPHY::configure: noisemaxsegmentduration = 1000000 usec
 10:35:22.607841  INFO PHYI 001 FrameworkPHY::configure: noisemaxsegmentoffset = 300000 usec
 10:35:22.607856  INFO PHYI 001 FrameworkPHY::configure: noisemode = all
 10:35:22.607871  INFO PHYI 001 FrameworkPHY::configure: propagationmodel = precomputed
 10:35:22.607884  INFO PHYI 001 FrameworkPHY::configure: subid = 2
 10:35:22.607906  INFO PHYI 001 FrameworkPHY::configure: systemnoisefigure = 4.00 dB
 10:35:22.607918  INFO PHYI 001 FrameworkPHY::configure: txpower = 0.00 dBm
 10:35:22.607927  INFO PHYI 001 FrameworkPHY::configure: frequency = 2347000000 Hz
 10:35:22.607939  INFO PHYI 001 FrameworkPHY::configure: frequencyofinterest = 2347000000 Hz
 10:35:22.607948  INFO PHYI 001 FrameworkPHY::configure: noisemaxclampenable = off
 10:35:22.607956  INFO PHYI 001 FrameworkPHY::configure: timesyncthreshold = 10000 usec
 10:35:22.737387  INFO NEM  001 NEMImpl::configure platformendpoint: 127.0.0.1/8181
 10:35:22.737452  INFO NEM  001 NEMImpl::configure transportendpoint: 127.0.0.1/8171
 @endverbatim
 */
