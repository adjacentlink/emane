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

inline EMANEMessage::LoggerMessage_LogLevel EMANE::convertLogLevel(EMANE::LogLevel level)
  {
     switch(level)
      {
      case EMANE::ABORT_LEVEL:
        return EMANEMessage::LoggerMessage_LogLevel_ABORT;
        
      case EMANE::ERROR_LEVEL:
        return EMANEMessage::LoggerMessage_LogLevel_ERROR;
        
      case EMANE::INFO_LEVEL:
        return EMANEMessage::LoggerMessage_LogLevel_INFO;
        
      default:
        return EMANEMessage::LoggerMessage_LogLevel_DEBUG;
      }
   }


inline EMANE::LogLevel EMANE::convertLogLevel(EMANEMessage::LoggerMessage_LogLevel level)
   {
     switch(level)
      {
        case EMANEMessage::LoggerMessage_LogLevel_ABORT:
          return EMANE::ABORT_LEVEL;

        case EMANEMessage::LoggerMessage_LogLevel_ERROR:
          return EMANE::ERROR_LEVEL;

      case EMANEMessage::LoggerMessage_LogLevel_INFO:
          return EMANE::INFO_LEVEL;

        default:
          return EMANE::DEBUG_LEVEL;
      }
   }
