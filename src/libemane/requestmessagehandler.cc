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

#include "requestmessagehandler.h"
#include "manifestqueryhandler.h"
#include "configurationqueryhandler.h"
#include "statisticqueryhandler.h"
#include "statistictablequeryhandler.h"
#include "configurationupdatehandler.h"
#include "statisticclearupdatehandler.h"
#include "statistictableclearupdatehandler.h"
#include "loglevelupdatehandler.h"
#include "errorresponse.h"
#include "remotecontrolportapi.pb.h"

std::string
EMANE::ControlPort::RequestMessageHandler::process(const EMANERemoteControlPortAPI::Request & request,
                                                   std::uint32_t u32Sequence)

{
  if(request.type() ==  EMANERemoteControlPortAPI::Request::TYPE_REQUEST_QUERY)
    {
      if(request.has_query())
        {
          const auto & query = request.query();

          if(query.type() == EMANERemoteControlPortAPI::TYPE_QUERY_MANIFEST)
            {
              return ManifestQueryHandler::process(u32Sequence,request.sequence());
            }
          else if(query.type() == EMANERemoteControlPortAPI::TYPE_QUERY_CONFIGURATION)
            {
              if(query.has_configuration())
                {
                  return ConfigurationQueryHandler::process(query.configuration(),u32Sequence,request.sequence());
                }
              else
                {
                  return ErrorResponse::serialize(EMANERemoteControlPortAPI::Response::Error::TYPE_ERROR_MALFORMED,
                                                  "Configuration query request malformed: missing configuration message",
                                                  u32Sequence,
                                                  request.sequence());
                }
            }
          else if(query.type() == EMANERemoteControlPortAPI::TYPE_QUERY_STATISTIC)
            {
              if(query.has_statistic())
                {
                  return StatisticQueryHandler::process(query.statistic(),u32Sequence,request.sequence());

                }
              else
                {
                  return ErrorResponse::serialize(EMANERemoteControlPortAPI::Response::Error::TYPE_ERROR_MALFORMED,
                                                  "Statistic query request malformed: missing statistic message",
                                                  u32Sequence,
                                                  request.sequence());
                }
            }
          else if(query.type() == EMANERemoteControlPortAPI::TYPE_QUERY_STATISTICTABLE)
            {
              if(query.has_statistictable())
                {
                  return StatisticTableQueryHandler::process(query.statistictable(),u32Sequence,request.sequence());
                }
              else
                {
                  return ErrorResponse::serialize(EMANERemoteControlPortAPI::Response::Error::TYPE_ERROR_MALFORMED,
                                                  "Statistic Table query request malformed: missing statisticTable message",
                                                  u32Sequence,
                                                  request.sequence());
                  
                }
            }
        }
      else
        {
           ErrorResponse::serialize(EMANERemoteControlPortAPI::Response::Error::TYPE_ERROR_MALFORMED,
                                    "Query request malformed: missing query message",
                                    u32Sequence,
                                    request.sequence());
           
        }
    }
  else if(request.type() == EMANERemoteControlPortAPI::Request::TYPE_REQUEST_UPDATE)
    {
      if(request.has_update())
        {
          const auto & update = request.update();

          if(update.type() == EMANERemoteControlPortAPI::TYPE_UPDATE_CONFIGURATION)
            {
              if(update.has_configuration())
                {
                  return ConfigurationUpdateHandler::process(update.configuration(),u32Sequence,request.sequence());
                }
              else
                {
                  return ErrorResponse::serialize(EMANERemoteControlPortAPI::Response::Error::TYPE_ERROR_MALFORMED,
                                                  "Configuration update request malformed: missing configuration message",
                                                  u32Sequence,
                                                  request.sequence());
                }
            }
          else if(update.type() == EMANERemoteControlPortAPI::TYPE_UPDATE_STATISTICCLEAR)
            {
              if(update.has_statisticclear())
                {
                  return StatisticClearUpdateHandler::process(update.statisticclear(),u32Sequence,request.sequence());

                }
              else
                {
                  return ErrorResponse::serialize(EMANERemoteControlPortAPI::Response::Error::TYPE_ERROR_MALFORMED,
                                                  "Statistic Table update request malformed: missing statisticClear message",
                                                  u32Sequence,
                                                  request.sequence());
                }
            }
          else if(update.type() == EMANERemoteControlPortAPI::TYPE_UPDATE_STATISTICTABLECLEAR)
            {
              if(update.has_statistictableclear())
                {
                  return StatisticTableClearUpdateHandler::process(update.statistictableclear(),u32Sequence,request.sequence());

                }
              else
                {
                  return ErrorResponse::serialize(EMANERemoteControlPortAPI::Response::Error::TYPE_ERROR_MALFORMED,
                                                  "Statistic Table Clear update request malformed: missing statisticTableClear message",
                                                  u32Sequence,
                                                  request.sequence());
                }
            }
          else if(update.type() == EMANERemoteControlPortAPI::TYPE_UPDATE_LOGLEVEL)
            {
              if(update.has_loglevel())
                {
                  return LogLevelUpdateHandler::process(update.loglevel(),u32Sequence,request.sequence());

                }
              else
                {
                  return ErrorResponse::serialize(EMANERemoteControlPortAPI::Response::Error::TYPE_ERROR_MALFORMED,
                                                  "Log Level update request malformed: missing LogLevel message",
                                                  u32Sequence,
                                                  request.sequence());
                }
            }
        }
      else
        {
          return ErrorResponse::serialize(EMANERemoteControlPortAPI::Response::Error::TYPE_ERROR_MALFORMED,
                                          "Update request malformed: missing update message",
                                          u32Sequence,
                                          request.sequence());
        }
    }
  
  return ErrorResponse::serialize(EMANERemoteControlPortAPI::Response::Error::TYPE_ERROR_MALFORMED,
                                  "Request malformed: unknown request type",
                                  u32Sequence,
                                  request.sequence());
}
