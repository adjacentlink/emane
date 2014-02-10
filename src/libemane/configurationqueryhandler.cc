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

#include "configurationqueryhandler.h"
#include "configurationservice.h"
#include "emane/serializationexception.h"
#include "emane/registrarexception.h"
#include "anyutils.h"

std::string
EMANE::ControlPort::ConfigurationQueryHandler::process(const EMANERemoteControlPortAPI::Request::Query::Configuration & configuration,
                                                       std::uint32_t u32Sequence,
                                                       std::uint32_t u32Reference)
{
  std::vector<std::string> names;
  
  for(int i = 0; i < configuration.names_size(); ++i)
    {
      names.push_back(configuration.names(i));
    }

  EMANERemoteControlPortAPI::Response response;

  try
    {
      auto configurationValues = 
        ConfigurationServiceSingleton::instance()->queryConfiguration(configuration.buildid(),
                                                                      names);
      
      response.set_type(EMANERemoteControlPortAPI::Response::TYPE_RESPONSE_QUERY);
      
      auto pQuery = response.mutable_query();
      
      pQuery->set_type(EMANERemoteControlPortAPI::TYPE_QUERY_CONFIGURATION);
      
      auto pConfiguration = pQuery->mutable_configuration();
      
      pConfiguration->set_buildid(configuration.buildid());
      
      for(const auto & value : configurationValues)
        {
          auto pParameter = pConfiguration->add_parameters();
          
          pParameter->set_name(value.first);
          
          for(const auto & any : value.second)
            {
              auto pValue = pParameter->add_values();
              
              convertToAny(pValue,any);
            }
        }
    }
  catch(RegistrarException & exp)
    {
      response.set_type(EMANERemoteControlPortAPI::Response::TYPE_RESPONSE_ERROR);
      
      auto pError = response.mutable_error();
      
      pError->set_type(EMANERemoteControlPortAPI::Response::Error::TYPE_ERROR_PARAMETER);
      
      pError->set_description(exp.what());
    }
  
  response.set_reference(u32Reference);
  
  response.set_sequence(u32Sequence);
  
  std::string sSerialization;

  try
    {
      if(!response.SerializeToString(&sSerialization))
        {
          throw SerializationException("unable to serialize configuration query response");
        }
    }
  catch(google::protobuf::FatalException & exp)
    {
      throw SerializationException("unable to serialize configuration query response");
    }

  return sSerialization;
}
