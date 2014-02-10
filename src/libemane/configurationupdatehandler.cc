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

#include "configurationupdatehandler.h"
#include "configurationservice.h"
#include "emane/serializationexception.h"
#include "anyutils.h"

std::string
EMANE::ControlPort::ConfigurationUpdateHandler::process(const EMANERemoteControlPortAPI::Request::Update::Configuration & configuration,
                                                       std::uint32_t u32Sequence,
                                                       std::uint32_t u32Reference)
{
  bool bUpdate{true};

  ConfigurationUpdate updates;
                      
  using RepeatedPtrFieldParameters = 
    google::protobuf::RepeatedPtrField<EMANERemoteControlPortAPI::Request::Update::Configuration::Parameter>;

  EMANERemoteControlPortAPI::Response response;
  
  for(const auto & parameter :
        RepeatedPtrFieldParameters(configuration.parameters()))
    {
      std::vector<Any> anys;
      
      using RepeatedPtrFieldAnys = 
        google::protobuf::RepeatedPtrField<EMANERemoteControlPortAPI::Any>;
      
      for(const auto & any :
            RepeatedPtrFieldAnys(parameter.values()))
        {
          try
            {
              anys.push_back(toAny(any));
            }
          catch(AnyException & exp)
            {
              response.set_type(EMANERemoteControlPortAPI::Response::TYPE_RESPONSE_ERROR);
              
              auto pError = response.mutable_error();
              
              pError->set_type(EMANERemoteControlPortAPI::Response::Error::TYPE_ERROR_PARAMETER);
              
              pError->set_description(exp.what());

              bUpdate = false;

              break;
            }
        }
      
      updates.push_back(std::make_pair(parameter.name(),std::move(anys)));
    }
  
  if(bUpdate)
    {
      try
        {
          ConfigurationServiceSingleton::instance()->update(configuration.buildid(),updates);
          
          response.set_type(EMANERemoteControlPortAPI::Response::TYPE_RESPONSE_UPDATE);      
        }
      catch(ConfigurationException & exp)
        {
          response.set_type(EMANERemoteControlPortAPI::Response::TYPE_RESPONSE_ERROR);
          
          auto pError = response.mutable_error();
          
          pError->set_type(EMANERemoteControlPortAPI::Response::Error::TYPE_ERROR_PARAMETER);
          
          pError->set_description(exp.what());
        }
    }

  response.set_reference(u32Reference);
  
  response.set_sequence(u32Sequence);
  
  std::string sSerialization;
  
  try
    {
      if(!response.SerializeToString(&sSerialization))
        {
          throw SerializationException("unable to serialize configuration update response");
        }
    }
  catch(google::protobuf::FatalException & exp)
    {
      throw SerializationException("unable to serialize configuration update response");
    }

  return sSerialization;
}
