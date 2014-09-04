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

#include "manifestqueryhandler.h"
#include "buildidservice.h"
#include "emane/serializationexception.h"

std::string
EMANE::ControlPort::ManifestQueryHandler::process(std::uint32_t u32Sequence,
                                                  std::uint32_t u32Reference)
{
  auto componentMap = BuildIdServiceSingleton::instance()->getNEMLayerComponentBuildIdMap();
  
  EMANERemoteControlPortAPI::Response response;
  
  response.set_type(EMANERemoteControlPortAPI::Response::TYPE_RESPONSE_QUERY);

  auto pQuery = response.mutable_query();

  pQuery->set_type(EMANERemoteControlPortAPI::TYPE_QUERY_MANIFEST);

  auto pManifest = pQuery->mutable_manifest();

  using Component = EMANERemoteControlPortAPI::Response::Query::Manifest::NEM::Component;


  for(const auto & nem : componentMap)
    {
      auto pNEM = pManifest->add_nems();

      pNEM->set_id(nem.first);

      for(const auto & component : nem.second)
        {
          auto pComponent = pNEM->add_components();
          
          pComponent->set_buildid(std::get<0>(component));

          ComponentType componentType{std::get<1>(component)};
          
          pComponent->set_type(componentType == ComponentType::COMPONENT_PHYILAYER ? 
                               Component::TYPE_COMPONENT_PHY : 
                               componentType == ComponentType::COMPONENT_MACILAYER ?
                               Component::TYPE_COMPONENT_MAC : 
                               componentType == ComponentType::COMPONENT_SHIMILAYER ?
                               Component::TYPE_COMPONENT_SHIM:
                               Component::TYPE_COMPONENT_TRANSPORT);

          pComponent->set_plugin(std::get<2>(component));
        }
    }
  
  response.set_reference(u32Reference);
  
  response.set_sequence(u32Sequence);

  std::string sSerialization;
  
  try
    {
      if(!response.SerializeToString(&sSerialization))
        {
          throw SerializationException("unable to serialize manifest query response");
        }
    }
  catch(google::protobuf::FatalException & exp)
    {
      throw SerializationException("unable to serialize manifest query response");
    }

  return sSerialization;
}
