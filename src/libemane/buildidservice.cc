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

#include "buildidservice.h"
#include "emane/buildexception.h"

EMANE::BuildIdService::BuildIdService():
  buildId_{},
  nemManagerBuildId_{},
  transportManagerBuildId_{},
  eventGeneratorManagerBuildId_{},
  eventAgentManagerBuildId_{}{}


EMANE::BuildId
EMANE::BuildIdService::assignBuildId(Buildable *pBuildable)
{
  ++buildId_;
  pBuildable->setBuildId(buildId_);
  return buildId_;
}
    
EMANE::BuildId EMANE::BuildIdService::registerBuildable(Application::NEMManager * pNEMManager)
{
  if(!nemManagerBuildId_)
    {
      nemManagerBuildId_ = assignBuildId(pNEMManager);
    }
  else
    {
      throw BuildException("NEM Manager already registered");
    }
  
  return nemManagerBuildId_;
}

EMANE::BuildId EMANE::BuildIdService::registerBuildable(NEMLayer * pLayer, ComponentType type,const std::string & sPluginName)
{
  auto iter = NEMLayerComponentBuildIdMap_.find(pLayer->getNEMId());
  
  BuildId buildId{assignBuildId(pLayer)};
    
  if(iter != NEMLayerComponentBuildIdMap_.end())
    {
      iter->second.push_back(std::make_tuple(buildId,type,sPluginName));
    }
  else
    {
      NEMLayerComponentBuildIdMap_.insert(std::make_pair(pLayer->getNEMId(),
                                                         std::vector<std::tuple<BuildId,ComponentType,std::string>>{std::make_tuple(buildId,type,sPluginName)}));
    }

  return buildId;
}

EMANE::BuildId EMANE::BuildIdService::registerBuildable(Application::TransportManager * pTransportManager)
{
  if(!transportManagerBuildId_)
    {
      transportManagerBuildId_ = assignBuildId(pTransportManager);
    }
  else
    {
      throw BuildException("Transport Manager already registered");
    }

  return transportManagerBuildId_;
}

EMANE::BuildId EMANE::BuildIdService::registerBuildable(Transport * pTransport)
{
  BuildId buildId{assignBuildId(pTransport)};

  NEMTransportBuildIdMap_.insert(std::make_pair(pTransport->getNEMId(),buildId));

  return buildId;
}

EMANE::BuildId EMANE::BuildIdService::registerBuildable(Application::NEM * pNEM)
{
  BuildId buildId{assignBuildId(pNEM)};

  NEMBuildIdMap_.insert(std::make_pair(pNEM->getNEMId(),buildId));

  return buildId;
}

EMANE::BuildId EMANE::BuildIdService::registerBuildable(Application::TransportAdapter * pTransportAdapter)
{
  BuildId buildId{assignBuildId(pTransportAdapter)};
  
  NEMTransportAdapterBuildIdMap_.insert(std::make_pair(pTransportAdapter->getNEMId(),
                                                       buildId));
  return buildId;
}

EMANE::BuildId EMANE::BuildIdService::registerBuildable(Application::EventGeneratorManager * pEventGeneratorManager)
{
  if(!eventGeneratorManagerBuildId_)
    {
      eventGeneratorManagerBuildId_ = assignBuildId(pEventGeneratorManager);
    }
  else
    {
      throw BuildException("Event Generator Manager already registered");
    }
  
  return eventGeneratorManagerBuildId_; 
}

EMANE::BuildId EMANE::BuildIdService::registerBuildable(EventGenerator * pGenerator)
{
  BuildId buildId{assignBuildId(pGenerator)};

  eventGeneratorBuildIds_.push_back(buildId);

  return buildId;
}

EMANE::BuildId EMANE::BuildIdService::registerBuildable(Application::EventAgentManager * pEventAgentManager)
{
  if(!eventAgentManagerBuildId_)
    {
      eventAgentManagerBuildId_ = assignBuildId(pEventAgentManager);
    }
  else
    {
      throw BuildException("Event Agent Manager already registered");
    }
  
  return eventAgentManagerBuildId_;
}

EMANE::BuildId EMANE::BuildIdService::registerBuildable(EventAgent * pEventAgent)
{
  BuildId buildId{assignBuildId(pEventAgent)};
  
  eventAgentBuildIds_.push_back(buildId);

  return buildId;
}

const EMANE::NEMLayerComponentBuildIdMap & EMANE::BuildIdService::getNEMLayerComponentBuildIdMap() const
{
  return NEMLayerComponentBuildIdMap_;
}

// EMANE::BuildId  EMANE::BuildIdService::getNEMManagerBuildId()
// {
//   return pNEMManager_->getBuildId();
// }

// std::vector<std::pair<EMANE::BuildId,EMANE::NEMId>>  EMANE::BuildIdService::getNEMBuildIds() const
// {

// }

// EMANE::BuildId
//   EMANE::BuildIdService::getNEMLayerBuildId(NEMId nemId, ComponentType type) const
// {
// }

// std::vector<std::pair<EMANE::BuildId,EMANE::ComponentType>>
//   EMANE::BuildIdService::getNEMLayerBuildIds(NEMId nemId)
// {
// }

// EMANE::BuildId EMANE::BuildIdService::getTransportManagerBuildId() const
// {
//   return pTransportManager_->getBuildId();
// }

// EMANE::BuildId EMANE::BuildIdService::getTransportBuildId(NEMId nemId) const
// {
//   const auto iter = NEMTransportMap_.find(nemId);
  
//   if(iter != NEMTransportMap_.end())
//     {
//       return iter->second->getBuildId();
//     }
//   else
//     {
//       return 0;
//     }
// }

// std::vector<EMANE::BuildId>  EMANE::BuildIdService::getTransportBuildIds() const
// {
//   std::vector<EMANE::BuildId> ids;
//   std::transform(NEMTransportMap_.begin(),
//                  NEMTransportMap_.end(),
//                  back_inserter(ids),
//                  std::bind(&Buildable::getBuildId,
//                            std::bind(&NEMTransportMap::value_type::second,
//                                      std::placeholders::_1)));
// }

// EMANE::BuildId EMANE::BuildIdService::getEventGeneratorManagerBuildId() const
// {
//   return pEventGeneratorManager_->getBuildId();
// }

// std::vector<EMANE::BuildId>  EMANE::BuildIdService::getEventGeneratorBuildIds() const
// {
//   std::vector<EMANE::BuildId> ids;
//   std::transform(eventGenerators_.begin(),
//                  eventGenerators_.end(),
//                  back_inserter(ids),
//                  std::bind(&Buildable::getBuildId,
//                            std::placeholders::_1));
// }

// EMANE::BuildId EMANE::BuildIdService::getEventAgentManagerBuildId() const
// {
//   return pEventAgentManager_->getBuildId();
// }

// std::vector<EMANE::BuildId> EMANE::BuildIdService::getEventAgentBuildIds() const
// {
//   std::vector<EMANE::BuildId> ids;
//   std::transform(eventAgents_.begin(),
//                  eventAgents_.end(),
//                  back_inserter(ids),
//                  std::bind(&Buildable::getBuildId,
//                            std::placeholders::_1));
// }
