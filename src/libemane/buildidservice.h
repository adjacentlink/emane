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

#ifndef EMANEBUILDIDSERVICE_HEADER_
#define EMANEBUILDIDSERVICE_HEADER_

#include "emane/types.h"
#include "emane/buildable.h"
#include "emane/nemlayer.h"
#include "emane/application/nem.h"
#include "emane/application/nemmanager.h"
#include "emane/application/transportmanager.h"
#include "emane/application/eventagentmanager.h"
#include "emane/application/eventgeneratormanager.h"
#include "emane/componenttypes.h"
#include "emane/utils/singleton.h"

#include <map>
#include <vector>
#include <tuple>

namespace EMANE
{
  using NEMLayerComponentBuildIdMap = std::map<NEMId,std::vector<std::tuple<BuildId, ComponentType,std::string>>>;

  class BuildIdService : public Utils::Singleton<BuildIdService>
  {
  public:
    BuildId registerBuildable(Application::NEMManager * pNEMManager);

    BuildId registerBuildable(Application::NEM * pNEM);

    BuildId registerBuildable(NEMLayer * pLayer,ComponentType type, const std::string & sPluginName);
    
    BuildId registerBuildable(Application::TransportManager * pTransportManager);

    BuildId registerBuildable(Application::TransportAdapter * pTransportAdapter);

    BuildId registerBuildable(Transport * pTransport);
    
    BuildId registerBuildable(Application::EventGeneratorManager * pEventGeneratorManager);

    BuildId registerBuildable(EventGenerator * pGenerator);
    
    BuildId registerBuildable(Application::EventAgentManager * pEventAgentManager);

    BuildId registerBuildable(EventAgent * pEventGenerator);
    
    const NEMLayerComponentBuildIdMap & getNEMLayerComponentBuildIdMap() const;

  protected:
    BuildIdService();

  private:
    using NEMBuildIdMap = std::map<NEMId,BuildId>;
    using NEMTransportBuildIdMap = std::map<NEMId,BuildId>;
    using NEMTransportAdapterBuildIdMap = std::map<NEMId, BuildId>;
    using EventGeneratorBuildIds = std::vector<BuildId>;
    using EventAgentBuildIds = std::vector<BuildId>;

    BuildId buildId_;
    BuildId nemManagerBuildId_;
    BuildId transportManagerBuildId_;
    BuildId eventGeneratorManagerBuildId_;
    BuildId eventAgentManagerBuildId_;
    
    NEMLayerComponentBuildIdMap NEMLayerComponentBuildIdMap_;
    NEMTransportAdapterBuildIdMap NEMTransportAdapterBuildIdMap_;
    NEMTransportBuildIdMap NEMTransportBuildIdMap_;
    NEMBuildIdMap NEMBuildIdMap_;
    EventGeneratorBuildIds eventGeneratorBuildIds_;
    EventAgentBuildIds eventAgentBuildIds_;

    BuildId assignBuildId(Buildable *pBuildable);
  };

  using BuildIdServiceSingleton = BuildIdService; 
}



#endif // EMANEBUILDIDSERVICE_HEADER_
