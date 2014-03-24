/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2009 - DRS CenGen, LLC, Columbia, Maryland
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

#include "eventdaemonconfiguration.h"
#include "eventagentconfiguration.h"

#include <utility>


EMANE::EventDaemonConfiguration::EventDaemonConfiguration(const std::string &sFile)
  : LayerConfiguration{"eventdaemon"}
{ 
  processDefinition("eventdaemon", sFile);
}
  

EMANE::EventDaemonConfiguration::~EventDaemonConfiguration()
{
  for(auto agent : agents_) { delete agent; }

  agents_.clear();
}


EMANE::NEMId 
EMANE::EventDaemonConfiguration::getNEMId()
{
  return nemId_;
}


const EMANE::LayerConfigurations &
EMANE::EventDaemonConfiguration::getAgents()
{
  return agents_;
}


void 
EMANE::EventDaemonConfiguration::doProcessRootAttributes(xmlNodePtr pRoot)
{
  sName_ = getAttrVal(pRoot, "name");

  nemId_ = getAttrValNumeric(pRoot, "nemid");
}


void 
EMANE::EventDaemonConfiguration::doProcessChildNode(xmlNodePtr pNode)
{
  std::string sURI{getDefinitionPath() + getAttrVal(pNode, "definition")};

  LayerConfiguration *pAgentConfig = new EventAgentConfiguration(pNode, sURI);

  if (pAgentConfig)
    {
      agents_.push_back(pAgentConfig);
    }

}
