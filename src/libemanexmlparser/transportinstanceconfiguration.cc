/*
 * Copyright (c) 2013,2016 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2009-2010 - DRS CenGen, LLC, Columbia, Maryland
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

#include "transportinstanceconfiguration.h"
#include "pluginconfiguration.h"

EMANE::TransportInstanceConfiguration::TransportInstanceConfiguration(
                                           xmlNodePtr pTransportInstanceNode,
                                           const std::string sURI)
  : LayerConfiguration{"instance"},
    u16Id_{getAttrValNumeric(pTransportInstanceNode, "nemid")}
{
  setPathAndFile(sURI);

  doProcessChildren(pTransportInstanceNode, params_);

  // There is no overlay here. The instance does not define the same
  // parameters as the transport. Or, if it does, it is by mistake.
}


EMANE::TransportInstanceConfiguration::~TransportInstanceConfiguration()
{
  for(auto layer : layers_) { delete layer; }

  layers_.clear();
}


EMANE::NEMId
EMANE::TransportInstanceConfiguration::getId()
{
  return u16Id_;
}


const EMANE::LayerConfigurations &
EMANE::TransportInstanceConfiguration::getLayers()
{
  return layers_;
}


void
EMANE::TransportInstanceConfiguration::doProcessChildNode(xmlNodePtr pNode)
{
  std::string sLayerName = reinterpret_cast<const char*>(pNode->name);

  if (sLayerName == "transport")
    {
      std::string sURI{getDefinitionPath() + getAttrVal(pNode, "definition")};

      layers_.push_back(new PluginConfiguration(pNode,
                                                sURI,
                                                "transport"));
    }
  else
    {
      throw makeException<ParseException>("Failed to parse '%s' layer.\n\n"
                                          "Possible reason(s):\n"
                                          " * '%s' layer is not allowed inside an 'instance'.\n",
                                          sLayerName.c_str(),
                                          sLayerName.c_str());
    }
}
