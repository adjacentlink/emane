/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "platformconfiguration.h"
#include "configurationparser.h"

#include <libxml/tree.h>


EMANE::PlatformConfiguration::PlatformConfiguration(const std::string &sFile)
  : 
  LayerConfiguration{"platform"},
  u16Id_{0}
{
  processDefinition("platform", sFile);

  for(auto nem : nems_)
    {
      if(!nem->isValid())
        {
          throw makeException<ValidateException>("NEM id %hu is NOT properly configured!\n"
                                                 "\n"
                                                 "Possible reason(s):\n"
                                                 "\n"
                                                 " * NEM XML is missing one of phy|mac|transport.\n",
                                                 nem->getNEMId());
        }
    }
}


EMANE::PlatformConfiguration::~PlatformConfiguration()
{
  for(auto nem : nems_) { delete nem; }

  nems_.clear();
}


ACE_UINT16 
EMANE::PlatformConfiguration::getPlatformId()
{
  return u16Id_;
}


const EMANE::NEMConfigurations&
EMANE::PlatformConfiguration::getNEMs()
{
  return nems_;
}


void 
EMANE::PlatformConfiguration::doProcessRootAttributes(xmlNodePtr pRoot)
{
  sName_ = getAttrVal(pRoot, "name");

  u16Id_ = getAttrValNumeric(pRoot, "id");
}


void 
EMANE::PlatformConfiguration::doProcessChildNode(xmlNodePtr pNode)
{
  std::string sDefinition{getAttrVal(pNode, "definition")};
  
  std::string sURI{};

  if(!sDefinition.empty())
    {
      sURI = getDefinitionPath() + sDefinition;
    }

  NEMConfiguration *pNEMConfig = new NEMConfiguration{pNode, sURI};

  if (pNEMConfig)
    {
      nems_.push_back(pNEMConfig);
    }
}
