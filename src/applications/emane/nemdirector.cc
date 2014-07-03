/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "configurationparser.h"
#include "nemdirector.h"
#include <sstream>

EMANE::Application::NEMDirector::NEMDirector(const std::string &filename,
                                             NEMBuilder &builder):
  configPlatform_(filename),
  rNEMBuilder_(builder)
{}
  
/*
 * Destructor
 */
EMANE::Application::NEMDirector::~NEMDirector()
{}

std::unique_ptr<EMANE::Application::NEMManager>
EMANE::Application::NEMDirector::construct(const uuid_t & uuid)
{
  NEMs nems;
  
  for(const auto & nemConfiguration : configPlatform_.getNEMs())
    {
      nems.push_back(createNEM(nemConfiguration));
    }

  /* Construct a platform first (initialized) */
  std::unique_ptr<NEMManager> 
    pPlatform{rNEMBuilder_.buildNEMManager(uuid,
                                           nems,
                                           configPlatform_.getConfigurationUpdateRequest())};


  return pPlatform;
}

std::unique_ptr<EMANE::Application::NEM>
EMANE::Application::NEMDirector::createNEM(EMANE::NEMConfiguration *pNEMConfig)
{
  NEMLayers layers;
  if (pNEMConfig->isValid()) 
    {
      EMANE::LayerConfigurations::const_iterator layerIter = 
        pNEMConfig->getLayers().begin();

      for ( ; layerIter != pNEMConfig->getLayers().end(); ++layerIter) 
        {
          if ((*layerIter)->getType() == "phy") 
            {
              layers.push_back(rNEMBuilder_.buildPHYLayer(
                                   pNEMConfig->getNEMId(),
                                   (*layerIter)->getLibrary(),
                                   (*layerIter)->getConfigurationUpdateRequest()));
            }
          else if ((*layerIter)->getType() == "mac") 
            {
              layers.push_back(rNEMBuilder_.buildMACLayer(
                                   pNEMConfig->getNEMId(),
                                   (*layerIter)->getLibrary(),
                                   (*layerIter)->getConfigurationUpdateRequest()));
            }
          else if ((*layerIter)->getType() == "shim") 
            {
              layers.push_back(rNEMBuilder_.buildShimLayer(
                                   pNEMConfig->getNEMId(),
                                   (*layerIter)->getLibrary(),
                                   (*layerIter)->getConfigurationUpdateRequest()));
            }
           else if ((*layerIter)->getType() == "transport" && !pNEMConfig->isExternalTransport()) 
            {
              layers.push_back(rNEMBuilder_.buildTransportLayer(
                                   pNEMConfig->getNEMId(),
                                   (*layerIter)->getLibrary(),
                                   (*layerIter)->getConfigurationUpdateRequest()));
            }
        }// end for layers    
    }// end if valid
  else 
    {
      std::stringstream excStream;
      excStream << "NEM "
                << pNEMConfig->getNEMId()
                << ")"
                << " is NOT properly configured!" 
                << std::endl
                << std::endl
                << "Possible reason(s):"
                << std::endl
                << std::endl
                << " * " 
                << " NEM XML is missing one of phy|mac|transport."
                << std::endl
                << std::ends;

      throw ConfigureException(excStream.str());    
    }

  return rNEMBuilder_.buildNEM(pNEMConfig->getNEMId(), 
                               layers,
                               pNEMConfig->getConfigurationUpdateRequest(),
                               pNEMConfig->isExternalTransport());
}
