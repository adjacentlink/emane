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

#include "transportdirector.h"
#include <sstream>

/*
 * Constructor
 *
 * @param filename reference to the base XML filename
 * @param builder reference to the TransportBuilder
 */
EMANE::Application::TransportDirector::TransportDirector(const std::string &filename, 
                                                         TransportBuilder &builder):
  transportConfig_(filename),
  rTransportBuilder_(builder)
{}
  
/*
 * Destructor
 */
EMANE::Application::TransportDirector::~TransportDirector()
{}

/*
 * Constructs the event agent and generators
 *
 */
std::unique_ptr<EMANE::Application::TransportManager>
EMANE::Application::TransportDirector::construct(const uuid_t & uuid)
{
  /* Now go through configuration of each instance and build appropriately */
  EMANE::TransportInstanceConfigurations::const_iterator instanceIter = 
    transportConfig_.getInstances().begin();
  
  TransportAdapters adapters;
  for ( ; instanceIter != transportConfig_.getInstances().end(); ++instanceIter)
    {
      /* Create the adapter for this transport */
     
      auto pTransport = createTransport(*instanceIter);

      adapters.push_back(rTransportBuilder_.buildTransportAdapter(pTransport,
                                                                  (*instanceIter)->getConfigurationUpdateRequest()));
    }

  /* Build TransportManager */
  return rTransportBuilder_.buildTransportManager(uuid,
                                                  adapters,
                                                  transportConfig_.getConfigurationUpdateRequest());
}

/*
 * Uses the passed-in builder to create a Transport and return
 * a pointer to it.
 *
 * @param pTIConfig Pointer to the Instance XML configuration
 *
 * @exception ConfigureException
 */
std::unique_ptr<EMANE::Transport>
EMANE::Application::TransportDirector::createTransport(TransportInstanceConfiguration *pTIConfig)
{
  EMANE::LayerConfigurations::const_iterator transportIter = 
    pTIConfig->getLayers().begin();

  EMANE::LayerConfiguration *pTransportConfig = (*transportIter);

  if (pTransportConfig == 0)
    {
      std::stringstream excStream;
      excStream << "Transport inside instance "
                << pTIConfig->getId() << " is NOT properly configured!" 
                << std::ends;
      throw ConfigureException(excStream.str());    
    }
  
  return rTransportBuilder_.buildTransport(
                                 pTIConfig->getId(),
                                 pTransportConfig->getLibrary(),
                                 pTransportConfig->getConfigurationUpdateRequest());
}
