/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2008 - DRS CenGen, LLC, Columbia, Maryland
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

#include "layerfactorymanager.h"

EMANE::LayerFactoryManager::LayerFactoryManager(){}

EMANE::LayerFactoryManager::~LayerFactoryManager()
{
  MACLayerFactoryMap::iterator macIter = macLayerFactoryMap_.begin();

  for(;macIter !=  macLayerFactoryMap_.end(); ++macIter)
    {
      delete macIter->second;
    }

  PHYLayerFactoryMap::iterator phyIter = phyLayerFactoryMap_.begin();

  for(;phyIter !=  phyLayerFactoryMap_.end(); ++phyIter)
    {
      delete phyIter->second;
    }

  ShimLayerFactoryMap::iterator shimIter = shimLayerFactoryMap_.begin();

  for(;shimIter !=  shimLayerFactoryMap_.end(); ++shimIter)
    {
      delete shimIter->second;
    }
}

const EMANE::MACLayerFactory & 
EMANE::LayerFactoryManager::getMACLayerFactory(const std::string & sLibraryFile)
{
  MACLayerFactoryMap::iterator iter;

  if((iter = macLayerFactoryMap_.find(sLibraryFile)) != macLayerFactoryMap_.end())
    {
      return *iter->second;
    }
  else
    {
      MACLayerFactory * pFactory =
        new  MACLayerFactory(sLibraryFile);

      macLayerFactoryMap_.insert(std::make_pair(sLibraryFile,pFactory));
      return *pFactory;
    }
}

const EMANE::PHYLayerFactory &
EMANE::LayerFactoryManager::getPHYLayerFactory(const std::string & sLibraryFile)
{
  PHYLayerFactoryMap::iterator iter;

  if((iter = phyLayerFactoryMap_.find(sLibraryFile)) != phyLayerFactoryMap_.end())
    {
      return *iter->second;
    }
  else
    {
      PHYLayerFactory * pFactory =
        new  PHYLayerFactory(sLibraryFile);

      phyLayerFactoryMap_.insert(std::make_pair(sLibraryFile,pFactory));
      return *pFactory;
    }
}

const EMANE::ShimLayerFactory &
EMANE::LayerFactoryManager::getShimLayerFactory(const std::string & sLibraryFile)
{
  ShimLayerFactoryMap::iterator iter;

  if((iter = shimLayerFactoryMap_.find(sLibraryFile)) != shimLayerFactoryMap_.end())
    {
      return *iter->second;
    }
  else
    {
      ShimLayerFactory * pFactory =
        new  ShimLayerFactory(sLibraryFile);

      shimLayerFactoryMap_.insert(std::make_pair(sLibraryFile,pFactory));
      return *pFactory;
    }
}
