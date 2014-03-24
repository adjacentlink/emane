/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2010 - DRS CenGen, LLC, Columbia, Maryland
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
 *
 */

#ifndef EMANEGENERATORSEELGENERATOR_HEADER_
#define EMANEGENERATORSEELGENERATOR_HEADER_

#include "eelloaderpluginfactory.h"
#include "emane/eventgenerator.h"

#include <vector>
#include <map>
#include <string>

#include <ace/Thread.h>
#include <ace/Thread_Mutex.h>
#include <ace/Condition_T.h>

namespace EMANE
{
  namespace Generators
  {
    namespace EEL
    {
      class Generator : public EventGenerator
      {
      public:
        Generator(PlatformServiceProvider *pPlatformService);
        
        ~Generator();
        
        void initialize(Registrar & registrar) override;
        
        void configure(const ConfigurationUpdate & update) override;
        
        void start() override;
        
        void stop() override;
        
        void destroy() throw() override;
        
      private:
        using InputFileNameVector = std::vector<std::string>;
        using PluginFactoryList = std::list<LoaderPluginFactory *>;
        using EventPluginMap = std::map<std::string, std::pair<LoaderPlugin *,EventPublishMode>>;
        
        ACE_thread_t thread_;
        ACE_Thread_Mutex mutex_;
        ACE_Condition<ACE_Thread_Mutex> cond_;
        bool bCancel_;
        InputFileNameVector inputFileNameVector_;
        PluginFactoryList pluginFactoryList_;
        EventPluginMap eventPluginMap_;
        
        ACE_THR_FUNC_RETURN generate();
        
        bool waitAndSendEvents(const ACE_Time_Value & tvTestStartTime, 
                               float fCurrentTime);
      };
    }
  }
}

#endif // EMANEGENERATORSEELGENERATOR_HEADER_
