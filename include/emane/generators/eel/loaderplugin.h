/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANEGENERATORSEELLOADERPLUGIN_HEADER_
#define EMANEGENERATORSEELLOADERPLUGIN_HEADER_

#include "emane/generators/eel/types.h"

namespace EMANE
{
  namespace Generators
  {
    namespace EEL
    {
      /**
       * @class LoaderPlugin
       *
       * @brief Interface used to create an EEL loader plugin implementation.
       */
      class LoaderPlugin
      {
      public:
        /**
         * Destroys an instance
         */
        virtual ~LoaderPlugin(){};

        /**
         * Loads an EEL expression
         *
         * @param moduleType Module type
         * @param moduleId Module id
         * @param eventType The event type
         * @param args The event arguments
         *
         * @throw FormatException when a module load fails
         */
        virtual void load(const ModuleType & moduleType, 
                          const ModuleId   & moduleId, 
                          const EventType  & eventType,
                          const InputArguments & args) = 0;

        /**
         * Gets the events to publish
         *
         * @param mode Publish either the complete current known
         * list of events or just those items that have changed
         *
         * @returns event information to publish
         */
        virtual EventInfoList getEvents(EventPublishMode mode) = 0;
        
      protected:
        LoaderPlugin(){};
      };
    }
  }
}

#define DECLARE_EEL_LOADER_PLUGIN(X)                       \
  extern "C" EMANE::Generators::EEL::LoaderPlugin * create() \
  {return new X;}                                          \
  extern "C"  void destroy(EMANE::Generators::EEL::LoaderPlugin * p)  \
  {delete p;}

#endif // EMANEGENERATORSEELLOADERPLUGIN_HEADER_
