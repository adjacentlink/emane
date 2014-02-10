/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2008-2011 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANEAPPLICATIONEVENTGENERATORBUILDER_HEADER_
#define EMANEAPPLICATIONEVENTGENERATORBUILDER_HEADER_

#include "emane/application/eventgeneratormanager.h"
#include "emane/configurationupdate.h"

#include <string>
#include <list>
#include <memory>

namespace EMANE
{
  namespace Application
  {
    using EventGenerators = std::list<std::unique_ptr<EventGenerator>>;

    /**
     * @class EventGeneratorBuilder
     *
     * @brief Provides methods for constructing event generators and a
     * manager to contain and control them as a group.
     *
     * Reference:
     * Erich Gamma, Richard Helm, Ralph Johnson, and John Vlissides.
     * Design Patterns: Elements of Reusable Object-Oriented Software.
     * Addison-Wesley, Reading MA, 1995
     * Builder, p 97
     */
    class EventGeneratorBuilder
    {
    public:
      EventGeneratorBuilder();
    
      ~EventGeneratorBuilder();
    
      /**
       * Build an EventGeneratorManager
       *
       * @param generators the generators to manage
       * @param request configuration update
       *
       * @return Smart pointer to an initialized and configured 
       *         EventGeneratorManager
       *
       * @throw InitializeException when an error occurs during
       * initialization.
       * @throw BuildException when an error occurs building an NEM
       * from the specified layers.
       * @throw ConfigureException when an error occurs during
       * configure.
       */
      std::unique_ptr<EventGeneratorManager> 
      buildEventGeneratorManager(EventGenerators & generators,
                                 const ConfigurationUpdateRequest & request);

      /**
       * Build an EventGenerator
       *
       * @param sLibraryFile Name of the dll containing the generator
       * @param pItems pointer to ConfigurationItems list
       *
       * @return Smart pointer an initialized and configure
       * EventGenerator
       *
       * @throw Utils::FactoryException when a DLL load error occurs.
       * @throw InitializeException when an error occurs during
       * initialization.
       * @throw ConfigureException when an error occurs during
       * configure.
       */
      std::unique_ptr<EventGenerator>
      buildEventGenerator(const std::string & sLibraryFile,
                          const ConfigurationUpdateRequest & request,
                          bool bSkipConfigure = false);
    };
  }
}

#endif // EMANEAPPLICATIONEVENTGENERATORBUILDER_HEADER_
