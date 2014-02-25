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

#ifndef EMANEEVENTDIRECTOR_HEADER_
#define EMANEEVENTDIRECTOR_HEADER_

#include "emane/application/eventgeneratorbuilder.h"
#include "eventserviceconfiguration.h"
#include "emane/configureexception.h"

#include <string>
#include <uuid.h>

/**
 * @class EMANE::EventDirector eventdirector.h "eventdirector.h"
 *
 * @brief Director used to build EventGenerator (s) with EventGeneratorBuilder.
 *
 * The EventDirector class is the implementation of the Director
 * part of the Builder design pattern from the GoF book (see 
 * reference, at end of file). Its task is to manage the creation
 * of EventGenerators with the use of the EventGeneratorBuilder
 *
 * @sa EventGeneratorBuilder EventGenerator
 */

namespace EMANE
{
  namespace Application
  {
  class EventDirector
  {
  public:
    /**
     * Constructor
     *
     * @param filename reference to the base XML filename
     * @param builder reference to the EventGeneratorBuilder

     * @throw ParseException when an XML  parse error occurs.
     * @throw ValidateException when an XML validation error occurs.
     */
    EventDirector(const std::string &filename, 
                  EventGeneratorBuilder &builder);

    /**
     * Destructor
     */
    ~EventDirector();
  
    /**
     * Constructs the passed-in platform
     *
     * @param uuid Instance UUID
     */
    std::unique_ptr<EventGeneratorManager> construct(const uuid_t & uuid);

  private:
    /**
     * Container for configuration data
     */
    EventServiceConfiguration eventServiceConfig_;

    /**
     * Container for the Event Builder
     */
    EventGeneratorBuilder & builder_;
  };

/* 
 * Reference:
 *  Erich Gamma, Richard Helm, Ralph Johnson, and John Vlissides.
 *  Design Patterns: Elements of Reusable Object-Oriented Software.
 *  Addison-Wesley, Reading MA, 1995
 *  Builder, p 97
 */
  }
}

#endif //EMANEEVENTDIRECTOR_HEADER_
