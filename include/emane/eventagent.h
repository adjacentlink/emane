/*
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

#ifndef EMANEEVENTAGENT_HEADER_
#define EMANEEVENTAGENT_HEADER_

#include "emane/component.h"
#include "emane/types.h"
#include "emane/platformserviceuser.h"
#include "emane/componenttypes.h"
#include "emane/buildable.h"

#include <list>

namespace EMANE
{
  /**
   * @class EventAgent
   *
   * @brief Base class for all event agents.
   */
  class EventAgent : public Component,
                     public PlatformServiceUser,
                     public Buildable
  {
  public:
    /**
     * Destroys an instance
     */
    virtual ~EventAgent(){}

  protected:
    EventAgent(NEMId id, PlatformServiceProvider *pPlatformService) :
      PlatformServiceUser(pPlatformService),
      id_(id)
    {}

    NEMId id_;
  };
}

#define DECLARE_EVENT_AGENT(X)                                                                \
  extern "C"  EMANE::EventAgent * create(EMANE::NEMId id, EMANE::PlatformServiceProvider *p)  \
  {return new X(id,p);}                                                                       \
  extern "C"  void destroy(EMANE::EventAgent * p)                                             \
  {delete p;}

#endif //EMANEEVENTAGENT_HEADER_
