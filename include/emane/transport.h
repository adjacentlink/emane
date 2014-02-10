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

#ifndef EMANETRANSPORT_HEADER_
#define EMANETRANSPORT_HEADER_

#include "emane/component.h"
#include "emane/upstreamtransport.h"
#include "emane/platformserviceuser.h"
#include "emane/buildable.h"
#include "emane/runningstatemutable.h"

#include <string>
#include <list>

namespace EMANE
{
  /**
   * @class Transport
   *
   * @brief Base class for all transports
   */
  class Transport : public Component,
                    public UpstreamTransport,
                    public PlatformServiceUser,
                    public Buildable,
                    public RunningStateMutable
  {
  public:
    /**
     * Destroys an instance
     */
    virtual ~Transport(){}

    /**
     * Gets the NEM identifier.
     *
     * @return NEM identifier
     */
    NEMId getNEMId() const { return id_; }
    
  protected:
    Transport(NEMId id, PlatformServiceProvider *pPlatformService):
      PlatformServiceUser(pPlatformService),
      id_(id)
     { }

    /**
     * Holds the NEM identifier.
     */
    const NEMId id_;
  };
}


#define DECLARE_TRANSPORT(X)                                                                \
  extern "C"  EMANE::Transport * create(EMANE::NEMId id, EMANE::PlatformServiceProvider *p) \
  {return new X(id,p);}                                                                     \
  extern "C"  void destroy(EMANE::Transport * p)                                            \
  {delete p;}
#endif //EMANETRANSPORT_HEADER_
