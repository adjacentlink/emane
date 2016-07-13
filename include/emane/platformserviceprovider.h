/*
 * Copyright (c) 2013-2014,2016 - Adjacent Link LLC, Bridgewater,
 * New Jersey
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
 */

#ifndef EMANEPLATFORMSERVICEPROVIDER_HEADER_
#define EMANEPLATFORMSERVICEPROVIDER_HEADER_

#include "emane/logserviceprovider.h"
#include "emane/eventserviceprovider.h"
#include "emane/timerserviceprovider.h"
#include "emane/filedescriptorserviceprovider.h"

namespace EMANE
{
  /**
   * @class PlatformServiceProvider
   *
   * @brief The PlatformServiceProvider interface provides
   * access to emulator services.
   *
   * @note Service references are not valid until Component::initialize().
   * Very bad things will happen if services are accessed prior the
   * initialization transition.
   */
  class PlatformServiceProvider
  {
  public:
    /**
     * Destroys an instance
     */
    virtual ~PlatformServiceProvider(){};

    /**
     * Gets a reference to the TimerServiceProvider
     *
     * @return TimerServiceProvider reference
     */
    virtual TimerServiceProvider & timerService() = 0;

    /**
     * Gets a reference to the LogServiceProvider
     *
     * @return LogServiceProvider reference
     */
    virtual LogServiceProvider & logService() = 0;

    /**
     * Gets a reference to the EventServiceProvider
     *
     * @return EventServiceProvider reference
     */
    virtual EventServiceProvider & eventService() = 0;

    virtual FileDescriptorServiceProvider &
    fileDescriptorService() = 0;

  protected:
    PlatformServiceProvider() = default;
  };
}

#endif //EMANEPLATFORMSERVICEPROVIDER_HEADER_
