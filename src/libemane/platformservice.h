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

#ifndef EMANEPLATFORMSERVICE_HEADER_
#define EMANEPLATFORMSERVICE_HEADER_

#include "emane/componenttypes.h"
#include "emane/platformserviceprovider.h"
#include "timerserviceproxy.h"
#include "eventserviceproxy.h"

#include <memory>

namespace EMANE
{

  class PlatformServiceUser;

  /**
   * @class PlatformService
   *
   * @brief platform service
   */
  class PlatformService : public PlatformServiceProvider
  {
  public:
    PlatformService();

    ~PlatformService();

    TimerServiceProvider & timerService() override;

    LogServiceProvider & logService() override;

    EventServiceProvider & eventService() override;

    FileDescriptorServiceProvider & fileDescriptorService() override;

    /**
     * set the platform service user
     *
     * @param buildId PlatformServiceUser build id
     * @param pPlatformServiceUser pointer to the PlatformServiceUser
     */
    void setPlatformServiceUser(BuildId buildId,
                                PlatformServiceUser * pPlatformServiceUser);

    /**
     * set the FileDescriptorServiceProvider
     *
     * @param pFileDescriptorServiceProvider pointer to the
     * FileDescriptorServiceProvider
     */
    void setFileDescriptorServiceProvider(FileDescriptorServiceProvider *
                                          pFileDescriptorServiceProvider);

  private:
    TimerServiceProxy timerServiceProxy_;
    EventServiceProxy eventServiceProxy_;
    FileDescriptorServiceProvider * pFileDescriptorService_;
  };
}


#endif // EMANEPLATFORMSERVICE_HEADER_
