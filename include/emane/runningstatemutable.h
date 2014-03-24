/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
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
 * * Neither the name of Adjacent Link LLC nor the names of its
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

#ifndef EMANERUNNINGSTATEMUTABLE_HEADER_
#define EMANERUNNINGSTATEMUTABLE_HEADER_

#include "emane/configurationupdate.h"

namespace EMANE
{
  /**
   * @class RunningStateMutable
   *
   * @brief The RunningStateMutable interface is used to allow
   * dynamic running-state configuration changes. Configuration is
   * placed on the NEMQueuedLayer functor queue for processing.
   */
  class RunningStateMutable
  {
  public:
    /**
     * Destroys an instance
     */
    virtual ~RunningStateMutable(){};

    /**
     * Process dynamic running-state configuration updates
     *
     * @param update Configuration update
     *
     * @note Running state configuration errors are indicated using
     * a validator. Validators can be registered using the ConfigurationRegistrar.
     */
    virtual void processConfiguration(const ConfigurationUpdate & update)
    {
      (void) update;
    };

  protected:
    RunningStateMutable() = default;
  };
}


#endif // EMANERUNNINGSTATEMUTABLE_HEADER_

