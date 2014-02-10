/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2011 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANEBUILDABLE_HEADER_
#define EMANEBUILDABLE_HEADER_

#include "emane/types.h"

namespace EMANE
{
  /**
   * @class Buildable
   *
   * @brief A interface to mark objects with a unique application
   * wide build Id. Builders manipulate buildable objects to register
   * their association with other application objects and to enforce
   * composition rules.
   */
  class Buildable
  {
  public:
    /**
     * Destroys an instance
     */
    virtual ~Buildable(){}

    /**
     * Gets the application wide unique BuildId of this object.
     *
     * @return the BuildId
     */
    BuildId getBuildId() const;
    
    /**
     * Sets the application wide unique BuildId of this object.
     * Builders assign a BuildId to objects at instantiation.
     *
     * @param bid the BuildId
     *
     * @note: This method is for internal framework use only.
     */
    void setBuildId(BuildId bid);

  protected:
    Buildable():buildId_{}{}

  private:
    BuildId buildId_;
  };
}

#include "emane/buildable.inl"

#endif //EMANEBUILDABLE_HEADER_
