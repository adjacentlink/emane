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

#ifndef EMANERADIOSERVICEPROVIDER_HEADER_
#define EMANERADIOSERVICEPROVIDER_HEADER_

#include "emane/spectrumserviceprovider.h"

namespace EMANE
{
  /**
   * @class RadioServiceProvider
   *
   * @brief The RadioServiceProvider interface provides 
   * access to radio (RF) model specific services.
   *
   * @note Radio model services are only valid when using
   * the FrameworkPHY, the emualtor's built in physical layer.
   *
   * @note Service references are not valid until Component::initialize().
   * Very bad things will happen if services are accessed prior the
   * initialization transition.
   */
  class RadioServiceProvider
  {
  public:
    /**
     * Destroys and instance
     */
    virtual ~RadioServiceProvider(){};

    /**
     * Gets a reference to the SpectrumServiceProvider
     *
     * @return SpectrumServiceProvider reference
     */
    virtual SpectrumServiceProvider & spectrumService() = 0;

  protected:
    RadioServiceProvider() = default;
  };
}

#endif //EMANERADIOSERVICEPROVIDER_HEADER_
