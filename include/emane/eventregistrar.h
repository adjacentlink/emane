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

#ifndef EMANEEVENTREGISTRAR_HEADER_
#define EMANEEVENTREGISTRAR_HEADER_

#include "emane/types.h"

namespace EMANE
{
  /**
   * @class EventRegistrar
   *
   * @brief The EventRegistrar allows NEM layers to register
   * to receive events. Only those events registered will be 
   * received.
   *
   * @note Events are addressed to all NEMs or a specific NEM.
   * Registered events will be delivered when the event is directly
   * addressed to the registered NEM or when the event is addressed
   * to all NEMs.
   *
   * @note Registration may only occur during Component::initialize()
   */
  class EventRegistrar
  {
  public:
    /**
     * Destroys an instance
     */
    virtual ~EventRegistrar(){}
    
    /**
     * Registers to receive an event
     *
     * @param eventId Id of the desired event
     */
    virtual void registerEvent(EventId eventId) = 0;

  protected:
    EventRegistrar() = default;
  };
}

#endif // EMANEEVENTREGISTRAR_HEADER_
