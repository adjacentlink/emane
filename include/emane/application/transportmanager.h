/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANEAPPLICATIONTRANSPORTMANAGER_HEADER_
#define EMANEAPPLICATIONTRANSPORTMANAGER_HEADER_

#include "emane/component.h"
#include "emane/buildable.h"

#include "emane/application/transportadapter.h"

#include <memory>
#include <uuid.h>

namespace EMANE
{
  namespace Application
  {
    /**
     * @class TransportManager
     *
     * @brief Manages all instantiated transports 
     *
     * @details Virtual base class for transport containers.
     */
    class TransportManager : public Component,
                             public Buildable
    {
    public:
      /**
       * Destroys an instance
       */
      virtual ~TransportManager(){}
      
      /**
       * Add a Transport Adapter
       * 
       * @param pTransportAdapter to add
       */
      virtual void add(std::unique_ptr<TransportAdapter> & pTransportAdapter) = 0;

      /**
       * Gets the UUID
       *
       * @return UUID
       */
      const uuid_t & getUUID() const
      {
        return uuid_;
      }
      
    protected:
      /**
       * Creates an TransportManager instance
       *
       * @param uuid Instance UUID
       */
      TransportManager(const uuid_t & uuid)
      {
        uuid_copy(uuid_,uuid);
      }

      uuid_t uuid_;
    };
  }
}

#endif // EMANEAPPLICATIONTRANSPORTMANAGER_HEADER_
