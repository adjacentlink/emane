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

#ifndef EMANETRANSPORTADAPATER_HEADER_
#define EMANETRANSPORTADAPATER_HEADER_

#include "emane/component.h"
#include "emane/downstreamtransport.h"
#include "emane/transport.h"
#include "emane/buildable.h"

#include <memory>

namespace EMANE
{
  namespace Application
  {
    /**
     * @class TransportAdapter
     *
     * @brief Transport Adapter interface. A Transport Adapter combines
     *        with a Transport to connect with its respective NEM stack.
     *        A TransportAdapter implementation passes traffic between
     *        the associated Transport and the Tranport's NEM.
     */
    class TransportAdapter : public Component,
                             public DownstreamTransport,
                             public Buildable
    {
    public:
      /**
       * Destroys an instance
       */
      virtual ~TransportAdapter(){}
      
      /**
       * Set the Transport
       *
       * @param pTransport pointer to the Transport instance to be
       *                   
       */
      virtual void setTransport(std::unique_ptr<Transport> & pTransport) = 0;

      /**
       * Gets the NEM Id
       *
       * @return NEM Id
       */
      NEMId getNEMId() const {return id_;}
      
    protected:
      TransportAdapter(NEMId id):
        id_(id){}
      
      const NEMId id_;
    };
  }
}

#endif //EMANETRANSPORT_HEADER_
