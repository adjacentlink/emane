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

#ifndef EMANEAPPLICATIONNEMIMPL_HEADER_
#define EMANEAPPLICATIONNEMIMPL_HEADER_

#include "emane/application/nem.h"

#include "nemlayerstack.h"
#include "nemotaadapter.h"
#include "nemnetworkadapter.h"

#include "emane/component.h"

#include <memory>

namespace EMANE
{
  namespace Application
  {
    /**
     * @class NEMImpl
     *
     * @brief Implementation of the Network emulation module consisting of NEM
     * components, OTA Adapater and network adapter
     */
    class NEMImpl : public NEM
    {
    public:
      /**
       * Constructor
       *
       * @param id NEMid of this NEM
       * @param pNEMLayerStack pointer to a NEMLayerStack to contain in this NEM
       *
       */
      NEMImpl(NEMId id, 
              std::unique_ptr<NEMLayerStack> & pNEMLayerStack);
    
      ~NEMImpl();

      void initialize(Registrar & registrar) override;
    
      void configure(const ConfigurationUpdate & update) override;
    
      void start() override;

      void postStart() override;

      void stop() override;
    
      void destroy()
        throw() override;

      /**
       * Get the NEM's NEMId
       *
       * @return the NEM's NEMId
       */    
      NEMId getNEMId() const override;

    private:
      std::unique_ptr<NEMLayerStack> pNEMLayerStack_;
      NEMId id_;
    
      NEMOTAAdapter     NEMOTAAdapter_;
      NEMNetworkAdapter NEMNetworkAdapter_;

      ACE_INET_Addr platformEndpointAddr_;
      ACE_INET_Addr transportEndpointAddr_;
    
      // prevent NEM copies
      NEMImpl(const NEMImpl &) = delete;
      NEMImpl & operator=(const NEMImpl &) = delete;
    };
  }
}

#endif // EMANEAPPLICATIONNEMIMPL_HEADER_
