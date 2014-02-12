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

#ifndef EMANEAPPLICATIONNEMMANAGERIMPL_HEADAER_
#define EMANEAPPLICATIONNEMMANAGERIMPL_HEADAER_

#include "emane/application/nemmanager.h"

#include <map>
#include <memory>
#include <thread>

#include <ace/SOCK_Stream.h>
#include <ace/SOCK_Acceptor.h>
#include "controlportservice.h"
#include <ace/Acceptor.h>

namespace EMANE
{
  namespace Application
  {
    /**
     * @class NEMManagerImpl
     *
     * @brief Implementation of Platform interface. Contains and manages NEMs.
     *
     */
    class NEMManagerImpl : public NEMManager
    {
    public:
      /**
       * Constructor
       *
       * @param id Platform id as configured in XML
       */
      NEMManagerImpl(PlatformId id);

      ~NEMManagerImpl();

      void add(std::unique_ptr<NEM> & pNEM) override;

      void initialize(Registrar & registrar) override;

      void configure(const ConfigurationUpdate & update) override;
    
      void start() override;

      void postStart() override;

      void stop() override;

      void destroy()
        throw() override;
    
    private:
      using PlatformNEMMap = std::map<NEMId,std::unique_ptr<NEM>>;
    
      PlatformNEMMap platformNEMMap_;

      PlatformId platformId_;

      ACE_Acceptor<EMANE::ControlPort::ControlPortService,ACE_SOCK_ACCEPTOR> acceptor_;

      ACE_INET_Addr OTAManagerGroupAddr_;
      std::string sOTAManagerGroupDevice_;
      std::uint8_t u8OTAManagerTTL_;
      bool bOTAManagerChannelLoopback_;
      bool bOTAManagerChannelEnable_;
    
      ACE_INET_Addr eventServiceGroupAddr_;
      std::string sEventServiceDevice_;
      std::uint8_t u8EventServiceTTL_;
      ACE_INET_Addr controlPortAddr_;
      std::string sAntennaProfileManifestURI_;
      std::thread thread_;
    };
  }
}

#endif // EMANEAPPLICATIONNEMMANAGERIMPL_HEADAER_
