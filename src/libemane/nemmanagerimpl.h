/*
 * Copyright (c) 2013-2015,2017 - Adjacent Link LLC, Bridgewater,
 * New Jersey
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
#include "emane/inetaddr.h"
#include "controlportservice.h"

#include <map>
#include <memory>

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
      NEMManagerImpl(const uuid_t & uuid);

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

      ControlPort::Service controlPortService_;

      INETAddr OTAManagerGroupAddr_;
      std::string sOTAManagerGroupDevice_;
      std::uint8_t u8OTAManagerTTL_;
      std::uint32_t u32OTAManagerMTU_;
      bool bOTAManagerChannelLoopback_;
      bool bOTAManagerChannelEnable_;
      Seconds OTAManagerPartCheckThreshold_;
      Seconds OTAManagerPartTimeoutThreshold_;

      INETAddr eventServiceGroupAddr_;
      std::string sEventServiceDevice_;
      std::uint8_t u8EventServiceTTL_;
      INETAddr controlPortAddr_;
      std::string sAntennaProfileManifestURI_;
    };
  }
}

#endif // EMANEAPPLICATIONNEMMANAGERIMPL_HEADAER_
