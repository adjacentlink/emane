/*
 * Copyright (c) 2025 - Adjacent Link LLC, Bridgewater, New Jersey
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
 *
 */

#ifndef EMANEGENERATORSEELLOADERPATHLOSSEX_HEADER_
#define EMANEGENERATORSEELLOADERPATHLOSSEX_HEADER_

#include "emane/generators/eel/loaderplugin.h"
#include "emane/events/pathlossex.h"

#include <map>

namespace EMANE
{
  namespace Generators
  {
    namespace EEL
    {
      class LoaderPathlossEx : public LoaderPlugin
      {
      public:
        LoaderPathlossEx();

        ~LoaderPathlossEx();

        void load(const ModuleType & modelType,
                  const ModuleId   & moduleId,
                  const EventType  & eventType,
                  const InputArguments & args) override;

        EventInfoList getEvents(EventPublishMode mode) override;

      private:
        using PathlossEntryMap = std::map<ModuleId,
                                          Events::PathlossEx::FrequencyPathlossMap>;
        using PathlossEntryCache = std::map<ModuleId,
                                            PathlossEntryMap>;
        PathlossEntryCache pathlossEntryCache_;
        PathlossEntryCache pathlossDeltaEntryCache_;

        void loadPathlossExCache(NEMId dstNEM,
                                 NEMId srcNEM,
                                 std::uint64_t u64FrequencyHz,
                                 float fForwardPathlossdB,
                                 float fReversePathlossdB,
                                 PathlossEntryCache & cache);
      };
    }
  }
}

#endif // EMANEGENERATORSEELLOADERPATHLOSS_HEADER_
