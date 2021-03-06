/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2008 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANEMODELSIEEE802ABGUNICASTRTSCTSTXSTATE_HEADER_
#define EMANEMODELSIEEE802ABGUNICASTRTSCTSTXSTATE_HEADER_


#include "transmissiontxstate.h"
#include "emane/utils/singleton.h"

namespace EMANE
{
  namespace Models
  {
    namespace IEEE80211ABG
    {
      /**
       *
       * @brief class used to define the transitions for the mac unicast rts cts tx state
       *
       */
      class UnicastRtsCtsTxState: public TransmissionTxState,
                                  public Utils::Singleton<UnicastRtsCtsTxState>
      {
      public:
        ~UnicastRtsCtsTxState();

        bool process(MACLayer *, DownstreamQueueEntry &);

        std::pair<TimePoint,bool> getWaitTime(DownstreamQueueEntry &);

        const char *statename();

      protected:
        UnicastRtsCtsTxState();
             
      };
  
      using UnicastRtsCtsTxStateSingleton = UnicastRtsCtsTxState;
    }
  }
}

#endif //EMANEMODELSIEEE802ABGUNICASTRTSCTSTXSTATE_HEADER_
