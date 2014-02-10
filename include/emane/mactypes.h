/*
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

#ifndef EMANEMACTYPES_HEADER_
#define EMANEMACTYPES_HEADER_

#include "emane/types.h"

namespace EMANE
{
  /* 
   * Resevered Mac Type Id Range (0,32767]
   *.
   * Local Mac Type Id Range     [32768,65535)
   */
  
  /*
   * Bypass/sample Mac Type. Contrib. DRS CenGen, LLC<labs at cengen dot com> Reg. 2008.10.02
   */
  const RegistrationId REGISTERED_EMANE_MAC_BYPASS = 0x0001;

  /*
   * Low Fidelity 802.11 Mac Type.  Contrib. DRS CenGen, LLC<labs at cengen dot com> Reg. 2008.10.02
   */
  const RegistrationId REGISTERED_EMANE_MAC_LEGACY_802_11 = 0x0002;

  /*
   * High Fidelity 802.11 Mac Type.  Contrib. DRS CenGen, LLC<labs at cengen dot com> Reg. 2008.10.02
   */
  const RegistrationId REGISTERED_EMANE_MAC_IEEE_802_11_ABG = 0x0003;

  /*
   * RF Pipe Mac Type.  Contrib. DRS CenGen, LLC<labs at cengen dot com> Reg. 2008.10.02
   */
  const RegistrationId REGISTERED_EMANE_MAC_RF_PIPE = 0x0004;

  /*
   * TDMA Mac Type.  Contrib. DRS CenGen, LLC<labs at cengen dot com> Reg. 2012.03.03
   */
  const RegistrationId REGISTERED_EMANE_MAC_TDMA = 0x0005;

}


#endif //EMANEMACTYPES_HEADER_
