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

#ifndef EMANEPHYTYPES_HEADER_
#define EMANEPHYTYPES_HEADER_

#include "emane/types.h"

namespace EMANE
{
  /* 
   * Resevered Phy Type Id Range (0,32767]
   *.
   * Local Phy Type Id Range     [32768,65535)
   */
  
  /*
   * Bypass/sample Phy Type. Contrib. DRS CenGen, LLC<labs at cengen dot com> Reg. 2008.10.02
   */
  const RegistrationId REGISTERED_EMANE_PHY_BYPASS = 0x0001;

  /*
   * Low Fidelity 802.11 Phy Type.  Contrib. DRS CenGen, LLC<labs at cengen dot com> Reg. 2008.09.22
   *
   * deprecated, use 802_11_ABG 
   * const RegistrationId REGISTERED_EMANE_PHY_LEGACY_802_11 = 0x0002;
   */

  /*
   * High Fidelity 802.11 Phy Type.  Contrib. DRS CenGen, LLC<labs at cengen dot com> Reg. 2008.09.22
   *
   * deprecated, use universal phy 
   * const RegistrationId REGISTERED_EMANE_PHY_IEEE_802_11_ABG = 0x0003;
   *
   */

  /*
   * RF Pipe Phy Type.  Contrib. DRS CenGen, LLC<labs at cengen dot com> Reg. 2008.09.22
   *
   * deprecated, use universal phy 
   * const RegistrationId REGISTERED_EMANE_PHY_RF_PIPE = 0x0004;
   *
   */

  /*
   * Comm Effect Type.  Contrib. DRS CenGen, LLC<labs at cengen dot com> Reg. 2010.05.04
   */
  const RegistrationId REGISTERED_EMANE_PHY_COMM_EFFECT = 0x0005;

  /*
   * Universal Type.  Contrib. DRS CenGen, LLC<labs at cengen dot com> Reg. 2010.08.24
   *
   * deprecated, use framework phy 
   * const RegistrationId REGISTERED_EMANE_PHY_UNIVERSAL = 0x0006;
   */


  /*
   * Framework Type.  Contrib. Adjacent Link LLC <labs at adjacentlink dot com> Reg. 2013.12.01
   */
  const RegistrationId REGISTERED_EMANE_PHY_FRAMEWORK = 0x0007;

}


#endif //EMANEPHYTYPES_HEADER_
