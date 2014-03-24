/*
 * Copyright (c) 2010 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANECOMPONENTTYPES_HEADER_
#define EMANECOMPONENTTYPES_HEADER_


namespace EMANE
{
  enum ComponentType
  {
    COMPONENT_ALL                     = 0x0000, /**< All NEM  components*/
    COMPONENT_PHYLAYER                = 0x0001, /**< Concreate PHY layer internal to the NEM  */
    COMPONENT_PHYILAYER               = 0x0002, /**< PHY layer implementation                 */
    COMPONENT_MACLAYER                = 0x0004, /**< Concreate MAC layer internal to the NEM  */
    COMPONENT_MACILAYER               = 0x0008, /**< MAC layer implementation                 */
    COMPONENT_SHIMLAYER               = 0x0010, /**< Concreate Shim layer internal to the NEM */
    COMPONENT_SHIMILAYER              = 0x0020, /**< Shim layer implementation                */
    COMPONENT_TRANSPORTLAYER          = 0x0040, /**< Concreate transport layer                */
    COMPONENT_TRANSPORTILAYER         = 0x0080, /**< Transport layer implementation           */
    COMPONENT_GENERATORLAYER          = 0x0100, /**< Concreate generator layer                */
    COMPONENT_GENERATORILAYER         = 0x0200, /**< Generator layer implementation           */
    COMPONENT_EVENTAGENTLAYER         = 0x0400, /**< Concreate  agent layer                   */
    COMPONENT_EVENTAGENTILAYER        = 0x0800, /**< Agent layer implementation               */
  };
}


#endif //EMANECOMPONENTTYPES_HEADER_

