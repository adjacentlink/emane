/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
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
 * * Neither the name of Adjacent Link LLC nor the names of its
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

#ifndef EMANETRANSMITTER_HEADER_
#define EMANETRANSMITTER_HEADER_

#include "emane/types.h"

#include <list>

namespace EMANE
{
  /**
   * @class Transmitter
   *
   * @brief Holds transmitter id and power level
   * 
   * @note Instances are immutable
   */
  class Transmitter
  {
  public:
    /**
     * Creates a Transmitter instance
     *
     * @param id NEM id of the transmitter
     * @param id Power level of the transmitter in dBm
     */
    Transmitter(NEMId id,
                double dPowerdBm);
    
    /**
     * Creates a Transmitter instance by copy
     */
    Transmitter(const Transmitter & rhs);
    
    /**
     * Gets the transmitter id
     *
     * @return NEM Id
     */
    EMANE::NEMId getNEMId() const;
    
    /**
     * Gets the transmitter power in dBm
     *
     * return power
     */
    double getPowerdBm() const;
    
  private:
    EMANE::NEMId id_;
    double dPowerdBm_;
  };
  
  using Transmitters = std::list<Transmitter>;
}

#include "emane/transmitter.inl"

#endif // EMANETRANSMITTER_HEADER_
