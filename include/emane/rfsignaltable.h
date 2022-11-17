/*
 * Copyright (c) 2022 - Adjacent Link LLC, Bridgewater, New Jersey
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
 * * Neither the name of Adjacent Link, LLC nor the names of its
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


#ifndef EMANERFSIGNALTABLE_HEADER_
#define EMANERFSIGNALTABLE_HEADER_

#include "emane/types.h"
#include "emane/statisticregistrar.h"
#include "emane/configurationinfo.h"
#include "emane/registrar.h"

namespace EMANE
{
 /**
  *
  * @class RFSignalTable
  *
  * @brief Manages RF Signal Table
  *
  */
  class RFSignalTable
  { 
    public:
     /**
      * Creates a RFSignalTable instance
      *
      * @param nemId NEM id
      */
      RFSignalTable(NEMId nemId);
    
     /**
      * Destroys an instance
      *
      */
      ~RFSignalTable();

     /**
      * Registers the table with the EMANE framework
      *
      */
     void initialize(EMANE::Registrar & registrar);

     /**
      * Configures the table via the EMANE framework
      *
      */
     void configure(const EMANE::ConfigurationUpdate & update);

     /**
      * Updates RF Signal Table
      *
      * @param src                     src NEM
      * @param rxAntennaId             rx antenna Id
      * @param frequencyHz             frequency in Hz
      * @param fRxPower_mW             rx power in milli watts
      * @param fNoiseFloor_mW          nosie floor milli watts
      * @param freceiverSensitivity_mW receiver sensitivity milli watts
      *
      */
     void update(NEMId src,
                 AntennaIndex rxAntennaId,
                 std::uint64_t frequencyHz,
                 double fRxPower_mW,
                 double fNoiseFloor_mW,
                 double fReceiverSensitivity_mW);

     /**
      * Clear all table table entries for a given antenna
      *
      * @param rxAntennaId             rx antenna Id
      *
      */
     void reset(AntennaIndex rxAntennaId);

     /**
      * Clear all table entries
      *
      */
     void resetAll();

     static constexpr const char * CONFIG_PREFIX{"rfsignaltable."}; 

   private:
     class Implementation;

     std::unique_ptr<Implementation> pImpl_;

     RFSignalTable(const RFSignalTable &) = delete;

     RFSignalTable & operator=(const RFSignalTable &) = delete;
  };
}

#endif // EMANERFSIGNALTABLE_HEADER_
