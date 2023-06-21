/*
 * Copyright (c) 2023 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "configurationvalidationmanager.h"
#include "configurehelpers.h"


EMANE::Models::BentPipe::ConfigurationValidationManager::ConfigurationValidationManager():
  bFinal_{}{}

std::pair<std::string,bool>
EMANE::Models::BentPipe::ConfigurationValidationManager::validate(const ConfigurationUpdate & update)
{
  TransponderConfigurations transponderConfigurations{};
  Antennas antennas{};

  for(const auto & item : update)
    {
      if(item.first == "transponder.receive.frequency")
        {
          for(const auto & entry : item.second)
            {
              configureTransponderValue(transponderConfigurations,
                                        entry.asString(),
                                        &TransponderConfiguration::setReceiveFrequencyHz);
            }
        }
      else if(item.first == "transponder.receive.bandwidth")
        {
          for(const auto & entry : item.second)
            {
              configureTransponderValue(transponderConfigurations,
                                        entry.asString(),
                                        &TransponderConfiguration::setReceiveBandwidthHz);
            }
        }
      else if(item.first == "transponder.receive.action")
        {
          for(const auto & entry : item.second)
            {
              configureTransponderValue(transponderConfigurations,
                                        entry.asString(),
                                        &TransponderConfiguration::setReceiveAction);
            }
        }
      else if(item.first == "transponder.receive.antenna")
        {
          for(const auto & entry : item.second)
            {
              configureTransponderValue(transponderConfigurations,
                                        entry.asString(),
                                        &TransponderConfiguration::setReceiveAntennaIndex);
            }
        }
      else if(item.first == "transponder.receive.enable")
        {
          for(const auto & entry : item.second)
            {
              configureTransponderValue(transponderConfigurations,
                                        entry.asString(),
                                        &TransponderConfiguration::setReceiveEnable);
            }
        }
      else if(item.first == "pcrcurveuri")
        {
          if(!bFinal_)
            {
              PCRManager pcrManager{};

              pcrManager.load(item.second[0].asString());

              for(const auto & entry: pcrManager.getCurveTable())
                {
                  pcrCurveIndexSet_.emplace(entry.first);
                }
            }
        }
      else if(item.first == "transponder.transmit.pcrcurveindex")
        {
          for(const auto & entry : item.second)
            {
              configureTransponderValue(transponderConfigurations,
                                        entry.asString(),
                                        &TransponderConfiguration::setPCRCurveIndex);
            }
        }
      else if(item.first == "transponder.transmit.frequency")
        {
          for(const auto & entry : item.second)
            {
              configureTransponderValue(transponderConfigurations,
                                        entry.asString(),
                                        &TransponderConfiguration::setTransmitFrequencyHz);
            }
        }
      else if(item.first == "transponder.transmit.bandwidth")
        {
          for(const auto & entry : item.second)
            {
              configureTransponderValue(transponderConfigurations,
                                        entry.asString(),
                                        &TransponderConfiguration::setTransmitBandwidthHz);
            }
        }
      else if(item.first == "transponder.transmit.ubend.delay")
        {
          for(const auto & entry : item.second)
            {
              configureTransponderValue(transponderConfigurations,
                                        entry.asString(),
                                        &TransponderConfiguration::setTransmitUbendDelay);
            }
        }
      else if(item.first == "transponder.transmit.datarate")
        {
          for(const auto & entry : item.second)
            {
              configureTransponderValue(transponderConfigurations,
                                        entry.asString(),
                                        &TransponderConfiguration::setTransmitDataRatebps);
            }
        }
      else if(item.first == "transponder.transmit.mtu")
        {
          for(const auto & entry : item.second)
            {
              configureTransponderValue(transponderConfigurations,
                                        entry.asString(),
                                        &TransponderConfiguration::setTransmitMTUBytes);
            }
        }
      else if(item.first == "transponder.transmit.power")
        {
          for(const auto & entry : item.second)
            {
              configureTransponderValue(transponderConfigurations,
                                        entry.asString(),
                                        &TransponderConfiguration::setTransmitPowerdBm);
            }
        }
      else if(item.first == "transponder.transmit.tosmap")
        {
          for(const auto & entry : item.second)
            {
              configureTransponderValue(transponderConfigurations,
                                        entry.asString(),
                                        &TransponderConfiguration::setTransmitProcessTOS);
            }
        }
      else if(item.first == "transponder.transmit.slotperframe")
        {
          for(const auto & entry : item.second)
            {
              configureTransponderValue(transponderConfigurations,
                                        entry.asString(),
                                        &TransponderConfiguration::setTransmitSlotsPerFrame);
            }
        }
      else if(item.first == "transponder.transmit.slotsize")
        {
          for(const auto & entry : item.second)
            {
              configureTransponderValue(transponderConfigurations,
                                        entry.asString(),
                                        &TransponderConfiguration::setTransmitSlotSize);
            }
        }
      else if(item.first == "transponder.transmit.txslots")
        {
          for(const auto & entry : item.second)
            {
              configureTransponderValue(transponderConfigurations,
                                        entry.asString(),
                                        &TransponderConfiguration::setTransmitSlots);
            }
        }
      else if(item.first == "transponder.transmit.antenna")
        {
          for(const auto & entry : item.second)
            {
              configureTransponderValue(transponderConfigurations,
                                        entry.asString(),
                                        &TransponderConfiguration::setTransmitAntennaIndex);
            }
        }
      else if(item.first == "transponder.transmit.enable")
        {
          for(const auto & entry : item.second)
            {
              configureTransponderValue(transponderConfigurations,
                                        entry.asString(),
                                        &TransponderConfiguration::setTransmitEnable);
            }
        }
      else if(item.first == "antenna.defines")
        {
          for(const auto & entry : item.second)
            {
              try
                {
                  configureAntennaValue(antennas,
                                        entry.asString());
                }
              catch(const std::exception & exp)
                {
                  return std::make_pair(std::string{exp.what()},false);
                }
            }
        }

    }

  // after initial configure() a bentpipe model instance may not add
  // or remove transponders or antennas. Once initially configured the
  // model antenna and transponder counts (and indexes) are considered
  // finalized -- no antennas or transponders may be removed or added.
  if(bFinal_)
    {
      for(const auto & entry : transponderConfigurations)
        {
          if(!transponderIndexSet_.count(entry.first))
            {
              std::string sTransponderId{"transponder " + std::to_string(entry.first)};
              return std::make_pair(sTransponderId +
                                    " must be specified at the onset of configuration",
                                    false);
            }
        }

      for(const auto & entry : antennas)
        {
          if(!antennaIndexSet_.count(entry.first))
            {
              std::string sAntennaId{"antenna " + std::to_string(entry.first)};

              return std::make_pair(sAntennaId +
                                    " must be specified at the onset of configuration",
                                    false);
            }
        }
    }


  using RxAntennaFreqSet = std::set<std::pair<AntennaIndex,std::uint64_t>>;
  RxAntennaFreqSet rxAntennaFreqSet{};

  for(const auto & entry : transponderConfigurations)
    {
      if(!bFinal_)
        {
          transponderIndexSet_.emplace(entry.first);
        }

      const auto & transponder  = entry.second;

      std::string sTransponderId{"transponder " + std::to_string(entry.first)};

      if(!transponder.getReceiveFrequencyHz())
        {
          return std::make_pair(sTransponderId + " transponder.receive.frequency not set or 0",
                                false);
        }

      if(!transponder.getReceiveBandwidthHz())
        {
          return std::make_pair(sTransponderId + " transponder.receive.bandwidth not set or 0",
                                false);
        }

      if(transponder.getReceiveAction() == ReceiveAction::UNKNOWN)
        {
          return std::make_pair(sTransponderId + " transponder.receive.action not set",
                                false);
        }

      if(!transponder.getTransmitFrequencyHz())
        {
          return std::make_pair(sTransponderId + " transponder.transmit.frequency not set or 0",
                                false);
        }

      if(!transponder.getTransmitBandwidthHz())
        {
          return std::make_pair(sTransponderId + " transponder.transmit.bandwidth not set or 0",
                                false);
        }

      if(!transponder.getTransmitDataRatebps())
        {
          return std::make_pair(sTransponderId + " transponder.transmit.datarate not set or 0",
                                false);
        }

      if(transponder.getTransmitSlotsPerFrame() &&
         (transponder.getTransmitSlotSize() == Microseconds::zero() ||
          transponder.getTransmitSlots().empty()))
        {
          return std::make_pair(sTransponderId + " transponder.transmit.slotperframe set but"
                                " transponder.transmit.slotsize and/or"
                                " transponder.transmit.txslots not set",
                                false);
        }

      if(transponder.getTransmitSlotSize() != Microseconds::zero() &&
         (!transponder.getTransmitSlotsPerFrame() ||
          transponder.getTransmitSlots().empty()))
        {
          return std::make_pair(sTransponderId + " transponder.transmit.slotsize set but"
                                " transponder.transmit.slotperframe and/or"
                                " transponder.transmit.txslots not set",
                                false);
        }

      if(!transponder.getTransmitSlots().empty() &&
         (!transponder.getTransmitSlotsPerFrame() ||
          transponder.getTransmitSlotSize() == Microseconds::zero()))
        {
          return std::make_pair(sTransponderId + " transponder.transmit.txslots set but"
                                " transponder.transmit.slotperframe and/or"
                                " transponder.transmit.slotsize not set",
                                false);
        }

      // previous tests established what is needed to use transmit
      // slot, prior conditional would fail if not all required config
      // is present
      if(transponder.getTransmitSlots().empty() &&
         !transponder.getTransmitSlotsPerFrame() &&
         transponder.getTransmitSlotSize() == Microseconds::zero() &&
         !transponder.getTransmitMTUBytes())
        {
          return std::make_pair(sTransponderId + " transponder.transmit.mtu not set or 0"
                                " but transponder.transmit.txslots,"
                                " transponder.transmit.slotperframe, and"
                                " transponder.transmit.slotsize not set",
                                false);
        }

      if(!pcrCurveIndexSet_.count(transponder.getPCRCurveIndex()))
        {
          return std::make_pair(sTransponderId + " transponder.transmit.pcrcurveindex set to"
                                " unknown index",
                                false);
        }

      if(!antennas.count(transponder.getReceiveAntennaIndex()))
        {
          return std::make_pair(sTransponderId + " transponder.receive.antenna set to"
                                " unknown antenna",
                                false);
        }

      if(!antennas.count(transponder.getTransmitAntennaIndex()))
        {
          return std::make_pair(sTransponderId + " transponder.transmit.antenna set to"
                                " unknown antenna",
                                false);
        }

      if(!rxAntennaFreqSet.emplace(std::make_pair(transponder.getReceiveAntennaIndex(),
                                                  transponder.getReceiveFrequencyHz())).second)
        {
          return std::make_pair(sTransponderId + " transponder.receive.antenna and"
                                " transponder.receive.frequency already in use",
                                false);
        }
    }

  if(!bFinal_)
    {
      for(const auto & entry: antennas)
        {
          antennaIndexSet_.emplace(entry.first);
        }
    }

  bFinal_ = true;

  return std::make_pair(std::string{},true);
}
