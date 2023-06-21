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

#include "transponderconfiguration.h"

EMANE::Models::BentPipe::TransponderConfiguration::TransponderConfiguration(TransponderIndex transponderIndex):
  transponderIndex_{transponderIndex},
  u64ReceiveFrequencyHz_{},
  u64ReceiveBandwidthHz_{},
  receiveAntennaIndex_{},
  curveIndex_{},
  receiveAction_{ReceiveAction::UNKNOWN},
  u64TransmitFrequencyHz_{},
  u64TransmitBandwidthHz_{},
  u64TransmitDataRatebps_{},
  transmitAntennaIndex_{},
  dTransmitPowerdBm_{},
  transmitUbendDelay_{},
  transmitProcessTOS_{},
  u16TransmitSlotsPerFrame_{},
  transmitSlotSize_{},
  transmitSlots_{},
  u64TransmitMTUBytes_{},
  bReceiveEnable_{},
  bTransmitEnable_{}{}

void EMANE::Models::BentPipe::TransponderConfiguration::setReceiveFrequencyHz(std::uint64_t u64FrequencyHz)
{
  u64ReceiveFrequencyHz_ = u64FrequencyHz;
}

void EMANE::Models::BentPipe::TransponderConfiguration::setReceiveBandwidthHz(std::uint64_t u64BandwidthHz)
{
  u64ReceiveBandwidthHz_ = u64BandwidthHz;
}

void EMANE::Models::BentPipe::TransponderConfiguration::setReceiveAntennaIndex(AntennaIndex antennaIndex)
{
  receiveAntennaIndex_ = antennaIndex;
}
void EMANE::Models::BentPipe::TransponderConfiguration::setPCRCurveIndex(PCRCurveIndex index)
{
  curveIndex_ = index;
}

void EMANE::Models::BentPipe::TransponderConfiguration::setReceiveAction(ReceiveAction action)
{
  receiveAction_ = action;
}

void EMANE::Models::BentPipe::TransponderConfiguration::setReceiveEnable(bool bEnable)
{
  bReceiveEnable_ = bEnable;
}

void EMANE::Models::BentPipe::TransponderConfiguration::setTransmitFrequencyHz(std::uint64_t u64FrequencyHz)
{
  u64TransmitFrequencyHz_ = u64FrequencyHz ;
}

void EMANE::Models::BentPipe::TransponderConfiguration::setTransmitBandwidthHz(std::uint64_t u64BandwidthHz)
{
  u64TransmitBandwidthHz_ = u64BandwidthHz;
}

void EMANE::Models::BentPipe::TransponderConfiguration::setTransmitDataRatebps(std::uint64_t u64DataRatebps)
{
  u64TransmitDataRatebps_ = u64DataRatebps;
}

void EMANE::Models::BentPipe::TransponderConfiguration::setTransmitMTUBytes(std::uint64_t u64MTUBytes)
{
  u64TransmitMTUBytes_ = u64MTUBytes;
}

void EMANE::Models::BentPipe::TransponderConfiguration::setTransmitAntennaIndex(AntennaIndex antennaIndex)
{
  transmitAntennaIndex_ = antennaIndex;
}

void EMANE::Models::BentPipe::TransponderConfiguration::setTransmitPowerdBm(double dPowerdBm)
{
  dTransmitPowerdBm_ = dPowerdBm;
}

void EMANE::Models::BentPipe::TransponderConfiguration::setTransmitUbendDelay(const Microseconds & delay)
{
  transmitUbendDelay_ = delay;
}

void EMANE::Models::BentPipe::TransponderConfiguration::setTransmitProcessTOS(const TOSSet & tosSet)
{
  transmitProcessTOS_ = tosSet;
}

void EMANE::Models::BentPipe::TransponderConfiguration::setTransmitSlotsPerFrame(std::uint16_t u16TransmitSlotsPerFrame)
{
  u16TransmitSlotsPerFrame_ = u16TransmitSlotsPerFrame;
}

void EMANE::Models::BentPipe::TransponderConfiguration::setTransmitSlotSize(const Microseconds & slotSize)
{
  transmitSlotSize_ = slotSize;
}

void EMANE::Models::BentPipe::TransponderConfiguration::setTransmitSlots(const TransmitSlots & slots)
{
  transmitSlots_ = slots;
}

void EMANE::Models::BentPipe::TransponderConfiguration::setTransmitEnable(bool bEnable)
{
  bTransmitEnable_ = bEnable;
}

EMANE::Models::BentPipe::TransponderIndex
EMANE::Models::BentPipe::TransponderConfiguration::getTransponderIndex() const
{
  return transponderIndex_;
}

std::uint64_t EMANE::Models::BentPipe::TransponderConfiguration::getReceiveFrequencyHz() const
{
  return u64ReceiveFrequencyHz_;
}

std::uint64_t EMANE::Models::BentPipe::TransponderConfiguration::getReceiveBandwidthHz() const
{
  return u64ReceiveBandwidthHz_;
}

EMANE::AntennaIndex EMANE::Models::BentPipe::TransponderConfiguration::getReceiveAntennaIndex() const
{
  return receiveAntennaIndex_;
}

EMANE::Models::BentPipe::PCRCurveIndex
EMANE::Models::BentPipe::TransponderConfiguration::getPCRCurveIndex() const
{
  return curveIndex_;
}

EMANE::Models::BentPipe::ReceiveAction
EMANE::Models::BentPipe::TransponderConfiguration::getReceiveAction() const
{
  return receiveAction_;
}

bool EMANE::Models::BentPipe::TransponderConfiguration::getReceiveEnable() const
{
  return bReceiveEnable_;
}

std::uint64_t EMANE::Models::BentPipe::TransponderConfiguration::getTransmitFrequencyHz() const
{
  return u64TransmitFrequencyHz_;
}

std::uint64_t EMANE::Models::BentPipe::TransponderConfiguration::getTransmitBandwidthHz() const
{
  return u64TransmitBandwidthHz_;
}

std::uint64_t EMANE::Models::BentPipe::TransponderConfiguration::getTransmitDataRatebps() const
{
  return u64TransmitDataRatebps_;
}

std::uint64_t EMANE::Models::BentPipe::TransponderConfiguration::getTransmitMTUBytes() const
{
  return u64TransmitMTUBytes_;
}

EMANE::AntennaIndex EMANE::Models::BentPipe::TransponderConfiguration::getTransmitAntennaIndex() const
{
  return transmitAntennaIndex_;
}

double EMANE::Models::BentPipe::TransponderConfiguration::getTransmitPowerdBm() const
{
  return dTransmitPowerdBm_;
}


const EMANE::Microseconds & EMANE::Models::BentPipe::TransponderConfiguration::getTransmitUbendDelay() const
{
  return transmitUbendDelay_;
}

const EMANE::Models::BentPipe::TOSSet &
EMANE::Models::BentPipe::TransponderConfiguration::getTransmitProcessTOS() const
{
  return transmitProcessTOS_;
}

std::uint16_t  EMANE::Models::BentPipe::TransponderConfiguration::getTransmitSlotsPerFrame() const
{
  return u16TransmitSlotsPerFrame_;
}

const EMANE::Microseconds & EMANE::Models::BentPipe::TransponderConfiguration::getTransmitSlotSize() const
{
  return transmitSlotSize_;
}

const EMANE::Models::BentPipe::TransmitSlots &
EMANE::Models::BentPipe::TransponderConfiguration::getTransmitSlots() const
{
  return transmitSlots_;
}

bool EMANE::Models::BentPipe::TransponderConfiguration::getTransmitEnable() const
{
  return bTransmitEnable_;
}
