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

#include "transponderconfigurationupdate.h"

EMANE::Models::BentPipe::TransponderConfigurationUpdate::TransponderConfigurationUpdate(TransponderIndex transponderIndex):
  transponderIndex_{transponderIndex},
  u64ReceiveFrequencyHz_{},
  curveIndex_{},
  u64TransmitFrequencyHz_{},
  u64TransmitDataRatebps_{},
  dTransmitPowerdBm_{},
  transmitUbendDelay_{},
  u16TransmitSlotsPerFrame_{},
  transmitSlotSize_{},
  transmitSlots_{},
  u64TransmitMTUBytes_{},
  bReceiveEnable_{},
  bTransmitEnable_{}{}

void EMANE::Models::BentPipe::TransponderConfigurationUpdate::setReceiveFrequencyHz(std::uint64_t u64FrequencyHz)
{
  u64ReceiveFrequencyHz_ = u64FrequencyHz;
}

void EMANE::Models::BentPipe::TransponderConfigurationUpdate::setReceiveEnable(bool bEnable)
{
  bReceiveEnable_ = bEnable;
}

void EMANE::Models::BentPipe::TransponderConfigurationUpdate::setPCRCurveIndex(PCRCurveIndex index)
{
  curveIndex_ = index;
}

void EMANE::Models::BentPipe::TransponderConfigurationUpdate::setTransmitFrequencyHz(std::uint64_t u64FrequencyHz)
{
  u64TransmitFrequencyHz_ = u64FrequencyHz ;
}

void EMANE::Models::BentPipe::TransponderConfigurationUpdate::setTransmitDataRatebps(std::uint64_t u64DataRatebps)
{
  u64TransmitDataRatebps_ = u64DataRatebps;
}

void EMANE::Models::BentPipe::TransponderConfigurationUpdate::setTransmitMTUBytes(std::uint64_t u64MTUBytes)
{
  u64TransmitMTUBytes_ = u64MTUBytes;
}

void EMANE::Models::BentPipe::TransponderConfigurationUpdate::setTransmitPowerdBm(double dPowerdBm)
{
  dTransmitPowerdBm_ = dPowerdBm;
}

void EMANE::Models::BentPipe::TransponderConfigurationUpdate::setTransmitUbendDelay(const Microseconds & delay)
{
  transmitUbendDelay_ = delay;
}

void EMANE::Models::BentPipe::TransponderConfigurationUpdate::setTransmitSlotsPerFrame(std::uint16_t u16TransmitSlotsPerFrame)
{
  u16TransmitSlotsPerFrame_ = u16TransmitSlotsPerFrame;
}

void EMANE::Models::BentPipe::TransponderConfigurationUpdate::setTransmitSlotSize(const Microseconds & slotSize)
{
  transmitSlotSize_ = slotSize;
}

void EMANE::Models::BentPipe::TransponderConfigurationUpdate::setTransmitSlots(const TransmitSlots & slots)
{
  transmitSlots_ = slots;
}

void EMANE::Models::BentPipe::TransponderConfigurationUpdate::setTransmitEnable(bool bEnable)
{
  bTransmitEnable_ = bEnable;
}

EMANE::Models::BentPipe::TransponderIndex
EMANE::Models::BentPipe::TransponderConfigurationUpdate::getTransponderIndex() const
{
  return transponderIndex_;
}

const std::optional<std::uint64_t> &
EMANE::Models::BentPipe::TransponderConfigurationUpdate::getReceiveFrequencyHz() const
{
  return u64ReceiveFrequencyHz_;
}

const std::optional<bool> &
EMANE::Models::BentPipe::TransponderConfigurationUpdate::getReceiveEnable() const
{
  return bReceiveEnable_;
}

const std::optional<EMANE::Models::BentPipe::PCRCurveIndex> &
EMANE::Models::BentPipe::TransponderConfigurationUpdate::getPCRCurveIndex() const
{
  return curveIndex_;
}

const std::optional<std::uint64_t> &
EMANE::Models::BentPipe::TransponderConfigurationUpdate::getTransmitFrequencyHz() const
{
  return u64TransmitFrequencyHz_;
}

const std::optional<std::uint64_t> &
EMANE::Models::BentPipe::TransponderConfigurationUpdate::getTransmitDataRatebps() const
{
  return u64TransmitDataRatebps_;
}

const std::optional<std::uint64_t> &
EMANE::Models::BentPipe::TransponderConfigurationUpdate::getTransmitMTUBytes() const
{
  return u64TransmitMTUBytes_;
}

const std::optional<double> &
EMANE::Models::BentPipe::TransponderConfigurationUpdate::getTransmitPowerdBm() const
{
  return dTransmitPowerdBm_;
}


const std::optional<EMANE::Microseconds> &
EMANE::Models::BentPipe::TransponderConfigurationUpdate::getTransmitUbendDelay() const
{
  return transmitUbendDelay_;
}

const std::optional<std::uint16_t> &
EMANE::Models::BentPipe::TransponderConfigurationUpdate::getTransmitSlotsPerFrame() const
{
  return u16TransmitSlotsPerFrame_;
}

const std::optional<EMANE::Microseconds> &
EMANE::Models::BentPipe::TransponderConfigurationUpdate::getTransmitSlotSize() const
{
  return transmitSlotSize_;
}

const std::optional<EMANE::Models::BentPipe::TransmitSlots> &
EMANE::Models::BentPipe::TransponderConfigurationUpdate::getTransmitSlots() const
{
  return transmitSlots_;
}

const std::optional<bool> &
EMANE::Models::BentPipe::TransponderConfigurationUpdate::getTransmitEnable() const
{
  return bTransmitEnable_;
}
