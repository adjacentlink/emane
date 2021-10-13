/*
 * Copyright (c) 2013-2014,2020 - Adjacent Link LLC, Bridgewater,
 * New Jersey
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

#ifndef EMANETYPES_HEADER_
#define EMANETYPES_HEADER_

#include <cstdint>
#include <chrono>
#include <string>
#include <list>
#include <set>
#include <vector>

namespace EMANE
{
  using Seconds = std::chrono::seconds;
  using Milliseconds = std::chrono::milliseconds;
  using Microseconds = std::chrono::microseconds;
  using Nanoseconds = std::chrono::nanoseconds;
  using DoubleSeconds = std::chrono::duration<double>;
  using Clock = std::chrono::system_clock;
  using Duration = Clock::duration;
  using TimePoint = Clock::time_point;

  using NEMId = std::uint16_t;
  using EventId = std::uint16_t;
  using TimerEventId = std::size_t;

  using PlatformId = std::uint16_t;
  using AntennaProfileId = std::uint16_t;
  using ControlMessageId = std::uint16_t;
  using RegistrationId = std::uint16_t;
  using BuildId = std::uint32_t;

  using LengthPrefix = std::uint16_t;

  using Priority = std::uint8_t;

  using Strings = std::list<std::string>;

  using AntennaIndex = std::uint16_t;
  using FilterIndex = std::uint16_t;
  using FilterData = std::string;

  using FrequencyGroupIndex = std::uint16_t;

  using FrequencySet = std::set<std::uint64_t>;
  using FrequencySets = std::vector<FrequencySet>;

  constexpr AntennaIndex DEFAULT_ANTENNA_INDEX{0};

  // All 1's NEMId represents a broad cast packet
  constexpr NEMId NEM_BROADCAST_MAC_ADDRESS{std::numeric_limits<NEMId>::max()};
}


#endif // EMANETYPES_HEADER_
