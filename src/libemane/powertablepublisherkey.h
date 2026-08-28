/*
 * Copyright (c) 2026 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANEPOWERTABLEPUBLISHERKEY_HEADER_
#define EMANEPOWERTABLEPUBLISHERKEY_HEADER_

#include "emane/types.h"
#include "emane/utils/hash.h"

#include <tuple>

namespace EMANE
{
  struct PowerTablePublisherKey
  {
    using Key =
      std::tuple<NEMId,
                 AntennaIndex,
                 AntennaIndex,
                 std::uint64_t>;

    PowerTablePublisherKey(NEMId nemId,
                           AntennaIndex rxAntennaId,
                           AntennaIndex txAntennaId,
                           std::uint64_t u64FrequencyHz):
      key_{std::make_tuple(nemId,
                           rxAntennaId,
                           txAntennaId,
                           u64FrequencyHz)}{}

    PowerTablePublisherKey() = default;

    PowerTablePublisherKey(PowerTablePublisherKey const &) = default;

    bool operator==(const PowerTablePublisherKey & other) const
    {
      return key_ == other.key_;
    }

    bool operator<(const PowerTablePublisherKey & other) const
    {
      return key_ < other.key_;
    }

    const Key key_;
  };
}

namespace std
{
  template<>
  struct hash<EMANE::PowerTablePublisherKey>
  {
    std::size_t operator()(const EMANE::PowerTablePublisherKey & key) const
    {
      std::size_t seed{};
      EMANE::Utils::hashCombine(seed,
                                EMANE::Utils::hashCompute(std::get<0>(key.key_)));
      EMANE::Utils::hashCombine(seed,
                                EMANE::Utils::hashCompute(std::get<1>(key.key_)));
      EMANE::Utils::hashCombine(seed,
                                EMANE::Utils::hashCompute(std::get<2>(key.key_)));
      EMANE::Utils::hashCombine(seed,
                                EMANE::Utils::hashCompute(std::get<3>(key.key_)));
      return seed;
    }
  };
}

#endif // EMANEPOWERTABLEPUBLISHERKEY_HEADER_
