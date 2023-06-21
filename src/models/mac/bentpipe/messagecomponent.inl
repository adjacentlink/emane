/*
 * Copyright (c) 2015,2023 - Adjacent Link LLC, Bridgewater, New Jersey
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

inline
EMANE::Models::BentPipe::MessageComponent::MessageComponent(NEMId destination,
                                                            const Utils::VectorIO & vectorIO):
  destination_{destination},
  fragmentIndex_{},
  fragmentOffset_{},
  bMoreFragments_{},
  u64FragmentSequence_{}
{
  for(const auto & entry : vectorIO)
    {
      data_.insert(data_.end(),
                   reinterpret_cast<std::uint8_t*>(entry.iov_base),
                   reinterpret_cast<std::uint8_t*>(entry.iov_base) + entry.iov_len);
    }
}

inline
EMANE::Models::BentPipe::MessageComponent::MessageComponent(NEMId destination,
                                                            const Utils::VectorIO & vectorIO,
                                                            size_t fragmentIndex,
                                                            size_t fragmentOffset,
                                                            std::uint64_t u64FragmentSequence,
                                                            bool bMore):
  destination_{destination},
  fragmentIndex_{fragmentIndex},
  fragmentOffset_{fragmentOffset},
  bMoreFragments_{bMore},
  u64FragmentSequence_{u64FragmentSequence}
{
  for(const auto & entry : vectorIO)
    {
      data_.insert(data_.end(),
                   reinterpret_cast<std::uint8_t*>(entry.iov_base),
                   reinterpret_cast<std::uint8_t*>(entry.iov_base) + entry.iov_len);
    }
}

inline
const EMANE::Models::BentPipe::MessageComponent::Data &
EMANE::Models::BentPipe::MessageComponent::getData() const
{
  return data_;
}

inline
EMANE::NEMId EMANE::Models::BentPipe::MessageComponent::getDestination() const
{
  return destination_;
}

inline
bool EMANE::Models::BentPipe::MessageComponent::isFragment() const
{
  return fragmentOffset_ || bMoreFragments_;
}

inline
size_t EMANE::Models::BentPipe::MessageComponent::getFragmentIndex() const
{
  return fragmentIndex_;
}

inline
size_t EMANE::Models::BentPipe::MessageComponent::getFragmentOffset() const
{
  return fragmentOffset_;
}

inline
std::uint64_t EMANE::Models::BentPipe::MessageComponent::getFragmentSequence() const
{
  return u64FragmentSequence_;
}

inline
bool EMANE::Models::BentPipe::MessageComponent::isMoreFragments() const
{
  return bMoreFragments_;
}
