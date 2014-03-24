/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include <functional>
inline 
EMANE::Events::Location::Location(NEMId id,
                                  const Position & position,
                                  std::pair<const Orientation &, bool> orientation,
                                  std::pair<const Velocity &, bool> velocity):
  id_{id},
  position_{position},
  orientation_{orientation.first},
  velocity_{velocity.first},
  bHasOrientation_{orientation.second},
  bHasVelocity_{velocity.second}{}

inline
EMANE::NEMId EMANE::Events::Location::getNEMId() const
{
  return id_;
}

inline
const EMANE::Position & EMANE::Events::Location::getPosition() const
{
  return position_;
}

inline
std::pair<const EMANE::Orientation &, bool>
EMANE::Events::Location::getOrientation() const
{
  return std::make_pair(std::cref(orientation_),bHasOrientation_);
}

inline
std::pair<const EMANE::Velocity &, bool>
EMANE::Events::Location::getVelocity() const
{
  return std::make_pair(std::cref(velocity_),bHasVelocity_);
}
