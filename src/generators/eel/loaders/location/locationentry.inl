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

inline
EMANE::Generators::EEL::LocationEntry::LocationEntry():
  position_{},
  orientation_{},
  velocity_{},
  bVelocity_{},
  bOrientation_{}{}

inline
void EMANE::Generators::EEL::LocationEntry::setPosition(const Position & position)
{
  position_ = position;
}
      
inline
void EMANE::Generators::EEL::LocationEntry::setOrientation(const Orientation & orientation)
{
  orientation_ = orientation;
  bOrientation_ = true;
}

inline
void EMANE::Generators::EEL::LocationEntry::setVelocity(const Velocity & velocity)
{
  velocity_ = velocity;
  bVelocity_ = true;
}

inline
const EMANE::Position EMANE::Generators::EEL::LocationEntry::getPosition() const
{
  return position_;
}

inline
std::pair<const EMANE::Orientation &,bool> EMANE::Generators::EEL::LocationEntry::getOrientation() const
{
  return std::make_pair(std::ref(orientation_),bOrientation_);
}

inline
std::pair<const EMANE::Velocity &,bool> EMANE::Generators::EEL::LocationEntry::getVelocity() const
{
  return std::make_pair(std::ref(velocity_),bVelocity_);
}
