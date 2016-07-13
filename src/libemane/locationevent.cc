/*
 * Copyright (c) 2013-2014,2016 - Adjacent Link LLC, Bridgewater,
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

#include "emane/events/locationevent.h"
#include "locationevent.pb.h"

class EMANE::Events::LocationEvent::Implementation
{
public:
  Implementation(const Locations & locations):
    locations_{locations}{}

  const Locations & getLocations() const
  {
    return locations_;
  }

private:
  Locations locations_;
};

EMANE::Events::LocationEvent::LocationEvent(const Serialization & serialization):
  Event(IDENTIFIER)
{
  EMANEMessage::LocationEvent msg;

  if(!msg.ParseFromString(serialization))
    {
      throw SerializationException("unable to deserialize : LocationEvent");
    }

  Locations locations;

  for(const auto & location : msg.locations())
    {
      const auto & positionMessage = location.position();
      NEMId nemId{static_cast<EMANE::NEMId>(location.nemid())};

      Position position{positionMessage.latitudedegrees(),
          positionMessage.longitudedegrees(),
          positionMessage.altitudemeters()};

      // optional
      Velocity velocity{};

      // optional
      Orientation orientation{};

      if(location.has_velocity())
        {
          const auto & velocityMessage = location.velocity();

          velocity = Velocity{velocityMessage.azimuthdegrees(),
                              velocityMessage.elevationdegrees(),
                              velocityMessage.magnitudemeterspersecond()};
        }

      if(location.has_orientation())
        {
          const auto & orientationMessage = location.orientation();

          orientation = Orientation{orientationMessage.rolldegrees(),
                                    orientationMessage.pitchdegrees(),
                                    orientationMessage.yawdegrees()};
        }

      locations.push_back({nemId,
            position,
              {orientation,location.has_orientation()},
                {velocity,location.has_velocity()}});
    }

  pImpl_.reset(new Implementation{locations});
}

EMANE::Events::LocationEvent::LocationEvent(const Locations & locations):
  Event{IDENTIFIER},
  pImpl_{new Implementation{locations}}{}

EMANE::Events::LocationEvent::LocationEvent(const LocationEvent & rhs):
  Event{IDENTIFIER},
  pImpl_{new Implementation{rhs.getLocations()}}{}

EMANE::Events::LocationEvent & EMANE::Events::LocationEvent::operator=(const LocationEvent & rhs)
{
  pImpl_.reset(new Implementation{rhs.getLocations()});
  return *this;
}

EMANE::Events::LocationEvent::LocationEvent(LocationEvent && rval):
  Event{IDENTIFIER},
  pImpl_{new Implementation{{}}}
{
  rval.pImpl_.swap(pImpl_);
}

EMANE::Events::LocationEvent & EMANE::Events::LocationEvent::operator=(LocationEvent && rval)
{
  rval.pImpl_.swap(pImpl_);
  return *this;
}

EMANE::Events::LocationEvent::~LocationEvent(){}

const EMANE::Events::Locations & EMANE::Events::LocationEvent::getLocations() const
{
  return pImpl_->getLocations();
}

EMANE::Serialization EMANE::Events::LocationEvent::serialize() const
{
  Serialization serialization;

  EMANEMessage::LocationEvent msg;

  for(auto & location : pImpl_->getLocations())
    {
      auto pLocationMessage = msg.add_locations();

      pLocationMessage->set_nemid(location.getNEMId());

      auto pPositionMessage = pLocationMessage->mutable_position();

      const auto & position = location.getPosition();

      pPositionMessage->set_latitudedegrees(position.getLatitudeDegrees());
      pPositionMessage->set_longitudedegrees(position.getLongitudeDegrees());
      pPositionMessage->set_altitudemeters(position.getAltitudeMeters());

      auto optionalVelocity = location.getVelocity();

      if(optionalVelocity.second)
        {
          auto pVelocityMessage = pLocationMessage->mutable_velocity();

          pVelocityMessage->set_azimuthdegrees(optionalVelocity.first.getAzimuthDegrees());
          pVelocityMessage->set_elevationdegrees(optionalVelocity.first.getElevationDegrees());
          pVelocityMessage->set_magnitudemeterspersecond(optionalVelocity.first.getMagnitudeMetersPerSecond());
        }

      auto optionalOrientation = location.getOrientation();

      if(optionalOrientation.second)
        {
          auto pOrientationMessage = pLocationMessage->mutable_orientation();

          pOrientationMessage->set_yawdegrees(optionalOrientation.first.getYawDegrees());
          pOrientationMessage->set_pitchdegrees(optionalOrientation.first.getPitchDegrees());
          pOrientationMessage->set_rolldegrees(optionalOrientation.first.getRollDegrees());
        }
    }

  if(!msg.SerializeToString(&serialization))
    {
      throw SerializationException("unable to serialize : LocationEvent");
    }

  return serialization;
}
