/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "emane/events/antennaprofileevent.h"
#include "antennaprofileevent.pb.h"

class EMANE::Events::AntennaProfileEvent::Implementation
{
public:
  Implementation(const AntennaProfiles & profiles):
    profiles_{profiles}{}

  const AntennaProfiles & getAntennaProfiles() const
  {
    return profiles_;
  }

private:
  AntennaProfiles profiles_;
};

EMANE::Events::AntennaProfileEvent::AntennaProfileEvent(const Serialization & serialization):
  Event(IDENTIFIER)
{
  EMANEMessage::AntennaProfileEvent msg;

  try
    {
      if(!msg.ParseFromString(serialization))
        {
          throw SerializationException("unable to deserialize : AntennaProfileEvent");
        }
    }
  catch(google::protobuf::FatalException & exp)
    {
      throw SerializationException("unable to deserialize  : AntennaProfileEvent");
    }
  
  using RepeatedPtrFieldAntennaProfile = 
    google::protobuf::RepeatedPtrField<EMANEMessage::AntennaProfileEvent::Profile>;
  
  AntennaProfiles profiles;
  
  for(const auto & repeatedAntennaProfile : RepeatedPtrFieldAntennaProfile(msg.profiles()))
    {
      profiles.push_back({static_cast<EMANE::NEMId>(repeatedAntennaProfile.nemid()),
          static_cast<EMANE::AntennaProfileId>(repeatedAntennaProfile.profileid()),
            repeatedAntennaProfile.antennaazimuthdegrees(),
            repeatedAntennaProfile.antennaelevationdegrees()});
    }

  pImpl_.reset(new Implementation{profiles});
}
    
EMANE::Events::AntennaProfileEvent::AntennaProfileEvent(const AntennaProfiles & profiles):
  Event{IDENTIFIER},
  pImpl_{new Implementation{profiles}}{}
    
EMANE::Events::AntennaProfileEvent::AntennaProfileEvent(const AntennaProfileEvent & rhs):
  Event{IDENTIFIER},
  pImpl_{new Implementation{rhs.getAntennaProfiles()}}{}
   
EMANE::Events::AntennaProfileEvent & EMANE::Events::AntennaProfileEvent::operator=(const AntennaProfileEvent & rhs)
{
  pImpl_.reset(new Implementation{rhs.getAntennaProfiles()});
  return *this;
}
    
EMANE::Events::AntennaProfileEvent::AntennaProfileEvent(AntennaProfileEvent && rval):
  Event{IDENTIFIER},
  pImpl_{new Implementation{{}}}
{
  rval.pImpl_.swap(pImpl_);
}
    
EMANE::Events::AntennaProfileEvent & EMANE::Events::AntennaProfileEvent::operator=(AntennaProfileEvent && rval)
{
  rval.pImpl_.swap(pImpl_);
  return *this;
}

EMANE::Events::AntennaProfileEvent::~AntennaProfileEvent(){}
    
const EMANE::AntennaProfiles & EMANE::Events::AntennaProfileEvent::getAntennaProfiles() const
{
  return pImpl_->getAntennaProfiles();
}

EMANE::Serialization EMANE::Events::AntennaProfileEvent::serialize() const
{
  Serialization serialization;

  EMANEMessage::AntennaProfileEvent msg;

  for(auto & profile : pImpl_->getAntennaProfiles())
    {
      auto pAntennaProfileMessage = msg.add_profiles();

      pAntennaProfileMessage->set_nemid(profile.getNEMId());

      pAntennaProfileMessage->set_profileid(profile.getAntennaProfileId());

      pAntennaProfileMessage->set_antennaazimuthdegrees(profile.getAntennaAzimuthDegrees());
      
      pAntennaProfileMessage->set_antennaelevationdegrees(profile.getAntennaElevationDegrees());
    }

  try
    {
      if(!msg.SerializeToString(&serialization))
        {
          throw SerializationException("unable to serialize : AntennaProfileEvent");
        }
    }
  catch(google::protobuf::FatalException & exp)
    {
      throw SerializationException("unable to serialize : AntennaProfileEvent");
    }

  return serialization;
}
