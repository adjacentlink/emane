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

#include "emane/controls/antennaprofilecontrolmessage.h"

class EMANE::Controls::AntennaProfileControlMessage::Implementation
{
public:
  Implementation(AntennaProfileId id,
                 double dAntennaAzimuthDegrees,
                 double dAntennaElevationDegrees):
    id_{id},
    dAntennaAzimuthDegrees_{dAntennaAzimuthDegrees},
    dAntennaElevationDegrees_{dAntennaElevationDegrees}
  {}

  AntennaProfileId getAntennaProfileId() const
  {
    return id_;
  }

  
  double getAntennaAzimuthDegrees() const
  {
    return dAntennaAzimuthDegrees_;
  }

    
  double getAntennaElevationDegrees() const
  {
    return dAntennaElevationDegrees_;
  }
  
private:
  const AntennaProfileId id_;
  const double dAntennaAzimuthDegrees_;
  const double dAntennaElevationDegrees_;
};

EMANE::Controls::AntennaProfileControlMessage::AntennaProfileControlMessage(AntennaProfileId id,
                                                                  double dAntennaAzimuthDegrees,
                                                                  double dAntennaElevationDegrees):
  ControlMessage{IDENTIFIER},
  pImpl_{new Implementation{id,dAntennaAzimuthDegrees,dAntennaElevationDegrees}}
{}

EMANE::Controls::AntennaProfileControlMessage::~AntennaProfileControlMessage(){}

EMANE::AntennaProfileId EMANE::Controls::AntennaProfileControlMessage::getAntennaProfileId() const
{
  return pImpl_->getAntennaProfileId();
}

  
double EMANE::Controls::AntennaProfileControlMessage::getAntennaAzimuthDegrees() const
{
  return pImpl_->getAntennaAzimuthDegrees();
}


double EMANE::Controls::AntennaProfileControlMessage::getAntennaElevationDegrees() const
{
  return pImpl_->getAntennaElevationDegrees();
}


EMANE::Controls::AntennaProfileControlMessage *
EMANE::Controls::AntennaProfileControlMessage::create(AntennaProfileId id,
                                            double dAntennaAzimuthDegrees,
                                            double dAntennaElevationDegrees)
{
  return new AntennaProfileControlMessage{id,dAntennaAzimuthDegrees,dAntennaElevationDegrees};
}
