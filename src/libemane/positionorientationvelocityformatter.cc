/*
 * Copyright (c) 2014 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "positionorientationvelocityformatter.h"
#include "emane/positionformatter.h"
#include "emane/orientationformatter.h"
#include "emane/velocityformatter.h"

EMANE::PositionOrientationVelocityFormatter::
PositionOrientationVelocityFormatter(const PositionOrientationVelocity & pov):
  pov_(pov)
{}
      
EMANE::Strings EMANE::PositionOrientationVelocityFormatter::operator()() const
{
  Strings strings{{"pov:"}};

  if(!pov_)
    {
      strings.push_back("invalid");
    }
  else
    {
      strings.splice(strings.end(),PositionFormatter(pov_.getPosition())());
  
      auto optionalOrientation =  pov_.getOrientation();

      if(optionalOrientation.second)
        {
          strings.splice(strings.end(),OrientationFormatter(optionalOrientation.first)());
        }
      else
        {
          strings.push_back("orientation: none");
        }
      auto optionalAdjustedOrientation =  pov_.getAdjustedOrientation();

      if(optionalAdjustedOrientation.second)
        {
          strings.push_back("adjusted");
          strings.splice(strings.end(),OrientationFormatter(optionalAdjustedOrientation.first)());
        }
      else
        {
          strings.push_back("adjusted orientation: none");
        }

      auto optionalVelocity = pov_.getVelocity();

      if(optionalVelocity.second)
        {
          strings.splice(strings.end(),VelocityFormatter(optionalVelocity.first)());
        }
      else
        {
          strings.push_back("velocity: none");
        }
    }

  return strings;
}
