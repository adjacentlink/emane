#
# Copyright (c) 2013-2014,2017 - Adjacent Link LLC, Bridgewater,
# New Jersey
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in
#   the documentation and/or other materials provided with the
#   distribution.
# * Neither the name of Adjacent Link LLC nor the names of its
#   contributors may be used to endorse or promote products derived
#   from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

from . import Event
from . import locationevent_pb2

class LocationEvent(Event):
    IDENTIFIER = 100

    def __init__(self):
        self._event = locationevent_pb2.LocationEvent()

    def append(self,nemId,**kwargs):
        hasLatitude = False
        hasLongitude = False
        hasAltitude = False

        hasAzimuth = False
        hasElevation = False
        hasMagnitude = False

        hasRoll = False
        hasPitch = False
        hasYaw = False

        location = self._event.locations.add()

        location.nemId = nemId


        for (name,value) in list(kwargs.items()):

            if name == 'latitude':
                if isinstance(value,int) or \
                        isinstance(value,float):
                    hasLatitude = True
                    location.position.latitudeDegrees = value
                else:
                    raise ValueError("latitude must be numeric")

            elif name == 'longitude':
                if isinstance(value,int) or \
                        isinstance(value,float):
                    hasLongitude = True
                    location.position.longitudeDegrees = value
                else:
                    raise ValueError("longitude must be numeric")

            elif name == 'altitude':
                if isinstance(value,int) or \
                        isinstance(value,float):
                    hasAltitude = True
                    location.position.altitudeMeters = value
                else:
                    raise ValueError("altitude must be numeric")

            elif name == 'azimuth':
                if isinstance(value,int) or \
                        isinstance(value,float):
                    hasAzimuth = True
                    location.velocity.azimuthDegrees = value
                else:
                    raise ValueError("azimuth must be numeric")

            elif name == 'elevation':
                if isinstance(value,int) or \
                        isinstance(value,float):
                    hasElevation = True
                    location.velocity.elevationDegrees = value
                else:
                    raise ValueError("elevation must be numeric")

            elif name == 'magnitude':
                if isinstance(value,int) or \
                        isinstance(value,float):
                    hasMagnitude = True
                    location.velocity.magnitudeMetersPerSecond = value
                else:
                    raise ValueError("magnitude must be numeric")

            elif name == 'roll':
                if isinstance(value,int) or \
                        isinstance(value,float):
                    hasRoll = True
                    location.orientation.rollDegrees = value
                else:
                    raise ValueError("roll must be numeric")

            elif name == 'pitch':
                if isinstance(value,int) or \
                        isinstance(value,float):
                    hasPitch = True
                    location.orientation.pitchDegrees = value
                else:
                    raise ValueError("pitch must be numeric")

            elif name == 'yaw':
                if isinstance(value,int) or \
                        isinstance(value,float):
                    hasYaw = True
                    location.orientation.yawDegrees = value
                else:
                    raise ValueError("yaw must be numeric")

            else:
                raise KeyError("unknown parameter: %s" % name)

        if not (hasLatitude and hasLongitude and hasAltitude):
            raise KeyError("must specify latitude, longitude and altitude")

        if (hasAzimuth and not (hasElevation and hasMagnitude)) or \
                (hasElevation and not (hasAzimuth and hasMagnitude)) or \
                (hasMagnitude and not (hasAzimuth and hasElevation)):
            raise KeyError("must specify azimuth, elevation and magnitude when specifing velocity")

        if (hasRoll and not (hasPitch and hasYaw)) or \
                (hasPitch and not (hasRoll and hasYaw)) or \
                (hasYaw and not (hasRoll and hasPitch)):
            raise KeyError("must specify roll, pitch and yaw when specifing orientation")

    def serialize(self):
        return self._event.SerializeToString()

    def restore(self,data):
        self._event.ParseFromString(data)

    def __iter__(self):
        for location in self._event.locations:
            kwargs = {'latitude':location.position.latitudeDegrees,
                      'longitude' : location.position.longitudeDegrees,
                      'altitude' : location.position.altitudeMeters}

            if location.HasField('velocity'):
                kwargs['azimuth'] = location.velocity.azimuthDegrees
                kwargs['elevation'] = location.velocity.elevationDegrees
                kwargs['magnitude'] = location.velocity.magnitudeMetersPerSecond

            if location.HasField('orientation'):
                kwargs['roll'] = location.orientation.rollDegrees
                kwargs['pitch'] = location.orientation.pitchDegrees
                kwargs['yaw'] = location.orientation.yawDegrees


            yield (location.nemId,kwargs)
