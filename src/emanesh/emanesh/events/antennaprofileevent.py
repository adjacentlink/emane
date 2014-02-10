#
# Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
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
import antennaprofileevent_pb2

class AntennaProfileEvent(Event):
    IDENTIFIER = 102

    def __init__(self):
        self._event = antennaprofileevent_pb2.AntennaProfileEvent()

    def append(self,nemId,**kwargs):
        hasProfile = False
        hasAzimuth = False
        hasElevation = False
              
        profile = self._event.profiles.add()

        profile.nemId = nemId
        
        for (name,value) in kwargs.items():

            if name == 'profile':
                if isinstance(value,int):
                    hasProfile = True
                    profile.profileId = value
                else:
                    raise ValueError("profile must be an integer")

            elif name == 'azimuth':
                if isinstance(value,int) or \
                        isinstance(value,float):
                    hasAzimuth = True
                    profile.antennaAzimuthDegrees = value
                else:
                    raise ValueError("azimuth must be numeric")

            elif name == 'elevation':
                if isinstance(value,int) or \
                        isinstance(value,float):
                    hasElevation = True
                    profile.antennaElevationDegrees = value
                else:
                    raise ValueError("elevation must be numeric")

            else:
                raise KeyError("unknown parameter: %s" % name)

        if not (hasProfile and hasAzimuth and hasElevation):
            raise KeyError("must specify profile, azimuth and elevation")


    def serialize(self):
        return self._event.SerializeToString()

    def restore(self,data):
        self._event.ParseFromString(data)
        
    def __iter__(self):
        for profile in self._event.profiles:
            kwargs = {'profile': profile.profileId,
                      'azimuth' : profile.antennaAzimuthDegrees,
                      'elevation' : profile.antennaElevationDegrees}

            yield (profile.nemId,kwargs)

                
if __name__ == "__main__":
    e = AntennaProfileEvent()
    e.append(1,profile=10,azimuth=11,elevation=13)

    for i in e:
        print i
