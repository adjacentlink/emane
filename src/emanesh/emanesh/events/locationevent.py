#
# Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
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
import locationevent_pb2

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

        
        for (name,value) in kwargs.items():

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

                
if __name__ == "__main__":
    import sys
    from optparse import OptionParser
    from emanesh.events import EventService

    usage = """emaneevent-location [OPTION]... NEMID[:NEMID] LATITUDE \\
         LONGITUDE ALTITUDE [VELOCITY] [ORIENTATION]

       LATITUDE := 'latitude'=DEGREES

       LONGITUDE := 'longitude'=DEGREES

       ALTITUDE := 'altitude'=MPS

       VELOCITY := 'azimuth'=DEGREES 'elevation'=DEGREES 'magnitude'=MPS    

       ORIENTATION := 'pitch'=DEGREES 'roll'=DEGREES  'yaw'=DEGREES

       DEGREES := degrees (float)

       MPS := Meters/Second (float)
"""
    description="Publish a location event containing NEMs in a specified \
range. Unless one or more targets are specified the location event will \
be sent to the 'all NEMs' event address.  This is always what you want \
to do since all NEMs must be aware of all other NEM locations."

    epilog="""
The NEM range specification creates a list of NEMs that will have the
specified location information.

For example

  emaneevent-location 1:10 latitude=40.031075 longitude=-74.523518 \\
   altitude=3.000000

Will create a single location event that contains 10 entries for NEM
1 to 10, all with the same location: 40.031075 -74.52351 3.0

Velocity and orientation are optional. When missing NEMs will you any
previously received values.

You can use the '--target' option and the '--reference' to specifically
target one or more NEMs in the range with one of more of the
location values. The NEMs specified with '--target' do not have to be
in the range.

For example

  emaneevent-location 1:10 latitude=40.031075 longitude=-74.523518 \\
    altitude=3.000000 -t 3 -t 4 -r 8 -r 9 -r 10

will send location events to NEM 3 and 4 containing the location information
for NEMs 8, 9 and 10.

  emaneevent-location 13 latitude=40.031075 longitude=-74.523518 \\
    altitude=3.000000 -t 7 -t 8 

will send a location event to NEM 7 and 8 with the location of NEM 13.

  emaneevent-location 15 -t 0 latitude=40.031075 longitude=-74.523518 \\
    altitude=3.000000 

will send a location event to all NEMs with the location of NEM 15.

"""

    class LocalParser(OptionParser):
        def format_epilog(self, formatter):
            return self.epilog

    optionParser = LocalParser(usage=usage,
                               description=description,
                               epilog=epilog)

    optionParser.add_option("-p", 
                            "--port",
                            action="store",
                            type="int",
                            dest="port",
                            default=45703,
                            help="Event channel listen port [default: %default]")

    optionParser.add_option("-g", 
                            "--group",
                            action="store",
                            type="string",
                            dest="group",
                            default="224.1.2.8",
                            help="Event channel multicast group [default: %default]")

    optionParser.add_option("-i",
                            "--device",
                            action="store",
                            type="string",
                            dest="device",
                            help="Event channel multicast device")

    optionParser.add_option("-t",
                            "--target",
                            action="append",
                            type="int",
                            dest="target",
                            help="Only send an event to the target")

    optionParser.add_option("-r",
                            "--reference",
                            action="append",
                            type="int",
                            dest="reference",
                            help="Send events to NEMs in the range but only include information for the reference NEM.")


    (options, args) = optionParser.parse_args()

    service = EventService((options.group,options.port,options.device))

    if len(args) < 2:
        print >>sys.stderr,"missing arguments"
        exit(1)

    nems = args[0].split(':')

    if len(nems) == 0 or len(nems) > 2:
        print >>sys.stderr,"invalid NEMID format:",args[0]
        exit(1)

    try:
        nems = [int(x) for x in nems]
    except:
        print >>sys.stderr,"invalid NEMID format:",args[0]
        exit(1)

    if len(nems) > 1:
        nems = range(nems[0],nems[1]+1)

    if not nems:
        print >>sys.stderr,"invalid target format:",args[0]
        exit(1)

    kwargs  = {}

    for component in args[1:]:
        try:
            (param,value) = component.split("=")
        except:
           print >>sys.stderr,"invalid component format:",component
           exit(1)  

        if param == 'latitude' or \
           param == 'longitude' or \
           param == 'altitude' or \
           param == 'azimuth' or \
           param == 'elevation' or \
           param == 'magnitude' or \
           param == 'pitch' or \
           param == 'roll' or \
           param == 'yaw':
            try:
                kwargs[param] = float(value)
            except:
                print >>sys.stderr,"invalid %s format:"% param,value
                exit(1) 
        else:
            print >>sys.stderr,"invalid location component:",param
            exit(1)  

    if nems[0] == 0:
        print >>sys.stderr,"0 is not a valid NEMID"
        exit(1)

    if options.target:
        targets = options.target
    else:
        targets = [0]

    if options.reference:
        references = options.reference
    else:
        references = nems

    for i in targets:
        event = LocationEvent()
        for j in nems:
            if j in references:
                try:
                    event.append(j,**kwargs);
                except Exception, exp:
                    print  >>sys.stderr, "error:",exp
                    exit(1)
                    

        service.publish(i,event)

