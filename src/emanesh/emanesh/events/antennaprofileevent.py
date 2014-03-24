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
    import sys
    from optparse import OptionParser
    from emanesh.events import EventService

    usage = """emaneevent-antennaprofile [OPTION]... NEMID[:NEMID] \\
         PROFILE ANTENNAAZIMUTH ANTENNAELEVATION

       PROFILE := 'profile'=ID

       ANTENNAAZIMUTH := 'azimuth'=DEGREES

       ANTENNAELEVATION := 'elevation'=DEGREES

       DEGREES := degrees (float)

       ID := Profile Id (uint)
"""
    description="Publish an antenna profile event containing NEMs in a \
specified range. Unless one or more targets are specified the antenna \
profile event will be sent to the 'all NEMs' event address. This is \
always what you want to do since all NEMs must be aware of all other \
NEM antenna profiles."

    epilog="""
The NEM range specification creates a list of NEMs that will have the
specified antenna profile information.

For example

  emaneevent-antennaprofile 1:10 profile=1 azimuth=45.0 elevation=0.0

Will create a single antenna profile event that contains 10 entries
for NEM 1 to 10, all with the same profile and pointing information.

You can use the '--target' option and the '--reference' to specifically
target one or more NEMs in the range with one of more of the
location values. The NEMs specified with '--target' do not have to be
in the range.

For example

  emaneevent-antennaprofile 1:10 profile=1 azimuth=45.0 elevation=0.0 \\
    -t 3 -t 4 -r 8 -r 9 -r 10

will send antenna profile events to NEM 3 and 4 containing the profile
information for NEMs 8, 9 and 10.

  emaneevent-antennaprofile 13 profile=1 azimuth=45.0 elevation=0.0 \\
     -t 7 -t 8 

will send an antenna profile event to NEM 7 and 8 with the profile \\
information for NEM 13.

  emaneevent-antennaprofile 15 -t 0 profile=1 azimuth=45.0 \\
    elevation=0.0

will send an antenna profile event to all NEMs with the profile information
for NEM 15.

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

        if param == 'azimuth' or \
           param == 'elevation':
            try:
                kwargs[param] = float(value)
            except:
                print >>sys.stderr,"invalid %s format:"% param,value
                exit(1) 
        elif  param == 'profile':
           try:
               kwargs[param] = int(value)
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
        event = AntennaProfileEvent()
        for j in nems:
            if j in references:
                try:
                    event.append(j,**kwargs);
                except Exception, exp:
                    print  >>sys.stderr, "error:",exp
                    exit(1)
                    

        service.publish(i,event)
