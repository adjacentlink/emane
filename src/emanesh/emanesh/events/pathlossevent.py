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
import pathlossevent_pb2

class PathlossEvent(Event):
    IDENTIFIER = 101

    def __init__(self):
        self._event = pathlossevent_pb2.PathlossEvent()

    def append(self,nemId,**kwargs):
        hasForward = False
        hasReverse = False

        pathloss = self._event.pathlosses.add()

        pathloss.nemId = nemId
        
        for (name,value) in kwargs.items():

            if name == 'forward':
                if isinstance(value,int) or \
                        isinstance(value,float):
                    hasForward = True
                    pathloss.forwardPathlossdB = value
                else:
                    raise ValueError("forward pathloss must be numeric")

            elif name == 'reverse':
                if isinstance(value,int) or \
                        isinstance(value,float):
                    hasReverse = True
                    pathloss.reversePathlossdB = value
                else:
                    raise ValueError("reverse pathloss must be numeric")

            else:
                raise KeyError("unknown parameter: %s" % name)

        if not hasForward:
            raise KeyError("must specify forward pathloss")
        elif not hasReverse:
            pathloss.reversePathlossdB = pathloss.forwardPathlossdB
            
    def serialize(self):
        return self._event.SerializeToString()

    def restore(self,data):
        self._event.ParseFromString(data)
        
    def __iter__(self):
        for pathloss in self._event.pathlosses:
            kwargs = {'forward': pathloss.forwardPathlossdB,
                      'reverse' : pathloss.reversePathlossdB}

            yield (pathloss.nemId,kwargs)

                
if __name__ == "__main__":
    import sys
    from optparse import OptionParser
    from emanesh.events import EventService

    usage = "emaneevent-pathloss [OPTION]... NEMID:NEMID PATHLOSSDB"

    optionParser = OptionParser(usage=usage)

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

    optionParser.add_option("--target",
                            action="store",
                            type="int",
                            dest="target",
                            help="Only send a single pahtloss event to the target")

    (options, args) = optionParser.parse_args()

    service = EventService((options.group,options.port,options.device))

    if len(args) < 2:
        print >>sys.stderr,"missing arguments"
        exit(1)

    targets = args[0].split(':')

    if len(targets) != 2:
        print >>sys.stderr,"invalid target format:",args[0]
        exit(1)

    try:
        targets = [int(x) for x in targets]
    except:
        print >>sys.stderr,"invalid target format:",args[0]
        exit(1)

    targets = range(targets[0],targets[1]+1)

    if not targets:
        print >>sys.stderr,"invalid target format:",args[0]
        exit(1)

    try:
        pathlossdB = float(args[1])
    except:
        print >>sys.stderr,"invalid pathloss format:",args[1]
        exit(1) 

    if targets[0] == 0:
         print >>sys.stderr,"0 is not a valid target"
         exit(1)

    # src nodes
    nodes = targets

    if options.target:
        targets = [options.target]  

    for i in targets:
        event = PathlossEvent()
        for j in nodes:
            if i != j:
                event.append(j,forward=pathlossdB)
                
        service.publish(i,event)

