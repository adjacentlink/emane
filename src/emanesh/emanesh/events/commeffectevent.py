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

import commeffectevent_pb2

class CommEffectEvent(Event):
    IDENTIFIER = 103

    def __init__(self):
        self._event = commeffectevent_pb2.CommEffectEvent()

    def append(self,nemId,**kwargs):
        commeffect = self._event.commEffects.add()

        commeffect.nemId = nemId
        
        for (name,value) in kwargs.items():

            if name == 'latency':
                if (isinstance(value,int) or \
                    isinstance(value,float)) and \
                value >= 0:
                    commeffect.latencySeconds = value
                else:
                    raise ValueError("latency must be a positive numeric")

            elif name == 'jitter':
                if (isinstance(value,int) or \
                    isinstance(value,float)) and \
                    value >= 0:
                    commeffect.jitterSeconds = value
                else:
                    raise ValueError("jitter must be a positive numeric")

            elif name == 'loss':
                if (isinstance(value,int) or \
                    isinstance(value,float)) and \
                    value >= 0:
                    commeffect.probabilityLoss = value
                else:
                    raise ValueError("loss must be numeric")

            elif name == 'duplicate':
                if (isinstance(value,int) or \
                    isinstance(value,float)) and \
                    value >= 0:
                    commeffect.probabilityDuplicate = value
                else:
                    raise ValueError("duplicate must be a positive numeric")

            elif name == 'unicast':
                if (isinstance(value,long) or isinstance(value,int)) and \
                   value >= 0:
                    commeffect.unicastBitRatebps = value
                else:
                    raise ValueError("unicast must be a positive integer")

            elif name == 'broadcast':
                if (isinstance(value,long) or isinstance(value,int)) and \
                   value >= 0:
                    commeffect.broadcastBitRatebps = value
                else:
                    raise ValueError("broadcast must be a positive integer")

            else:
                raise KeyError("unknown parameter: %s" % name)

            
    def serialize(self):
        return self._event.SerializeToString()

    def restore(self,data):
        self._event.ParseFromString(data)
        
    def __iter__(self):
        for commeffect in self._event.commEffects:
            kwargs = {'latency': commeffect.latencySeconds,
                      'jitter' : commeffect.jitterSeconds,
                      'loss' : commeffect.probabilityLoss,
                      'duplicate' : commeffect.probabilityDuplicate,
                      'unicast' : commeffect.unicastBitRatebps,
                      'broadcast' : commeffect.broadcastBitRatebps}

            yield (commeffect.nemId,kwargs)

                
if __name__ == "__main__":
    import sys
    from optparse import OptionParser
    from emanesh.events import EventService

    usage = """emaneevent-commeffect [OPTION]... NEMID[:NEMID] EFFECTS

       EFFECTS := EFFECT [EFFECT]...

       EFFECT  := 'loss'=PERCENT      |
                  'duplicate'=PERCENT |
                  'latency'=SECS      |
                  'jitter'=SECS       |
                  'unicast'=BPS       |
                  'broadcast'=BPS

       PERCENT := percentage (float)

       SECS    := seconds (float)

       BPS     := bits/second (ulong), 0 indicates no limit
"""
    description="Publish a comm effect event to all or some of the NEMs specified in a range."

    epilog="""
The NEM range specification creates a logical two dimensional matrix.

For example:

  emaneevent-commeffect 1:10 unicast=1000000 jitter=1.5 loss=25

Will create the following logical matrix:

       Transmitters

       1  2  3  4  5  6  7  8  9  10
R   1     E  E  E  E  E  E  E  E  E
e   2  E     E  E  E  E  E  E  E  E
c   3  E  E     E  E  E  E  E  E  E
e   4  E  E  E     E  E  E  E  E  E
i   5  E  E  E  E     E  E  E  E  E
v   6  E  E  E  E  E     E  E  E  E
e   7  E  E  E  E  E  E     E  E  E
r   8  E  E  E  E  E  E  E     E  E
s   9  E  E  E  E  E  E  E  E     E
   10  E  E  E  E  E  E  E  E  E   

where, E is (loss=25, duplication=0, jitter=1.5 latency=0,
unicast=1000000, broadcast=0).

Missing effects default to 0.

Each NEM receives one event with its respective matrix slice
containing the effect value to use for each respective
transmitter.

This example will publish 10 comm effect events, each event will target
a specific NEM in the range [1,10].

You can use the '--target' option and the '--reference' to specifically
target one or more NEMs in the range with one of more of the
transmitter values. The NEMs specified with '--target' do not have to
be in the range.

For example:

  emaneevent-commeffect 1:10 unicast=1000000 jitter=1.5 \\
              loss=25 -t 3 -t 4 -r 8 -r 9 -r 10

will send events to NEM 3 and 4 containing the transmitter information
for NEMs 8, 9 and 10.

  emaneevent-commeffect 13 -t 7 -t 8 latency=.5

will send a comm effect event to NEM 7 and 8 with a 500msec latency
value and 0 for all other effects for packets transmitted by NEM 13.

  emaneevent-commeffect 15 -t 0 loss=10 jitter=.1

will send a comm effect event to all NEMs with a loss value of 10%, a
jitter value of 100msec and 0 for all other effects for packets
transmitted by NEM 15.

Asymmetric link effects can be achieved my invoking emaneevent-commeffect
multiple times with different targets.

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
                            help="Only send an  event to the target")

    optionParser.add_option("-r",
                            "--reference",
                            action="append",
                            type="int",
                            dest="reference",
                            help="Send events to targeted NEMs but only include information for the reference NEM.")


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

    if  len(nems) > 1:
        nems = range(nems[0],nems[1]+1)

    if not nems:
        print >>sys.stderr,"invalid NEMID format:",args[0]
        exit(1)
        
    kwargs  = {'loss':0,
               'duplicate':0,
               'latency':0,
               'jitter':0,
               'unicast':0L,
               'broadcast':0L}
    
    for effectinfo  in args[1:]:
        try:
            (effect,value) = effectinfo.split("=")
        except:
           print >>sys.stderr,"invalid effect format:",effectinfo
           exit(1)  

        if effect == 'loss' or \
           effect == 'duplicate' or \
           effect == 'latency' or \
           effect == 'jitter':
            try:
                kwargs[effect] = float(value)
            except:
                print >>sys.stderr,"invalid %s format:"% effect,value
                exit(1) 
        elif effect == 'unicast' or \
             effect == 'broadcast':
            try:
                kwargs[effect] = long(value)
            except:
                print >>sys.stderr,"invalid %s format:"% effect,value
                exit(1) 
        else:
            print >>sys.stderr,"invalid effect:",effect
            exit(1)  

    if nems[0] == 0:
        print >>sys.stderr,"0 is not a valid NEMID"
        exit(1)

    if options.target:
        targets = options.target
    else:
        targets = nems
    
    if options.reference:
        references = options.reference
    else:
        references = nems

    for i in targets:
        event = CommEffectEvent()
        for j in nems:
            if i != j and j in references:
                try:
                    event.append(j,**kwargs)
                except Exception, exp:
                    print  >>sys.stderr, "error:",exp
                    exit(1)

        service.publish(i,event)
