#
# Copyright (c) 2015 - Adjacent Link LLC, Bridgewater, New Jersey
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

import tdmascheduleevent_pb2
from collections import namedtuple

class TDMAScheduleEvent(Event):
    IDENTIFIER = 105
    TX=1
    RX=2
    IDLE=3

    _FrameEntry = namedtuple('FrameEntry', ['frame', 'slots'])

    def __init__(self,**kwargs):
        self._event = tdmascheduleevent_pb2.TDMAScheduleEvent()
        self._frames = {}

        for (name,value) in kwargs.items():
            if name == 'frequency':
                if (isinstance(value,int) or \
                    isinstance(value,long)) and \
                    value >= 0:
                    self._event.frequencyHz = value
                else:
                    raise ValueError("frequency must be a positive numeric")

            elif name == 'datarate':
                if (isinstance(value,int) or \
                    isinstance(value,long)) and \
                    value >= 0:
                    self._event.dataRatebps = value
                else:
                    raise ValueError("datarate must be a positive numeric")

            elif name == 'service':
                if (isinstance(value,int) or \
                    isinstance(value,long)) and \
                    value >= 0:
                    self._event.serviceClass = value
                else:
                    raise ValueError("service must be a positive numeric")

            elif name == 'power':
                if (isinstance(value,int) or \
                    isinstance(value,long) or \
                    isinstance(value,float)):
                    self._event.powerdBm = value
                else:
                    raise ValueError("power must be a numeric")

            else:
                raise KeyError("unknown parameter: %s" % name)

    def structure(self,**kwargs):
        slotsPerFrame = None
        framesPerMultiFrame = None
        slotOverheadMicroseconds = None
        slotDurationMicroseconds = None
        bandwidthHz = None

        if not kwargs:

            if self._event.HasField('structure'):
                return {'slots': self._event.structure.slotsPerFrame,
                        'frames': self._event.structure.framesPerMultiFrame,
                        'slotduration':self._event.structure.slotDurationMicroseconds,
                        'slotoverhead':self._event.structure.slotOverheadMicroseconds,
                        'bandwidth': self._event.structure.bandwidthHz}
            else:
                return None

        for (name,value) in kwargs.items():
            if name == 'slots':
                if (isinstance(value,int) or \
                    isinstance(value,long)) and \
                    value > 0:
                    slotsPerFrame = value
                else:
                    raise ValueError("'slots' must be a positive integer greater than 0")

            elif name == 'frames':
                if (isinstance(value,int) or \
                    isinstance(value,long)) and \
                    value > 0:
                    framesPerMultiFrame = value
                else:
                    raise ValueError("'frames' must be a positive integer greater than 0")

            elif name == 'slotduration':
                if (isinstance(value,int) or \
                    isinstance(value,long)) and \
                    value > 0:
                    slotDurationMicroseconds = value
                else:
                    raise ValueError("'slotduration' must be a positive integer greater than 0")

            elif name == 'slotoverhead':
                if (isinstance(value,int) or \
                    isinstance(value,long)) and \
                    value >= 0:
                    slotOverheadMicroseconds = value
                else:
                    raise ValueError("'slotoverhead' must be a positive integer (usecs)")

            elif name == 'bandwidth':
                if (isinstance(value,int) or \
                    isinstance(value,long)) and \
                    value > 0:
                    bandwidthHz = value
                else:
                    raise ValueError("'bandwidth' must be a positive integer greater than 0 (Hz)")

            else:
                raise KeyError("unknown parameter: %s" % name)


        if slotsPerFrame == None or \
           framesPerMultiFrame == None or \
           slotOverheadMicroseconds == None or \
           slotDurationMicroseconds == None or \
           bandwidthHz == None:
            raise KeyError("Missing one ore more keys: 'slots', 'frames', 'slotduration', 'slotoverhead', 'bandwidth'")

        self._event.structure.slotsPerFrame = slotsPerFrame
        self._event.structure.framesPerMultiFrame = framesPerMultiFrame
        self._event.structure.slotDurationMicroseconds = slotDurationMicroseconds
        self._event.structure.slotOverheadMicroseconds = slotOverheadMicroseconds
        self._event.structure.bandwidthHz = bandwidthHz


    def append(self,frameIndex,slotIndex,**kwargs):
        frameFrequencyHz = None
        frameDataRatebps = None
        frameClass = None
        framePowerdBm = None
        slotFrequencyHz = None
        slotDataRatebps = None
        slotClass = None
        slotPowerdBm = None
        slotType = None
        slotDestination = None

        if frameIndex in self._frames and \
           slotIndex in self._frames[frameIndex].slots:
            raise ValueError("slot index already defined for frame")

        for (name,value) in kwargs.items():
            if name == 'frame.frequency':
                if (isinstance(value,int) or \
                    isinstance(value,long)) and \
                value > 0:
                    frameFrequencyHz = value
                else:
                    raise ValueError("'frame.frequency' must be a integer greater that 0 (Hz)")

            elif name == 'frame.datarate':
                if (isinstance(value,int) or \
                    isinstance(value,long)) and \
                value > 0:
                    frameDataRatebps = value
                else:
                    raise ValueError("'frame.datarate' must be a positive integer greater than 0 (bps)")

            elif name == 'frame.service':
                if (isinstance(value,int) or \
                    isinstance(value,long)) and \
                value >= 0 and value <= 3:
                    frameClass = value
                else:
                    raise ValueError("'frame.service' must be a positive integer in the set [0,3]")

            elif name == 'frame.power':
                if (isinstance(value,int) or \
                    isinstance(value,long) or \
                    isinstance(value,float)):

                    framePowerdBm = value
                else:
                    raise ValueError("'frame.power' must be a numeric (dBm)")

            elif name == 'frequency':
                if (isinstance(value,int) or \
                    isinstance(value,long)) and \
                value > 0:
                    slotFrequencyHz = value
                else:
                    raise ValueError("'frequency' must be a integer greater that 0 (Hz)")

            elif name == 'datarate':
                if (isinstance(value,int) or \
                    isinstance(value,long)) and \
                value > 0:
                    slotDataRatebps = value
                else:
                    raise ValueError("'datarate' must be a positive integer greater than 0 (bps)")

            elif name == 'service':
                if (isinstance(value,int) or \
                    isinstance(value,long)) and \
                value >= 0 and value <= 3:
                    slotClass = value
                else:
                    raise ValueError("'service' must be a positive integer in the set [0,3]")

            elif name == 'power':
                if (isinstance(value,int) or \
                    isinstance(value,long) or \
                    isinstance(value,float)):
                    slotPowerdBm = value
                else:
                    raise ValueError("'power' must be a numeric (dBm)")


            elif name == 'destination':
                if (isinstance(value,int) or \
                    isinstance(value,long)) and \
                value > 0:
                    slotDestination = value
                else:
                    raise ValueError("'destination' must be a positive integer (NEM Id)")

            elif name == 'type':
                if value == "tx" or value == TDMAScheduleEvent.TX:
                    slotType = "tx"

                elif value == "rx" or value == TDMAScheduleEvent.RX:
                    slotType = "rx"

                elif value =="idle" or value == TDMAScheduleEvent.IDLE:
                    slotType = "idle"

                else:
                    raise ValueError("'type' must be one of: tx, rx or idle")

            else:
                raise KeyError("unknown parameter: %s" % name)

        if slotType == "tx":
            if slotFrequencyHz == None and \
               frameFrequencyHz == None and \
               self._event.frequencyHz == None:
                raise KeyError("tx slot 'frequency' must be specified when 'frame.frequency' missing and default not set")

            if slotDataRatebps == None and \
               frameDataRatebps == None and \
               self._event.dataRatebps == None:
                raise KeyError("tx slot 'datarate' must be specified when 'frame.datarate' missing and default not set")

            if slotClass == None and \
               frameClass == None and \
               self._event.serviceClass == None:
                raise KeyError("tx slot 'service' must be specified when 'frame.service' missing and default not set")

            if slotPowerdBm != None and \
               framePowerdBm == None and \
               self._event.powerdBm == None:
                raise KeyError("tx slot 'power' must be specified when 'frame.power' missing and default not set")

        elif slotType == "rx":
            if slotDataRatebps != None or \
               slotClass != None or \
               slotPowerdBm != None or \
               slotDestination != None:
                raise KeyError("rx slot cannot have 'datarate', 'service', 'power' and/or 'destination'")

            if slotFrequencyHz == None and \
               frameFrequencyHz == None and \
               self._event.frequencyHz == None:
                raise KeyError("rx slot 'frequency' must be specified when 'frame.frequency' missing and default not set")

        elif slotType == "idle":
            if slotFrequencyHz != None or \
               slotDataRatebps != None or \
               slotClass != None or \
               slotPowerdBm != None:
                raise ValueError("idle slot cannot have 'frequency', 'datarate', 'service', 'power',  and/or 'destination'")

        else:
            raise KeyError("missing 'type'")

        if frameIndex in self._frames:
            frame = self._frames[frameIndex].frame
        else:
            frame = self._event.frames.add()
            self._frames[frameIndex] = TDMAScheduleEvent._FrameEntry(frame,set())
            frame.index = frameIndex

        if frameFrequencyHz != None:
            frame.frequencyHz = frameFrequencyHz

        if frameDataRatebps != None:
            frame.dataRatebps = frameDataRatebps

        if frameClass != None:
            frame.serviceClass = frameClass

        if framePowerdBm != None:
            frame.powerdBm = framePowerdBm

        slot = frame.slots.add()

        slot.index = slotIndex

        if slotType == "tx":
            slot.type = tdmascheduleevent_pb2.TDMAScheduleEvent.Frame.Slot.SLOT_TX

            if slotFrequencyHz != None:
                slot.tx.frequencyHz = slotFrequencyHz

            if slotDataRatebps != None:
                slot.tx.dataRatebps = slotDataRatebps

            if slotClass != None:
                slot.tx.serviceClass = slotClass

            if slotPowerdBm != None:
                slot.tx.powerdBm = slotPowerdBm

            if slotDestination != None:
                slot.tx.destination = slotDestination

        elif slotType == "rx":
            slot.type = tdmascheduleevent_pb2.TDMAScheduleEvent.Frame.Slot.SLOT_RX

            if slotFrequencyHz != None:
                slot.rx.frequencyHz = slotFrequencyHz

        else:
            slot.type = tdmascheduleevent_pb2.TDMAScheduleEvent.Frame.Slot.SLOT_IDLE

        self._frames[frameIndex].slots.add(slotIndex)

    def serialize(self):
        return self._event.SerializeToString()

    def restore(self,data):
        self._event.ParseFromString(data)

    def __iter__(self):
        for frame in self._event.frames:
            kwargs = {'index' : frame.index}

            if frame.HasField('frequencyHz'):
                kwargs['frame.frequency'] = frame.frequencyHz

            if frame.HasField('dataRatebps'):
                kwargs['frame.datarate'] = frame.dataRatebps

            if frame.HasField('serviceClass'):
                kwargs['frame.service'] = frame.serviceClass

            if frame.HasField('powerdBm'):
                kwargs['frame.power'] = frame.powerdBm

            slots = {}

            for slot in frame.slots:
                s = {}

                if slot.type == tdmascheduleevent_pb2.TDMAScheduleEvent.Frame.Slot.SLOT_TX:
                    s['type'] = 'tx'

                    if slot.HasField('tx'):
                        if slot.tx.HasField('frequencyHz'):
                            s['frequency'] = slot.tx.frequencyHz

                        if slot.tx.HasField('dataRatebps'):
                            s['datarate'] = slot.tx.dataRatebps

                        if slot.tx.HasField('serviceClass'):
                            s['service'] = slot.tx.serviceClass

                        if slot.tx.HasField('powerdBm'):
                            s['power'] = slot.tx.powerdBm

                        if slot.tx.HasField('destination'):
                            s['destination'] = slot.tx.destination

                elif slot.type == tdmascheduleevent_pb2.TDMAScheduleEvent.Frame.Slot.SLOT_RX:
                    s['type'] = 'rx'

                    if slot.HasField('rx') and slot.rx.HasField('frequencyHz'):
                        s['frequency'] = slot.rx.frequencyHz

                else:
                    s['type'] = 'idle'

                slots[slot.index] = s

            kwargs['slots'] = slots

            yield kwargs


if __name__ == "__main__":
    import sys
    from optparse import OptionParser
    from emanesh.events import EventService
    import copy
    from . import TDMASchedule

    usage="%prog [OPTION]... NEMID..."
    description=""
    epilog=""

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

    optionParser.add_option("-f",
                            "--file",
                            action="store",
                            type="string",
                            dest="file",
                            help="Schedule XML file")

    (options, args) = optionParser.parse_args()

    if options.file:
        schedule = TDMASchedule(options.file)
    else:
        print >>sys.stderr, "currently only --file supported"
        exit(1)

    eventService = EventService((options.group,options.port,options.device))

    info = schedule.info()

    structure = schedule.structure()

    frameDefaults =  schedule.defaultsFrame();

    for nodeId in info:
        event = TDMAScheduleEvent(**schedule.defaults())
        for frameIndex in info[nodeId]:
            for slotIndex,args in info[nodeId][frameIndex].items():
                defaults = args
                for key,value in frameDefaults[frameIndex].items():
                    defaults["frame." + key] = value
                event.append(frameIndex,slotIndex,**defaults)

        if structure != None:
            event.structure(**structure)

        eventService.publish(nodeId,event)
