#
# Copyright (c) 2015,2017 - Adjacent Link LLC, Bridgewater, New Jersey
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

from __future__ import absolute_import, division, print_function
from pkg_resources import resource_filename
import sys
import re
import collections
from lxml import etree

def decodeSI(value):
    if not isinstance(value,str):
        return None

    tmp = value.lstrip('0')

    powerOf10 = 0

    if tmp[-1] == 'G':
        powerOf10 = 9

    elif tmp[-1] == 'M':
        powerOf10 = 6

    elif tmp[-1] == 'K':
        powerOf10 = 3

    if powerOf10:
        tmp = tmp[0:-1]

        # location of decimal point, if exists
        indexPoint = tmp.find('.')

        if indexPoint != -1:
            numberOfDigitsAfterPoint = len(tmp) - indexPoint - 1

            if numberOfDigitsAfterPoint > powerOf10:
                # need to move the decimal point, enough digits are present
                splitPoint = len(tmp) - (numberOfDigitsAfterPoint - powerOf10)
                tmp = tmp[0:splitPoint] + '.' + tmp[splitPoint:]
            else:
                # need to add some trailing 0s
                tmp += '0' * (powerOf10 - numberOfDigitsAfterPoint)

            # remove the decimal point
            tmp = tmp.replace('.','',1)

        else:
            # need to add trailing 0s
            tmp += '0' * powerOf10

    return tmp

def expandIndex(index):
    result = set()
    for item in index.split(','):
        match = re.match(r"(\d+):(\d+)",item)

        if match:
            list(map(lambda x: result.add(x),list(range(int(match.group(1)),int(match.group(2))+1))))
        else:
            result.add(int(item))

    return result

class TDMASchedule(object):
    def __init__(self,scheduleXML):
        self._structure = None
        self._multiFrameDefaults = {}
        self._frameDefaults = {}
        self._info = {}

        tree = etree.parse(scheduleXML)

        root = tree.getroot()

        schemaDoc = etree.parse(resource_filename('emane.events',
                                                  'schema/tdmaschedule.xsd'))

        schema = etree.XMLSchema(etree=schemaDoc,attribute_defaults=True)

        if not schema(root):
            message = []
            for entry in schema.error_log:
                message.append("%d: %s" % (entry.line,entry.message))
            print("\n".join(message), file=sys.stderr)
            exit(1)

        # if structrure is present this is a full schedule
        for structure in root.xpath('/emane-tdma-schedule/structure'):
            self._structure = {'slotduration' : int(structure.get('slotduration')),
                               'slotoverhead' : int(structure.get('slotoverhead')),
                               'bandwidth' : int(decodeSI(structure.get('bandwidth'))),
                               'slots' : int(structure.get('slots')),
                               'frames' : int(structure.get('frames'))}

        def nodeDefaults(node):
            frequency = decodeSI(node.get('frequency'));
            power = node.get('power')
            serviceClass = node.get('class')
            datarate = decodeSI(node.get('datarate'))
            defaults = {}

            if frequency != None:
                defaults['frequency'] = int(frequency)

            if power != None:
                defaults['power'] = float(power)

            if serviceClass != None:
                defaults['service'] = int(serviceClass)

            if datarate != None:
                defaults['datarate'] = int(datarate)

            return defaults


        multiframe = root.xpath('/emane-tdma-schedule/multiframe')[0]

        defaults = nodeDefaults(multiframe)

        if defaults:
            self._multiFrameDefaults = defaults

        for frame in multiframe.xpath('frame'):
            frameIndexes = expandIndex(frame.get('index'))

            defaults = nodeDefaults(frame)

            for frameIndex in frameIndexes:
                self._frameDefaults[frameIndex] = defaults

            for slot in frame.xpath('slot'):
                slotIndexes = expandIndex(slot.get('index'))

                nodes = expandIndex(slot.get('nodes'))

                args = {'type' : 'tx'}

                for child in slot:
                    if child.tag == 'tx':
                        args['type'] = 'tx'

                        frequency = decodeSI(child.get('frequency'))

                        if frequency != None:
                            args['frequency'] = int(frequency)

                        power = child.get('power')

                        if power != None:
                            args['power'] = float(power)

                        serviceClass = child.get('class')

                        if serviceClass != None:
                            args['service'] = int(serviceClass)


                        datarate = decodeSI(child.get('datarate'))

                        if datarate != None:
                            args['datarate'] = int(datarate)

                        destination = child.get('destination')

                        if destination != None:
                            args['destination'] = int(destination)

                    elif  child.tag == 'rx':
                        args['type'] = 'rx'

                        frequency = decodeSI(child.get('frequency'))

                        if frequency != None:
                            args['frequency'] = int(frequency)

                    else:
                        args['type'] = 'idle'


                for node in nodes:
                    for frameIndex in frameIndexes:
                        for slotIndex in slotIndexes:
                            if node not in self._info:
                                self._info[node] = {}

                            if frameIndex not in self._info[node]:
                                self._info[node][frameIndex] = {}

                            self._info[node][frameIndex][slotIndex] = args


    def defaults(self):
        return self._multiFrameDefaults

    def defaultsFrame(self):
        return self._frameDefaults

    def info(self):
        return self._info

    def structure(self):
        return self._structure
