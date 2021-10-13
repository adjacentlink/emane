#
# Copyright (c) 2014,2017 - Adjacent Link LLC, Bridgewater, New Jersey
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

from . import otaheader_pb2
from . import commonphyheader_pb2
import uuid
import struct

class OTAMessage:
    def __init__(self,
                 source,
                 destination,
                 registrationId,
                 subId,
                 bandwidthHz,
                 transmitters,
                 segments,
                 fixedAntennaGain = None):

        self._sequence = 0
        self._otaHeader = otaheader_pb2.OTAHeader()
        self._otaHeader.source = source
        self._otaHeader.destination = destination
        self._otaHeader.payloadInfo.eventLength = 0
        self._otaHeader.payloadInfo.controlLength = 2
        self._otaHeader.payloadInfo.dataLength = 0

        self._phyHeader = commonphyheader_pb2.CommonPHYHeader()
        self._phyHeader.registrationId = registrationId;
        self._phyHeader.subId = subId;

        for nemId,powerdBm in transmitters:
            transmitter = self._phyHeader.transmitters.add()
            transmitter.nemId = nemId
            transmitter.powerdBm = powerdBm

        frequencyGroup =  self._phyHeader.frequencyGroups.add()

        for frequencyHz, offsetMicroseconds, durationMicroseconds in segments:
            segment = frequencyGroup.frequencySegments.add()
            segment.frequencyHz = frequencyHz
            segment.offsetMicroseconds = offsetMicroseconds
            segment.durationMicroseconds = durationMicroseconds

        txantenna = self._phyHeader.transmitAntennas.add()
        txantenna.antennaIndex = 0
        txantenna.bandwidthHz = bandwidthHz
        txantenna.frequencyGroupIndex = 0

        if fixedAntennaGain is not None:
            txantenna.fixedGaindBi = fixedAntennaGain

    def generate(self,txTimeMicroseconds,sequence,uuid):
        self._otaHeader.uuid = uuid.bytes
        self._otaHeader.sequence = sequence
        self._phyHeader.sequenceNumber = self._sequence;
        self._sequence += 1
        self._phyHeader.txTimeMicroseconds = txTimeMicroseconds;

        phyHeader = self._phyHeader.SerializeToString()

        self._otaHeader.payloadInfo.dataLength = len(phyHeader) + 2;

        otaHeader = self._otaHeader.SerializeToString()

        return b"".join((struct.pack("!H",len(otaHeader)),
                        otaHeader,
                        # PartInfo: no more parts, offset 0, size
                        struct.pack("!BII",
                                    0,
                                    0,
                                    self._otaHeader.payloadInfo.dataLength +
                                    self._otaHeader.payloadInfo.controlLength +
                                    self._otaHeader.payloadInfo.eventLength),
                        struct.pack("!H",0),
                        struct.pack("!H",len(phyHeader)),
                        phyHeader))
