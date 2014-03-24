#
# Copyright (c) 2014 - Adjacent Link LLC, Bridgewater, New Jersey
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

from . import OTAMessage
from . import OTAPublisherException
import os
import socket
import fcntl
import uuid
import threading
import struct

def get_ip_address(ifname):
    # http://code.activestate.com/recipes/439094/
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15])
    )[20:24])

def init_multicast_socket(group,port,device):
    try:
        sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

    except socket.error, msg :
        raise OTAPublisherException("ota socket failure %s %s" % (str(msg[0]), msg[1]),True)

    try:
        sock.setsockopt(socket.IPPROTO_IP,socket.IP_MULTICAST_TTL,32)
    except socket.error, msg :
        raise OTAPublisherException("ota socket option failure %s %s" % (str(msg[0]), msg[1]),True)

    try:
        sock.setsockopt(socket.IPPROTO_IP,socket.IP_MULTICAST_LOOP,1)
    except socket.error, msg :
        raise OTAPublisherException("ota socket option failure %s %s" % (str(msg[0]), msg[1]),True)

    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    except socket.error, msg :
        raise OTAPublisherException("ota socket option failure %s %s" % (str(msg[0]), msg[1]),True)

    try:
        sock.bind((group,port))
    except socket.error as msg:
        raise OTAPublisherException("bind failure %s %s" % (str(msg[0]), msg[1]),True)

    try:
        if device:
            devAddress = socket.inet_aton(get_ip_address(device))
        else:
            devAddress = socket.inet_aton("0.0.0.0")

        sock.setsockopt(socket.SOL_IP,
                        socket.IP_ADD_MEMBERSHIP,
                        socket.inet_aton(group) + 
                        devAddress)

        sock.setsockopt(socket.SOL_IP,
                        socket.IP_MULTICAST_IF,
                        devAddress)


    except socket.error as msg:
        raise OTAPublisherException("mulicast add membership failure %s %s" % (str(msg[0]), msg[1]),True)

    except IOError:
        raise  OTAPublisherException("unknown device %s" % device,True)

    return sock


class OTAPublisher:
    def __init__(self,otachannel):
        (self._multicastGroup,self._port,_) = otachannel
        self._socket = None
        self._uuid = uuid.uuid4()
        self._sequenceNumber = 0

        self._socket = init_multicast_socket(*otachannel)


    def publish(self,otaMessage,txTimeMicroseconds):
        self._socket.sendto(otaMessage.generate(txTimeMicroseconds,
                                                self._sequenceNumber,
                                                self._uuid)
                            ,(self._multicastGroup,self._port))


if __name__ == "__main__":
    from optparse import OptionParser
    from emanesh.ota import OTAMessage
    from emanesh.ota import OTAPublisher
    from pkg_resources import resource_filename
    from lxml import etree
    import sys
    import re
    import time

    class Flow(threading.Thread):
        def __init__(self,publisher,rate,start,end,otaMessage):
            threading.Thread.__init__(self)
            self._publisher = publisher
            self._otaMessage = otaMessage
            self._rate = rate
            self._start = start
            self._end = end
            self._finishEvent = threading.Event()

        def stop(self):
            self._finishEvent.set() 

        def run(self):
            now = time.time()
            
            # wait for start
            if now < self._start:
                if self._finishEvent.wait(self._start - now):
                    return
                    
            delta = 1.0 / self._rate
            
            sot = time.time() + delta

            while not self._finishEvent.wait(delta):
                
                if time.time() < self._end:
                    self._publisher.publish(self._otaMessage,int(sot * 1000000))
                else:
                    break
                

    usage = "emaneota-publisher [OPTION]..."

    optionParser = OptionParser(usage=usage)

    optionParser.add_option("-p", 
                            "--port",
                            action="store",
                            type="int",
                            dest="port",
                            default=45702,
                            help="OTA channel listen port [default: %default]")

    optionParser.add_option("-g", 
                            "--group",
                            action="store",
                            type="string",
                            dest="group",
                            default="224.1.2.8",
                            help="OTA channel multicast group [default: %default]")

    optionParser.add_option("-i",
                            "--device",
                            action="store",
                            type="string",
                            dest="device",
                            help="OTA channel multicast device")

    optionParser.add_option("", 
                            "--starttime",
                            action="store",
                            type="string",
                            dest="starttime",
                            help="Specify test start time HH:MM:SS")


    (options, args) = optionParser.parse_args()

    if len(args) != 1:
        print >>sys.stderr, "Invalid number of arguments"
        exit(1)

    startTime = None

    if options.starttime:
        m = re.match("(\d\d):(\d\d):(\d\d)",options.starttime)

        if m:
            now = time.localtime()

            startTime = time.mktime((now.tm_year,
                                    now.tm_mon,
                                    now.tm_mday,
                                    int(m.group(1)),
                                    int(m.group(2)),
                                    int(m.group(3)),
                                    now.tm_wday,
                                    now.tm_yday,
                                    -1))

            if startTime < time.time():
               print >>sys.stderr,"start time in the past"
               exit(1) 
    else:
        startTime = time.time()

    publisher = OTAPublisher((options.group,options.port,options.device),)

    flows = []

    tree = etree.parse(args[0])
    root = tree.getroot()
    
    schemaDoc = etree.parse(resource_filename('emanesh', 'schema/otapublisherscenario.xsd'))
    
    schema = etree.XMLSchema(etree=schemaDoc,attribute_defaults=True)

    if not schema(root):
        message = ""
        for entry in schema.error_log:
            print >>sys.stderr,"%d: %s" % (entry.line,entry.message)
        exit(1)

    for flow in root.xpath('/otapublisherscenario/flow'):

        transmitters = []

        for transmitter in flow.xpath('transmitters/transmitter'):
            transmitters.append((int(transmitter.get('nem')),float(transmitter.get('power'))))


        segments = []

        for segment in flow.xpath('segments/segment'):
            segments.append((int(segment.get('frequency')),int(segment.get('offset')),int(segment.get('duration'))))

        fixedgain = flow.get('fixedgain')
        start = float(flow.get('start'))
        end = float(flow.get('end'))
        
        flow = Flow(publisher,
                    float(flow.get('rate')),
                    startTime + start,
                    startTime + end,
                    OTAMessage(int(flow.get('source')),
                               int(flow.get('destination')),
                               int(flow.get('registrationid')),
                               int(flow.get('subid')),
                               int(flow.get('bandwidth')),
                               transmitters,
                               segments,
                               fixedgain if fixedgain is None else int(fixedgain)))
        flows.append(flow)


    for flow in flows: 
        flow.start()

    try:
        for flow in flows:
            while flow.is_alive():
                flow.join(timeout=0.5)

    except (KeyboardInterrupt, SystemExit):
        for flow in flows:
            flow.stop()

