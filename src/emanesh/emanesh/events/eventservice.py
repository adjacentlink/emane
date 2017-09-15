#
# Copyright (c) 2013-2015,2017 - Adjacent Link LLC, Bridgewater,
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

from . import event_pb2
from ..ota import otaheader_pb2
from . import EventServiceException
import os
import socket
import threading
import fcntl
import struct
import select
import time
import uuid
import sys

def get_ip_address(ifname):
    # http://code.activestate.com/recipes/439094/
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15].encode() if sys.version_info >= (3,0) else ifname[:15])
    )[20:24])


def init_multicast_socket(group,port,device):
    try:
        sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

    except socket.error as msg :
        if sys.version_info >= (3,3):
            raise EventServiceException("event socket failure %s" % str(msg),True)
        else:
            raise EventServiceException("event socket failure %s %s" % (str(msg[0]), msg[1]),True)

    try:
        sock.setsockopt(socket.IPPROTO_IP,socket.IP_MULTICAST_TTL,32)
    except socket.error as msg :
        if sys.version_info >= (3,3):
            raise EventServiceException("event socket option failure %s" % str(msg),True)
        else:
            raise EventServiceException("event socket option failure %s %s" % (str(msg[0]), msg[1]),True)

    try:
        sock.setsockopt(socket.IPPROTO_IP,socket.IP_MULTICAST_LOOP,1)
    except socket.error as msg :
        if sys.version_info >= (3,3):
            raise EventServiceException("event socket option failure %s" % str(msg),True)
        else:
            raise EventServiceException("event socket option failure %s %s" % (str(msg[0]), msg[1]),True)

    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    except socket.error as msg :
        if sys.version_info >= (3,3):
            raise EventServiceException("event socket option failure %s" % str(msg),True)
        else:
            raise EventServiceException("event socket option failure %s %s" % (str(msg[0]), msg[1]),True)

    try:
        sock.bind((group,port))
    except socket.error as msg:
        if sys.version_info >= (3,3):
            raise EventServiceException("bind failure %s" % str(msg),True)
        else:
            raise EventServiceException("bind failure %s %s" % (str(msg[0]), msg[1]),True)

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
        if sys.version_info >= (3,3):
            raise EventServiceException("mulicast add membership failure %s" % str(msg),True)
        else:
            raise EventServiceException("mulicast add membership failure %s %s" % (str(msg[0]), msg[1]),True)

    except IOError:
        raise  EventServiceException("unknown device %s" % device,True)

    return sock


class EventService:
    def __init__(self,eventchannel,otachannel = None):
        (self._multicastGroup,self._port,_) = eventchannel
        self._defaultHandler = None
        self._handlers = {}
        self._socket = None
        self._readFd,self._writeFd = os.pipe()
        self._uuid = uuid.uuid4()
        self._sequenceNumber = 0
        self._socketOTA = None

        self._socket = init_multicast_socket(*eventchannel)

        if otachannel:
            self._socketOTA = init_multicast_socket(*otachannel)

        self._lock = threading.Lock()

    def breakloop(self):
        os.write(self._writeFd,"\n".encode())

    def loop(self,default=None):
        buffer = ""
        running = True

        while running:
            rdfds = [self._socket,self._readFd]

            if self._socketOTA:
                rdfds.append(self._socketOTA)

            try:
                readable,_,_ = select.select(rdfds,[],[])
            except select.error:
                continue

            for fd in readable:
                if fd is self._socket:
                    data,_ = self._socket.recvfrom(65535)

                    if not len(data):
                        running = False
                        break

                    (length,) = struct.unpack_from("!H",data)

                    if length == len(data) - 2:
                        event = event_pb2.Event()

                        event.ParseFromString(data[2:])

                        for serialization in event.data.serializations:
                            self._lock.acquire()

                            try:

                                if serialization.eventId in self._handlers:
                                    self._handlers[serialization.eventId](serialization.nemId,
                                                                          serialization.eventId,
                                                                          serialization.data,
                                                                          uuid.UUID(bytes=event.uuid),
                                                                          event.sequenceNumber)
                                elif default:
                                    default(serialization.nemId,
                                            serialization.eventId,
                                            serialization.data,
                                            uuid.UUID(bytes=event.uuid),
                                            event.sequenceNumber)
                            finally:
                                self._lock.release()

                elif fd is self._readFd:
                    running = False
                    break

                elif fd is self._socketOTA:
                    data,_ = self._socketOTA.recvfrom(65535)

                    if not len(data):
                        running = False
                        break

                    (headerLength,) = struct.unpack_from("!H",data)

                    otaHeader = otaheader_pb2.OTAHeader()

                    otaHeader.ParseFromString(data[2:headerLength+2])

                    eventData = event_pb2.Event.Data()

                    eventData.ParseFromString(data[2+headerLength:2 + headerLength +otaHeader.eventLength])

                    for serialization in eventData.serializations:
                        self._lock.acquire()

                        try:

                            if serialization.eventId in self._handlers:
                                self._handlers[serialization.eventId](serialization.nemId,
                                                                      serialization.eventId,
                                                                      serialization.data,
                                                                      uuid.UUID(bytes=otaHeader.uuid),
                                                                      otaHeader.sequenceNumber)
                            elif default:
                                default(serialization.nemId,
                                        serialization.eventId,
                                        serialization.data,
                                        uuid.UUID(bytes=otaHeader.uuid),
                                        otaHeader.sequenceNumber)
                        finally:
                            self._lock.release()


    def nextEvent(self):
        events = []
        eventId = 0
        running = True

        while running:
            try:
                rdfds = [self._socket,self._readFd]

                if self._socketOTA:
                    rdfds.append(self._socketOTA)

                readable,_,_ = select.select(rdfds,[],[])
            except select.error:
                continue

            for fd in readable:
                if fd is self._socket:
                    data,_ = self._socket.recvfrom(65535)

                    if not len(data):
                        running = False
                        break

                    (length,) = struct.unpack_from("!H",data)

                    if length == len(data) - 2:
                        event = event_pb2.Event()

                        event.ParseFromString(data[2:])

                        for serialization in event.data.serializations:
                            events.append((serialization.nemId,
                                           serialization.eventId,
                                           serialization.data))

                        return (uuid.UUID(bytes=event.uuid),
                                event.sequenceNumber,
                                tuple(events))

                elif fd is self._readFd:
                    running = False
                    break

                elif fd is self._socketOTA:
                    data,_ = self._socketOTA.recvfrom(65535)

                    if not len(data):
                        running = False
                        break

                    (headerLength,) = struct.unpack_from("!H",data)

                    otaHeader = otaheader_pb2.OTAHeader()

                    otaHeader.ParseFromString(data[2:headerLength+2])

                    eventData = event_pb2.Event.Data()

                    eventData.ParseFromString(data[2+headerLength:2 + headerLength +otaHeader.eventLength])

                    for serialization in eventData.serializations:
                        events.append((serialization.nemId,
                                       serialization.eventId,
                                       serialization.data))

                    return (uuid.UUID(bytes=otaHeader.uuid),
                            otaHeader.sequenceNumber,
                            tuple(events))

        return (None, None, tuple(events))


    def subscribe(self,eventId,callback):
        self._lock.acquire()

        if callback:
            self._handlers[eventId] = callback

        self._lock.release()


    def unsubscribe(self,eventId):
        self._lock.acquire()

        if eventId in self._handlers:
            del self._handlers[eventId]

        self._lock.release()


    def publish(self,nemId,event):
        self._sequenceNumber += 1
        msg = event_pb2.Event()
        msg.uuid = self._uuid.bytes
        msg.sequenceNumber =  self._sequenceNumber
        serialization = msg.data.serializations.add()
        serialization.nemId = nemId
        serialization.eventId = event.IDENTIFIER
        serialization.data = event.serialize()

        buf = msg.SerializeToString()

        self._socket.sendto(struct.pack("!H",len(buf)) + buf,
                            (self._multicastGroup,self._port))
