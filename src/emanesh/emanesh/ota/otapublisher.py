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

from . import OTAMessage
from . import OTAPublisherException
import os
import socket
import fcntl
import uuid
import struct
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
            raise OTAPublisherException("ota socket failure %s" % str(msg),True)
        else:
            raise OTAPublisherException("ota socket failure %s %s" % (str(msg[0]), msg[1]),True)

    try:
        sock.setsockopt(socket.IPPROTO_IP,socket.IP_MULTICAST_TTL,32)
    except socket.error as msg :
        if sys.version_info >= (3,3):
            raise OTAPublisherException("ota socket option failure %s" % str(msg),True)
        else:
            raise OTAPublisherException("ota socket option failure %s %s" % (str(msg[0]), msg[1]),True)

    try:
        sock.setsockopt(socket.IPPROTO_IP,socket.IP_MULTICAST_LOOP,1)
    except socket.error as msg :
        if sys.version_info >= (3,3):
            raise OTAPublisherException("ota socket option failure %s" % str(msg),True)
        else:
            raise OTAPublisherException("ota socket option failure %s %s" % (str(msg[0]), msg[1]),True)

    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    except socket.error as msg :
        if sys.version_info >= (3,3):
            raise OTAPublisherException("ota socket option failure %s" % str(msg),True)
        else:
            raise OTAPublisherException("ota socket option failure %s %s" % (str(msg[0]), msg[1]),True)

    try:
        sock.bind((group,port))
    except socket.error as msg:
        if sys.version_info >= (3,3):
            raise OTAPublisherException("bind failure %s" % str(msg),True)
        else:
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
        if sys.version_info >= (3,3):
            raise OTAPublisherException("mulicast add membership failure %s" % str(msg),True)
        else:
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
