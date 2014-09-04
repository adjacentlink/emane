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

import remotecontrolportapi_pb2
from . import ControlPortException
import socket
import sys
import select
import struct
import threading 
import os

def fromAny(any):
    if any.type == remotecontrolportapi_pb2.Any.TYPE_ANY_INT8:
        return (any.i32Value,ControlPortClient.TYPE_INT8)
    elif any.type == remotecontrolportapi_pb2.Any.TYPE_ANY_UINT8:
        return (any.u32Value,ControlPortClient.TYPE_UINT8)
    elif any.type == remotecontrolportapi_pb2.Any.TYPE_ANY_INT16:
        return (any.i32Value,ControlPortClient.TYPE_INT16)
    elif any.type == remotecontrolportapi_pb2.Any.TYPE_ANY_UINT16:
        return (any.u32Value,ControlPortClient.TYPE_UINT16)
    elif any.type == remotecontrolportapi_pb2.Any.TYPE_ANY_INT32:
        return (any.i32Value,ControlPortClient.TYPE_INT32)
    elif any.type == remotecontrolportapi_pb2.Any.TYPE_ANY_UINT32:
        return (any.u32Value,ControlPortClient.TYPE_UINT32)
    elif any.type == remotecontrolportapi_pb2.Any.TYPE_ANY_INT64:
        return (any.i64Value,ControlPortClient.TYPE_INT64)
    elif any.type == remotecontrolportapi_pb2.Any.TYPE_ANY_UINT64:
        return (any.u64Value,ControlPortClient.TYPE_UINT64)
    elif any.type == remotecontrolportapi_pb2.Any.TYPE_ANY_FLOAT:
        return (any.fValue,ControlPortClient.TYPE_FLOAT)
    elif any.type == remotecontrolportapi_pb2.Any.TYPE_ANY_DOUBLE:
        return (any.dValue,ControlPortClient.TYPE_DOUBLE)
    elif any.type == remotecontrolportapi_pb2.Any.TYPE_ANY_STRING:
        return (any.sValue,ControlPortClient.TYPE_STRING)
    elif any.type == remotecontrolportapi_pb2.Any.TYPE_ANY_BOOLEAN:
        return (any.bValue,ControlPortClient.TYPE_BOOLEAN)
    elif any.type == remotecontrolportapi_pb2.Any.TYPE_ANY_INETADDR:
        return (any.sValue,ControlPortClient.TYPE_INETADDR)

def toAny(any,value,valueType):

    if valueType == ControlPortClient.TYPE_INT8:
        any.type = remotecontrolportapi_pb2.Any.TYPE_ANY_INT8
        any.i32Value = value

    elif valueType == ControlPortClient.TYPE_UINT8:
        any.type = remotecontrolportapi_pb2.Any.TYPE_ANY_UINT8
        any.u32Value = value

    elif valueType == ControlPortClient.TYPE_INT16:
        any.type = remotecontrolportapi_pb2.Any.TYPE_ANY_INT16
        any.i32Value = value

    elif valueType == ControlPortClient.TYPE_UINT16:
        any.type = remotecontrolportapi_pb2.Any.TYPE_ANY_UINT16
        any.u32Value = value

    elif valueType == ControlPortClient.TYPE_INT32:
        any.type = remotecontrolportapi_pb2.Any.TYPE_ANY_INT32
        any.i32Value = value

    elif valueType == ControlPortClient.TYPE_UINT32:
        any.type = remotecontrolportapi_pb2.Any.TYPE_ANY_UINT32
        any.u32Value = value

    elif valueType == ControlPortClient.TYPE_INT64:
        any.type = remotecontrolportapi_pb2.Any.TYPE_ANY_INT64
        any.i64Value = value

    elif valueType == ControlPortClient.TYPE_UINT64:
        any.type = remotecontrolportapi_pb2.Any.TYPE_ANY_UINT64
        any.u64Value = value

    elif valueType == ControlPortClient.TYPE_FLOAT:
        any.type = remotecontrolportapi_pb2.Any.TYPE_ANY_FLOAT
        any.fValue = value

    elif valueType == ControlPortClient.TYPE_DOUBLE:
        any.type = remotecontrolportapi_pb2.Any.TYPE_ANY_DOUBLE
        any.dValue = value

    elif valueType == ControlPortClient.TYPE_STRING:
        any.type = remotecontrolportapi_pb2.Any.TYPE_ANY_STRING
        any.sValue = value

    elif valueType == ControlPortClient.TYPE_BOOLEAN:
        any.type = remotecontrolportapi_pb2.Any.TYPE_ANY_BOOLEAN
        any.bValue = value

    elif valueType == ControlPortClient.TYPE_INETADDR:
        any.type = remotecontrolportapi_pb2.Any.TYPE_ANY_INETADDR
        any.sValue = value

    return any

class ControlPortClient:
    TYPE_INT8 = 1
    TYPE_UINT8 = 2
    TYPE_INT16 = 3
    TYPE_UINT16 = 4
    TYPE_INT32 = 5
    TYPE_UINT32 = 6
    TYPE_INT64 = 7
    TYPE_UINT64 = 8
    TYPE_FLOAT = 9
    TYPE_DOUBLE = 10
    TYPE_STRING = 11
    TYPE_BOOLEAN = 12
    TYPE_INETADDR = 13

    def __init__(self,hostname,port,disconnect=None):
        self._disconnect = disconnect
        self._buffer = None
        self._messageLengthBytes = 0
        self._sock =  socket.socket()  
        self._connected = True
        
        try:
            self._sock.connect((hostname,port))
        except:
            raise ControlPortException("unable to connect to %s:%d" % (hostname,port),True)

        self._eventMap = {}
        self._responseMap = {}
        self._sequence = 0
        self._lock = threading.Lock()
        self._read,self._write = os.pipe()
        self._thread = threading.Thread(target=self._run)
        self._thread.start() 

    def start(self):
        self._thread.start() 

    def stop(self):
        os.write(self._write,"\n")
        self._thread.join()

    def getManifest(self):
        request = remotecontrolportapi_pb2.Request()
        request.type = remotecontrolportapi_pb2.Request.TYPE_REQUEST_QUERY
        request.query.type = remotecontrolportapi_pb2.TYPE_QUERY_MANIFEST
        
        response = self._sendMessage(request)

        if response.type == remotecontrolportapi_pb2.Response.TYPE_RESPONSE_QUERY:
            if response.query.type == remotecontrolportapi_pb2.TYPE_QUERY_MANIFEST:
                manifest = {}
                for nem in response.query.manifest.nems:
                    layers = []
                    for component in nem.components:
                        if component.type == remotecontrolportapi_pb2.Response.Query.Manifest.NEM.Component.TYPE_COMPONENT_PHY:
                            name = 'PHY'
                        elif component.type == remotecontrolportapi_pb2.Response.Query.Manifest.NEM.Component.TYPE_COMPONENT_MAC:
                            name = 'MAC'
                        elif component.type == remotecontrolportapi_pb2.Response.Query.Manifest.NEM.Component.TYPE_COMPONENT_SHIM:
                            name = 'SHIM'
                        else:
                            name = 'TRANSPORT'

                        layers.append((component.buildId,name,component.plugin))

                    manifest[nem.id] = tuple(layers)

                return manifest
            else:
                raise ControlPortException('malformed manifest query response')
        else:
            raise ControlPortException('malformed query response')


    def getConfiguration(self,buildId,parameters = ()):
        request = remotecontrolportapi_pb2.Request()
        request.type = remotecontrolportapi_pb2.Request.TYPE_REQUEST_QUERY
        request.query.type = remotecontrolportapi_pb2.TYPE_QUERY_CONFIGURATION
        request.query.configuration.buildId = buildId
        for parameter in parameters:
            request.query.configuration.names.append(parameter)

        response = self._sendMessage(request)

        if response.type == remotecontrolportapi_pb2.Response.TYPE_RESPONSE_QUERY:
            if response.query.type == remotecontrolportapi_pb2.TYPE_QUERY_CONFIGURATION:
                configuration = {}
                for parameter in response.query.configuration.parameters:
                    values = []

                    for value in parameter.values:
                        values.append(fromAny(value))

                    configuration[parameter.name] = tuple(values)

                return configuration
            else:
                raise ControlPortException('malformed configuration query response')
        else:
            raise ControlPortException('malformed query response')


    def getStatistic(self,buildId,elements = ()):
        request = remotecontrolportapi_pb2.Request()
        request.type = remotecontrolportapi_pb2.Request.TYPE_REQUEST_QUERY
        request.query.type = remotecontrolportapi_pb2.TYPE_QUERY_STATISTIC
        request.query.statistic.buildId = buildId
        for element in elements:
            request.query.statistic.names.append(element)

        response = self._sendMessage(request)

        if response.type == remotecontrolportapi_pb2.Response.TYPE_RESPONSE_QUERY:
            if response.query.type == remotecontrolportapi_pb2.TYPE_QUERY_STATISTIC:
                statistics = {}
                for element in response.query.statistic.elements:
                    statistics[element.name] = fromAny(element.value)

                return statistics

            else:
                raise ControlPortException('malformed statistic query response')
        else:
            raise ControlPortException('malformed query response')


    def getStatisticTable(self,buildId,tables = ()):
        request = remotecontrolportapi_pb2.Request()
        request.type = remotecontrolportapi_pb2.Request.TYPE_REQUEST_QUERY
        request.query.type = remotecontrolportapi_pb2.TYPE_QUERY_STATISTICTABLE
        request.query.statisticTable.buildId = buildId
        for table in tables:
            request.query.statisticTable.names.append(table)

        response = self._sendMessage(request)

        if response.type == remotecontrolportapi_pb2.Response.TYPE_RESPONSE_QUERY:
            if response.query.type == remotecontrolportapi_pb2.TYPE_QUERY_STATISTICTABLE:
                tables = {}

                for table in response.query.statisticTable.tables:
                    tableData = []
                    for row in table.rows:
                        rowData = []
                        for value in row.values:
                            rowData.append(fromAny(value))
                        tableData.append(tuple(rowData))
                    tables[table.name] = (tuple(table.labels),tuple(tableData))

                return tables
            else:
                raise ControlPortException('malformed statisticTable query response')
        else:
            raise ControlPortException('malformed query response')


    def clearStatistic(self,buildId,elements = ()):
        request = remotecontrolportapi_pb2.Request()
        request.type = remotecontrolportapi_pb2.Request.TYPE_REQUEST_UPDATE
        request.update.type = remotecontrolportapi_pb2.TYPE_UPDATE_STATISTICCLEAR
        request.update.statisticClear.buildId = buildId
        for element in elements:
            request.update.statisticClear.names.append(element)

        response = self._sendMessage(request)

        if response.type == remotecontrolportapi_pb2.Response.TYPE_RESPONSE_UPDATE:
            if response.update.type == remotecontrolportapi_pb2.TYPE_UPDATE_STATISTICCLEAR:
                    return
            else:
                raise ControlPortException('malformed statistic clear update response')
        else:
            raise ControlPortException('malformed update response')


    def clearTable(self,buildId,tables = ()):
        request = remotecontrolportapi_pb2.Request()
        request.type = remotecontrolportapi_pb2.Request.TYPE_REQUEST_UPDATE
        request.update.type = remotecontrolportapi_pb2.TYPE_UPDATE_STATISTICTABLECLEAR
        request.update.statisticTableClear.buildId = buildId
        for table in tables:
            request.update.statisticTableClear.names.append(table)

        response = self._sendMessage(request)

        if response.type == remotecontrolportapi_pb2.Response.TYPE_RESPONSE_UPDATE:
            if response.update.type == remotecontrolportapi_pb2.TYPE_UPDATE_STATISTICTABLECLEAR:
                return
            else:
                raise ControlPortException('malformed statistic table clear update response')
        else:
            raise ControlPortException('malformed update response')


    def updateConfiguration(self,buildId,updates):
        request = remotecontrolportapi_pb2.Request()
        request.type = remotecontrolportapi_pb2.Request.TYPE_REQUEST_UPDATE
        request.update.type = remotecontrolportapi_pb2.TYPE_UPDATE_CONFIGURATION
        request.update.configuration.buildId = buildId
        
        for (name,dataType,values) in updates:
            parameter = request.update.configuration.parameters.add()
            parameter.name = name
            for value in values:
                any = parameter.values.add()
                toAny(any,value,dataType)
            
        response = self._sendMessage(request)

        if response.type == remotecontrolportapi_pb2.Response.TYPE_RESPONSE_UPDATE:
            if response.update.type == remotecontrolportapi_pb2.TYPE_UPDATE_CONFIGURATION:
                return
            else:
                raise ControlPortException('malformed configuration update response')
        else:
            raise ControlPortException('malformed update response')


    def setLogLevel(self,level):
        request = remotecontrolportapi_pb2.Request()
        request.type = remotecontrolportapi_pb2.Request.TYPE_REQUEST_UPDATE
        request.update.type = remotecontrolportapi_pb2.TYPE_UPDATE_LOGLEVEL
        request.update.logLevel.level = level

        response = self._sendMessage(request)

        if response.type == remotecontrolportapi_pb2.Response.TYPE_RESPONSE_UPDATE:
            if response.update.type == remotecontrolportapi_pb2.TYPE_UPDATE_LOGLEVEL:
                return
            else:
                raise ControlPortException('malformed log level update response')
        else:
            raise ControlPortException('malformed update response')



    def _sendMessage(self,request):
        self._lock.acquire()

        if not self._connected:
            self._lock.release()
            raise ControlPortException('connection terminated by server',True)

        sequence = ++self._sequence

        request.sequence = sequence

        msg = request.SerializeToString()

        self._sock.send(struct.pack("!L%ds" % len(msg),len(msg),msg))

        event = threading.Event()

        self._eventMap[sequence] = (event)

        self._lock.release()

        event.wait()

        self._lock.acquire()

        if self._connected:
            response =  self._responseMap[sequence]
        
            del self._responseMap[sequence]

            del self._eventMap[sequence]

            self._lock.release()

            if response.type == remotecontrolportapi_pb2.Response.TYPE_RESPONSE_ERROR:
                raise ControlPortException(response.error.description);

            return response
        else:
            self._lock.release()
            raise ControlPortException('connection terminated by server',True)


    def _run(self):
        buffer = ""
        messageLengthBytes = 0
        running = True

        while running:
            readable,_,_ = select.select([self._sock,self._read],[],[])
    
            for fd in readable:
                if fd is self._sock:
                    if not messageLengthBytes:
                        data = self._sock.recv(4-len(buffer))

                        if not len(data):
                            running = False
                            break

                        buffer+=data

                        if(len(buffer) == 4):
                            (messageLengthBytes,) = struct.unpack('!I',buffer);
                            buffer = ""

                    else:
                        data = self._sock.recv(messageLengthBytes-len(buffer))

                        if not len(data):
                            running = False
                            break

                        buffer+=data

                        if(len(buffer) == messageLengthBytes):
                            response = remotecontrolportapi_pb2.Response()
                            
                            response.ParseFromString(buffer)

                            self._lock.acquire()

                            if response.reference in self._eventMap:
                                self._responseMap[response.reference] = response
                                self._eventMap[response.reference].set()
                                
                            self._lock.release()

                            messageLengthBytes = 0
                            buffer = ""

                elif fd is self._read:
                    running = False
                    break

        self._lock.acquire()

        self._connected = False

        for event in self._eventMap.values():
            event.set()

        self._lock.release()

        if self._disconnect:
            self._disconnect()
                    
