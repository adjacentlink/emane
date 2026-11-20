# Copyright (c) 2025-2026 - Adjacent Link LLC, Bridgewater, New Jersey
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
# See toplevel COPYING for more information.
#

import zmq
import os
import logging
import ctypes
import struct
import secrets
import sys
import traceback

from collections import defaultdict

from threading import Thread

from .flowloader import FlowLoader
from .flowmanager import FlowManager

from simactivityservice.interface import Topics
import simactivityservice.interface.simactivityservice_pb2 as simactivityservice_pb2

import emane.boundary.interface.traffic_pb2 as traffic_pb2

class Service(object):
    def __init__(self,
                 scenario_file,
                 traffic_endpoint,
                 activity_service_subscribe_endpoint,
                 activity_service_publish_endpoint,
                 activity_service_tag,
                 local_id,
                 out_file,
                 action_name,
                 uuid,
                 stop_callback):

        logging.info('scenario file: {}'.format(scenario_file))

        logging.info('traffic endpoint: {}'.
                     format(traffic_endpoint))

        logging.info('activity service subscribe endpoint: {}'.
                     format(activity_service_subscribe_endpoint))

        logging.info('activity service publish endpoint: {}'.
                     format(activity_service_publish_endpoint))

        logging.info('activity service tag: {}'.format(activity_service_tag))

        logging.info('out file: {}'.format(out_file))

        if out_file:
            self._ofd = open(out_file,"w")
        else:
            self._ofd = sys.stdout

        self._thread = None

        self._libc = ctypes.CDLL(None)

        self._eventfd = \
            self._libc.eventfd(ctypes.c_uint(0),
                               ctypes.c_uint(0))

        self._local_id = local_id

        self._action_name = action_name

        self._context = zmq.Context()

        self._activity_publisher = self._context.socket(zmq.PUB)

        self._activity_publisher.setsockopt(zmq.IPV6,1)

        self._activity_publisher.bind('tcp://{}'.format(activity_service_publish_endpoint))

        self._activity_subscriber = self._context.socket(zmq.SUB)

        self._activity_subscriber.setsockopt(zmq.IPV6,1)

        self._activity_subscriber.connect('tcp://{}'.format(activity_service_subscribe_endpoint))

        self._activity_subscriber.setsockopt(zmq.SUBSCRIBE,
                                             Topics.TOPIC_TIME_NOTICE)

        self._activity_subscriber.setsockopt(zmq.SUBSCRIBE,
                                             Topics.TOPIC_TIME_NOW)

        self._activity_subscriber.setsockopt(zmq.SUBSCRIBE,
                                             Topics.TOPIC_TIME_SIGNAL)

        self._activity_subscriber.setsockopt(zmq.SUBSCRIBE,
                                             Topics.TOPIC_HELLO_ANNOUNCE)

        self._stream = self._context.socket(zmq.STREAM)

        self._stream.setsockopt(zmq.IPV6,1)

        self._stream.connect('tcp://{}'.format(traffic_endpoint))

        self._identity = self._stream.getsockopt(zmq.IDENTITY)

        self._connected = False

        flows, stop_time = FlowLoader.load(scenario_file)

        self._stop_time_microseconds = int(stop_time * 1000000)

        self._flow_manager = FlowManager(flows)

        self._flow_seqeunce = defaultdict(lambda : 0)

        self._activity_service_tag = activity_service_tag

        self._uuid = uuid

        self._simulation_time_now_microseconds = 0

        self._flow_seqeunce = defaultdict(lambda : 0)

        self._stop_callback = stop_callback

        self._next_time = None

        self._sequence_map = defaultdict(lambda : 0)

        self._sequence_number = 0

        self._last_status_time = -1

    def _send_hello(self):
        activity = simactivityservice_pb2.Activity()

        activity.uuid = self._uuid.bytes

        activity.type = simactivityservice_pb2.Activity.TYPE_ACTIVITY_HELLO

        activity.tag = self._activity_service_tag

        activity.time_microseconds_since_epoch = self._simulation_time_now_microseconds

        activity.sequence_number = self._sequence_number

        logging.debug('sending hello sequence: {}'.format(self._sequence_number))

        self._sequence_number += 1

        self._activity_publisher.send_multipart([Topics.TOPIC_HELLO_ACKNOWLEDGE,
                                                 activity.SerializeToString()])

    def _send_time_confirm_notice(self,time_confirm,ready,reference_sequence_number):
        activity = simactivityservice_pb2.Activity()

        activity.uuid = self._uuid.bytes

        activity.type = simactivityservice_pb2.Activity.TYPE_ACTIVITY_TIME_CONFIRM_NOTICE

        activity.tag = self._activity_service_tag

        activity.sequence_number = self._sequence_number

        activity.time_microseconds_since_epoch = self._simulation_time_now_microseconds

        activity.time_confirm_notice.time_microseconds_since_epoch = time_confirm

        activity.time_confirm_notice.ready = ready

        activity.time_confirm_notice.reference_sequence_number = reference_sequence_number

        logging.debug('sending time confirm notice:'
                      ' now: {}'
                      ' sequence: {}'
                      ' confirm: {}'
                      ' ready: {}'
                      ' reference: {}'.format(self._simulation_time_now_microseconds,
                                              self._sequence_number,
                                              time_confirm,
                                              ready,
                                              reference_sequence_number))

        self._sequence_number += 1

        self._activity_publisher.send_multipart([Topics.TOPIC_TIME_CONFIRM_NOTICE,
                                                 activity.SerializeToString()])

    def _send_time_confirm_now(self,time_confirm,ready,reference_sequence_number):
        activity = simactivityservice_pb2.Activity()

        activity.uuid = self._uuid.bytes

        activity.type = simactivityservice_pb2.Activity.TYPE_ACTIVITY_TIME_CONFIRM_NOW

        activity.tag = self._activity_service_tag

        activity.sequence_number = self._sequence_number

        activity.time_microseconds_since_epoch = self._simulation_time_now_microseconds

        activity.time_confirm_now.time_microseconds_since_epoch = time_confirm

        activity.time_confirm_now.ready = ready

        activity.time_confirm_now.reference_sequence_number = reference_sequence_number

        logging.debug('sending time confirm now'
                      ' now: {}'
                      ' sequence: {}'
                      ' confirm: {}'
                      ' ready: {}'
                      ' reference: {}'.format(self._simulation_time_now_microseconds,
                                              self._sequence_number,
                                              time_confirm,
                                              ready,
                                              reference_sequence_number))

        self._sequence_number += 1

        self._activity_publisher.send_multipart([Topics.TOPIC_TIME_CONFIRM_NOW,
                                                 activity.SerializeToString()])

    def _send_status(self,earliest_next_time):
        activity = simactivityservice_pb2.Activity()

        activity.uuid = self._uuid.bytes

        activity.type = simactivityservice_pb2.Activity.TYPE_ACTIVITY_STATUS

        activity.tag = self._activity_service_tag

        activity.sequence_number = self._sequence_number

        activity.time_microseconds_since_epoch = self._simulation_time_now_microseconds

        activity.status.next_time_microseconds_since_epoch = \
            0 if earliest_next_time is None else earliest_next_time

        logging.debug("sending status time: {} sequence: {}".
                      format(activity.status.next_time_microseconds_since_epoch,
                             self._sequence_number))

        activity.SerializeToString()

        self._sequence_number += 1

        self._last_status_time = self._simulation_time_now_microseconds

        self._activity_publisher.send_multipart([Topics.TOPIC_STATUS,
                                                 activity.SerializeToString()])


    def _send_activity_action(self,source,destination,sequence_number):
        activity = simactivityservice_pb2.Activity()

        activity.uuid = self._uuid.bytes

        activity.type = simactivityservice_pb2.Activity.TYPE_ACTIVITY_ACTION

        activity.tag = self._activity_service_tag

        activity.time_microseconds_since_epoch = self._simulation_time_now_microseconds

        activity.sequence_number = self._sequence_number

        activity.action.name='{}.{}'.format(self._action_name,self._local_id)

        activity.action.unique_identifier='{}-{}-{}-{}'.format(source,
                                                               destination,
                                                               self._local_id,
                                                               sequence_number)

        logging.debug("sending activity action: {} indentifier: {} now:{}".
                      format(activity.action.name,
                             activity.action.unique_identifier,
                             activity.time_microseconds_since_epoch))

        activity.SerializeToString()

        self._sequence_number += 1

        self._activity_publisher.send_multipart([Topics.TOPIC_ACTION,
                                                 activity.SerializeToString()])


    def _send_data(self,flows):
        if self._identity is not None:
            for flow in flows:
                flow_info = flow.info()
                msg = traffic_pb2.Traffic()
                msg.type = traffic_pb2.Traffic.TYPE_DOWNSTREAM
                msg.source = flow_info.source
                msg.destination = flow_info.destination
                msg.priority =flow_info.priority
                msg.sequence_number = self._sequence_map[flow_info.source]
                msg.ttl = flow_info.ttl

                self._sequence_map[flow_info.source] += 1

                header = struct.pack('!HQQ',
                                     flow_info.id,
                                     self._flow_seqeunce[flow_info.id],
                                     self._simulation_time_now_microseconds)



                if flow_info.length < len(header):
                    msg.payload = header
                else:
                    msg.payload = header + secrets.token_bytes(flow_info.length - len(header))

                if not self._flow_seqeunce[flow_info.id]:
                    print('{}.{:06d} ON flow>{} local>{} src>{} dst>{}'.format(self._simulation_time_now_microseconds // 1000000,
                                                                               self._simulation_time_now_microseconds % 1000000,
                                                                               flow_info.id,
                                                                               self._local_id,
                                                                               msg.source,
                                                                               msg.destination),
                          file=self._ofd,
                          flush=True)


                print('{}.{:06d}'
                      ' SEND'
                      ' flow>{}'
                      ' seq>{}'
                      ' local>{}'
                      ' src>{}'
                      ' dst>{}'
                      ' prio>{}'
                      ' ttl>{}'
                      ' size>{}'.format(self._simulation_time_now_microseconds // 1000000,
                                        self._simulation_time_now_microseconds % 1000000,
                                        flow_info.id,
                                        self._flow_seqeunce[flow_info.id],
                                        self._local_id,
                                        msg.source,
                                        msg.destination,
                                        msg.priority,
                                        msg.ttl,
                                        len(msg.payload)),
                      file=self._ofd,
                      flush=True)

                self._flow_seqeunce[flow_info.id] += 1

                serialization = msg.SerializeToString()

                logging.debug('sending data serialization length: {}'.format(len(serialization)))


                self._stream.send_multipart([self._identity,
                                             struct.pack('!I{}s'.format(len(serialization)),
                                                         len(serialization),
                                                         serialization)])

                self._send_activity_action(msg.source,msg.destination,msg.sequence_number)



    def start(self):
        logging.info('starting')
        self._thread = Thread(target=self.__process)
        self._thread.start()

    def __process(self):
        logging.debug("process start")

        try:
            simulation_time_start_microseconds = None

            timeout_milliseconds = 1000

            poller = zmq.Poller()

            poller.register(self._activity_subscriber,zmq.POLLIN)
            poller.register(self._eventfd,zmq.POLLIN)
            poller.register(self._stream,zmq.POLLIN)

            stream_buffer = bytes()
            pending_message_length = 0

            while True:
                socks = dict(poller.poll(None))

                if self._eventfd in socks and \
                   socks[self._eventfd] == zmq.POLLIN:
                    break

                elif self._stream in socks and \
                    socks[self._stream] == zmq.POLLIN:

                    msgs = self._stream.recv_multipart()

                    identity = msgs[0]

                    if not len(msgs[1]):

                        if self._connected:
                            logging.info("traffic connection terminated")
                            self._connected = False
                        else:
                            logging.info("traffic connection established")
                            self._connected = True

                    else:
                        stream_buffer += msgs[1]

                        while True:
                            if not pending_message_length:
                                if len(stream_buffer) < 4:
                                    # wait for more data find length prefix frame
                                    break

                                else:
                                    (pending_message_length,) = struct.unpack('!I',stream_buffer[0:4])

                                    stream_buffer = stream_buffer[4:]

                            else:
                                if len(stream_buffer) >= pending_message_length:

                                    traffic = traffic_pb2.Traffic()

                                    traffic.ParseFromString(stream_buffer[0:pending_message_length])

                                    if traffic.type == traffic_pb2.Traffic.TYPE_UPSTREAM:

                                        (flow_id,
                                         sequence_number,
                                         sent_microseconds) = struct.unpack('!HQQ',
                                                                            traffic.payload[:struct.calcsize('!HQQ')])

                                        print('{}.{:06d}'
                                              ' RECV'
                                              ' flow>{}'
                                              ' seq>{}'
                                              ' local>{}'
                                              ' src>{}'
                                              ' dst>{}'
                                              ' prio>{}'
                                              ' ttl>{}'
                                              ' size>{}'
                                              ' sent>{}.{:06d}'.format(self._simulation_time_now_microseconds // 1000000,
                                                                       self._simulation_time_now_microseconds % 1000000,
                                                                       flow_id,
                                                                       sequence_number,
                                                                       traffic.local,
                                                                       traffic.source,
                                                                       traffic.destination,
                                                                       traffic.priority,
                                                                       traffic.ttl,
                                                                       len(traffic.payload),
                                                                       sent_microseconds // 1000000,
                                                                       sent_microseconds % 1000000),
                                              file=self._ofd,
                                              flush=True)

                                        self._send_activity_action(traffic.source,
                                                                   traffic.destination,
                                                                   traffic.sequence_number)

                                        # if simulation now == next
                                        # time that means we are
                                        # receiving before a scheduled
                                        # tx flow send, so don't send
                                        # status
                                        if self._simulation_time_now_microseconds != self._next_time:
                                            logging.debug('sending status after rx')
                                            self._send_status(self._next_time)

                                    stream_buffer = stream_buffer[pending_message_length:]

                                    pending_message_length = 0

                                else:
                                    # wait for more data to find message
                                    break

                elif self._activity_subscriber in socks and \
                    socks[self._activity_subscriber] == zmq.POLLIN:

                    msgs = self._activity_subscriber.recv_multipart()

                    logging.debug('received {}'.format(msgs[0]))

                    if msgs[0] == Topics.TOPIC_HELLO_ANNOUNCE:
                        self._send_hello()

                    elif msgs[0] == Topics.TOPIC_TIME_NOTICE:
                        activity = simactivityservice_pb2.Activity()

                        activity.ParseFromString(msgs[1])

                        if activity.type == simactivityservice_pb2.Activity.TYPE_ACTIVITY_TIME_NOTICE:
                            timeout_milliseconds = None

                            if activity.HasField('time_notice'):
                                logging.debug('received {} now: {}'.
                                              format(msgs[0],
                                                     activity.time_notice.time_microseconds_since_epoch))

                                self._send_time_confirm_notice(activity.time_notice.time_microseconds_since_epoch,
                                                               True,
                                                               activity.sequence_number)
                            else:
                                logging.error("malformed time notice message")

                    elif  msgs[0] == Topics.TOPIC_TIME_NOW:
                        activity = simactivityservice_pb2.Activity()

                        activity.ParseFromString(msgs[1])

                        if activity.type == simactivityservice_pb2.Activity.TYPE_ACTIVITY_TIME_NOW:

                            if activity.HasField('time_now'):
                                self._simulation_time_now_microseconds = \
                                    activity.time_now.time_microseconds_since_epoch

                                if simulation_time_start_microseconds is None:
                                    simulation_time_start_microseconds = \
                                        self._simulation_time_now_microseconds

                                    if self._stop_time_microseconds is not None:
                                        self._stop_time_microseconds += simulation_time_start_microseconds

                                logging.debug('received {} now: {}'.
                                              format(msgs[0],
                                                     self._simulation_time_now_microseconds))

                                self._send_time_confirm_now(self._simulation_time_now_microseconds,
                                                            True,
                                                            activity.sequence_number)
                            else:
                                logging.error("malformed time now message")

                    elif  msgs[0] == Topics.TOPIC_TIME_SIGNAL:
                        activity = simactivityservice_pb2.Activity()

                        activity.ParseFromString(msgs[1])

                        if activity.type == simactivityservice_pb2.Activity.TYPE_ACTIVITY_TIME_SIGNAL:

                            if self._stop_time_microseconds is not None:
                                if self._simulation_time_now_microseconds == \
                                   self._stop_time_microseconds:
                                    print('{}.{:06d} STOP'.format(self._simulation_time_now_microseconds  // 1000000,
                                                                  self._simulation_time_now_microseconds  % 1000000),
                                          file=self._ofd,
                                          flush=True)

                                    if self._stop_callback is not None:
                                        self._stop_callback()

                                    break

                            ready,next_time = \
                                self._flow_manager.ready(self._simulation_time_now_microseconds -
                                                         simulation_time_start_microseconds)

                            logging.debug('next time is {}'.format(next_time))

                            self._next_time = next_time

                            if ready:
                                logging.debug('ready flows!')
                                self._send_data(ready)

                            if next_time is None:
                                if self._stop_time_microseconds is not None:
                                    next_time = self._stop_time_microseconds + \
                                        simulation_time_start_microseconds
                            else:
                                if self._stop_time_microseconds is not None:
                                    stop_time = self._stop_time_microseconds + \
                                        simulation_time_start_microseconds
                                    if stop_time < next_time:
                                        next_time = stop_time

                            self._next_time = next_time

                            if ready or \
                               self._last_status_time != self._simulation_time_now_microseconds:
                                self._send_status(next_time)

                        else:
                            logging.error('unexpected activity type')
                    else:
                        logging.error('unexpected activity message')


        except:
            logging.error(traceback.format_exc())

    logging.debug("process fin")

    def stop(self):
        if self._thread is not None:
            os.write(self._eventfd,struct.pack('Q',1))
            self._thread.join()
            self._thread = None
