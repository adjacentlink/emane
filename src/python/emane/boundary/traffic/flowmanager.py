# Copyright (c) 2025 - Adjacent Link LLC, Bridgewater, New Jersey
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

import logging

class FlowManager:
    def __init__(self,flows):
        self._flows = flows
        self._next_flow_times = {}
        self._next()

    def _next(self):
        self._earliest_time = None

        for flow_id in self._flows:
            if flow_id not in self._next_flow_times:
                next_time = self._flows[flow_id].next()

                if next_time != None:
                    self._next_flow_times[flow_id] = next_time

        if self._next_flow_times:
            # get the earliest time
            for flow_id,next_time in self._next_flow_times.items():
                if next_time != None:
                    if self._earliest_time == None:
                        self._earliest_time =  next_time
                    else:
                        if self._earliest_time > next_time:
                            self._earliest_time = next_time

        return self._earliest_time

    def ready(self,now_time):
        logging.debug("ready time now: {} earliest {}".format(now_time,
                                                              self._earliest_time))

        ready = set()

        if self._earliest_time != None:
            if now_time > self._earliest_time:
                raise RuntimeError('now time {} > earliest time {}'.format(now_time,
                                                                           self._earliest_time))

            elif now_time == self._earliest_time:
                for flow_id,next_time in self._next_flow_times.items():
                    if next_time == now_time:
                        ready.add(self._flows[flow_id])

        for flow in ready:
            logging.debug('removing {} form next flow time'.format(flow.id()))
            del self._next_flow_times[flow.id()]

        self._next()

        return ready,self._earliest_time
