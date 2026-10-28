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

from collections import namedtuple

class Flow(object):
    Info = namedtuple('Info',
                      ['id',
                       'source',
                       'destination',
                       'priority',
                       'ttl',
                       'length'])

    def __init__(self,
                 flow_id,
                 realtive_start_seconds,
                 source,
                 destnation,
                 priority,
                 ttl,
                 length):
        self._realtive_start_seconds = realtive_start_seconds
        self._id = flow_id
        self._source = source
        self._destination = destnation
        self._priority = priority
        self._ttl = ttl
        self._length = length
        self._start_time_seconds = 0
        self._relative_off_seconds = None
        self._generated_count = 0

    def set_off(self,relative_off_seconds):
        self._relative_off_seconds = relative_off_seconds

    def info(self):
        return Flow.Info(self._id,
                         self._source,
                         self._destination,
                         self._priority,
                         self._ttl,
                         self._length)

    def id(self):
        return self._id
