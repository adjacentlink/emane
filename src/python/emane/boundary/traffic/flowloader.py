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

import logging
import regex

from .periodicflow  import PeriodicFlow

class FlowLoader(object):
    PATTERN_FLOW_ON = \
        regex.compile(r'(?P<start>\d+(?:.\d+)?)\s+ON\s+(?P<flow>\d+)'
                      r'\s+SRC\s+(?P<src>[^\s]+)\s+DST\s+(?P<dst>[^\s]+)'
                      r'\s+PRI\s+(?P<pri>\d+)\s+(PERIODIC)\s+\[\s*'
                      r'(?P<rate>\d*(?:.\d+)?)\s+(?P<length>\d+)\s*\]'
                      r'(\s+TTL\s+(?P<ttl>\d+))?\s*$')

    PATTERN_FLOW_OFF = \
        regex.compile(r'(?P<off>\d+(?:.\d+)?)\s+OFF\s+(?P<flow>\d+)\s*$')

    PATTERN_STOP = \
        regex.compile(r'(?P<stop>\d+(?:.\d+)?)\s+STOP\s*$')

    PATTERN_BLANK_OR_COMMENT = \
        regex.compile(r'^\s*(#.*)?$')

    @staticmethod
    def load(scenario_file):
        line_count = 0
        flows = {}
        stop_time = None

        with open(scenario_file,'r') as fd:
            for line in fd:
                line = line.strip()

                line_count += 1

                m = FlowLoader.PATTERN_BLANK_OR_COMMENT.match(line)

                if m:
                    continue

                m = FlowLoader.PATTERN_FLOW_ON.match(line)

                if m:
                    flow = int(m.group('flow'))

                    if flow not in flows:
                        ttl = m.group('ttl')

                        if ttl is None:
                            ttl = 64
                        else:
                            ttl = int(ttl)

                        flows[flow] = PeriodicFlow(flow,
                                                   float(m.group('start')),
                                                   m.group('src'),
                                                   m.group('dst'),
                                                   int(m.group('pri')),
                                                   ttl,
                                                   float(m.group('rate')),
                                                   int(m.group('length')))
                    else:
                        logging.error("error flow redefined line {}: {}".format(line_count,line))
                        raise RuntimeError("error syntax line {}: {}".format(line_count,line))


                else:
                    m = FlowLoader.PATTERN_FLOW_OFF.match(line)

                    if m:
                        flow = int(m.group('flow'))

                        if flow in flows:
                            flows[flow].set_off(float(m.group('off')))
                        else:
                            logging.error("error flow undefined line {}: {}".format(line_count,line))
                            raise RuntimeError("error syntax line {}: {}".format(line_count,line))

                    else:
                        m = FlowLoader.PATTERN_STOP.match(line)

                        if m:
                            stop_time = float(m.group('stop'))
                        else:
                            logging.error("error syntax line {}: {}".format(line_count,line))
                            raise RuntimeError("error syntax line {}: {}".format(line_count,line))

            return flows,stop_time
