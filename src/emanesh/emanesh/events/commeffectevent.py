#
# Copyright (c) 2013-2014,2017 - Adjacent Link LLC, Bridgewater,
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

from . import Event
from . import commeffectevent_pb2

class CommEffectEvent(Event):
    IDENTIFIER = 103

    def __init__(self):
        self._event = commeffectevent_pb2.CommEffectEvent()

    def append(self,nemId,**kwargs):
        commeffect = self._event.commEffects.add()

        commeffect.nemId = nemId

        for (name,value) in list(kwargs.items()):

            if name == 'latency':
                if (isinstance(value,int) or \
                    isinstance(value,float)) and \
                value >= 0:
                    commeffect.latencySeconds = value
                else:
                    raise ValueError("latency must be a positive numeric")

            elif name == 'jitter':
                if (isinstance(value,int) or \
                    isinstance(value,float)) and \
                    value >= 0:
                    commeffect.jitterSeconds = value
                else:
                    raise ValueError("jitter must be a positive numeric")

            elif name == 'loss':
                if (isinstance(value,int) or \
                    isinstance(value,float)) and \
                    value >= 0:
                    commeffect.probabilityLoss = value
                else:
                    raise ValueError("loss must be numeric")

            elif name == 'duplicate':
                if (isinstance(value,int) or \
                    isinstance(value,float)) and \
                    value >= 0:
                    commeffect.probabilityDuplicate = value
                else:
                    raise ValueError("duplicate must be a positive numeric")

            elif name == 'unicast':
                if (isinstance(value,int) or isinstance(value,int)) and \
                   value >= 0:
                    commeffect.unicastBitRatebps = value
                else:
                    raise ValueError("unicast must be a positive integer")

            elif name == 'broadcast':
                if (isinstance(value,int) or isinstance(value,int)) and \
                   value >= 0:
                    commeffect.broadcastBitRatebps = value
                else:
                    raise ValueError("broadcast must be a positive integer")

            else:
                raise KeyError("unknown parameter: %s" % name)


    def serialize(self):
        return self._event.SerializeToString()

    def restore(self,data):
        self._event.ParseFromString(data)

    def __iter__(self):
        for commeffect in self._event.commEffects:
            kwargs = {'latency': commeffect.latencySeconds,
                      'jitter' : commeffect.jitterSeconds,
                      'loss' : commeffect.probabilityLoss,
                      'duplicate' : commeffect.probabilityDuplicate,
                      'unicast' : commeffect.unicastBitRatebps,
                      'broadcast' : commeffect.broadcastBitRatebps}

            yield (commeffect.nemId,kwargs)
