#
# Copyright (c) 2017 - Adjacent Link LLC, Bridgewater, New Jersey
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
from . import fadingselectionevent_pb2

class FadingSelectionEvent(Event):
    IDENTIFIER = 106

    MODELS = ('none','nakagami','lognormal')

    def __init__(self):
        self._event = fadingselectionevent_pb2.FadingSelectionEvent()

    def append(self,nemId,**kwargs):
        hasModel = False
        entry = self._event.entries.add()

        entry.nemId = nemId

        for (name,value) in list(kwargs.items()):

            if name == 'model':
                if value == 'none':
                    entry.model  = fadingselectionevent_pb2.FadingSelectionEvent.TYPE_NONE
                    hasModel = True
                elif value == 'nakagami':
                    entry.model  = fadingselectionevent_pb2.FadingSelectionEvent.TYPE_NAKAGAMI
                    hasModel = True
                elif value == 'lognormal':
                    entry.model  = fadingselectionevent_pb2.FadingSelectionEvent.TYPE_LOGNORMAL
                    hasModel = True
                else:
                    raise ValueError("unknown model")
            else:
                raise KeyError("unknown parameter: %s" % name)

        if not hasModel:
            raise KeyError("must specify model")

    def serialize(self):
        return self._event.SerializeToString()

    def restore(self,data):
        self._event.ParseFromString(data)

    def __iter__(self):
        for entry in self._event.entries:
            model = ""

            if entry.model == fadingselectionevent_pb2.FadingSelectionEvent.TYPE_NONE:
                model = 'none'
            elif entry.model == fadingselectionevent_pb2.FadingSelectionEvent.TYPE_NAKAGAMI:
                model = 'nakagami'
            elif entry.model == fadingselectionevent_pb2.FadingSelectionEvent.TYPE_LOGNORMAL:
                model = 'lognormal'

            kwargs = {'model' : model}

            yield (entry.nemId,kwargs)
