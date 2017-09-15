#
# Copyright (c) 2013,2017 - Adjacent Link LLC, Bridgewater, New Jersey
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

from pkg_resources import resource_filename
from lxml import etree
from . import ManifestException

def toBool(value):

    if value == "yes" or \
            value == "true":
        return True
    else:
        return False

class Manifest:
    def __init__(self,filename):
        self._statistics = {}
        self._tables = {}
        self._configuration = {}

        tree = etree.parse(filename)
        root = tree.getroot()

        schemaDoc = etree.parse(resource_filename('emanesh', 'schema/manifest.xsd'))

        schema = etree.XMLSchema(etree=schemaDoc,attribute_defaults=True)

        if not schema(root):
            message = ""
            for entry in schema.error_log:
                message += "%d: %s" % (entry.line,entry.message)
            raise ManifestException(message)

        self._name = root.xpath('/manifest/plugin/@name')[0]

        for parameter in root.xpath('/manifest/plugin/configuration/parameter'):
            entry = {}

            values = parameter.xpath('.//values')[0]

            description = ""

            descriptions = parameter.xpath('.//description')

            if descriptions:
                description = descriptions[0].text

            regex = ""

            regexes = parameter.xpath('.//regex')

            if regexes:
                regex = regexes[0].text

            entry = { 'default' : toBool(parameter.get('default')),
                      'required' : toBool(parameter.get('required')),
                      'modifiable' : toBool(parameter.get('modifiable')),
                      'description' : description,
                      'regex' : regex,
                      'minOccurs' : int(values.get("minOccurs")),
                      'maxOccurs' : int(values.get("maxOccurs")),
                      'values' : [x.text for x in  values.xpath('value')]}

            numeric = parameter.xpath('numeric')

            if numeric:
                numeric = numeric[0]

                entry['numeric'] = { 'type' : numeric.get('type'),
                                     'minValue' : numeric.get('minValue'),
                                     'maxValue' : numeric.get('maxValue') }
            else:
                nonnumeric = parameter.xpath('nonnumeric')[0]
                entry['nonnumeric'] = { 'type' : nonnumeric.get('type')}

            self._configuration[parameter.get('name')] = entry


        for element in root.xpath('/manifest/plugin/statistics/element'):
            description = ""

            descriptions = element.xpath('description')

            if descriptions:
                description = descriptions[0].text

            self._statistics[element.get('name')] = {'type' : element.get('type'),
                                                     'clearable': toBool(element.get('clearable')),
                                                     'description' : description}

        for table in root.xpath('/manifest/plugin/statistictables/table'):
            description = ""

            descriptions = table.xpath('description')

            if descriptions:
                description = descriptions[0].text

            self._tables[table.get('name')] = {'description' : description,
                                               'clearable': toBool(table.get('clearable'))}


    def getName(self):
        return self._name

    def getAllConfiguration(self):
        return list(self._configuration.keys())

    def getModifiableConfiguration(self):
        return [name for name,value in list(self._configuration.items())
                if value['modifiable']]

    def getAllStatistics(self):
        return list(self._statistics.keys())

    def getClearableStatistics(self):
        return [name for name,value in list(self._statistics.items())
                if value['clearable']]

    def getClearableTables(self):
        return [name for name,value in list(self._tables.items())
                if value['clearable']]

    def getAllTables(self):
        return list(self._tables.keys())

    def getConfigurationInfo(self,name):
        if name in self._configuration:
            return self._configuration[name]
        else:
            raise ManifestException("unknown configuration parameter: %s" % name)

    def getStatisticInfo(self,name):
        if name in self._statistics:
            return self._statistics[name]
        else:
            raise ManifestException("unknown statistic element: %s" % name)

    def getTableInfo(self,name):
        if name in self._tables:
            return self._tables[name]
        else:
            raise ManifestException("unknown table: %s" % name)
