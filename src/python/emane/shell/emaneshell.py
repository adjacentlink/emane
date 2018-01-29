#!/us/bin/env python
#
# Copyright (c) 2013-2014,2016-2017 - Adjacent Link LLC, Bridgewater,
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

from __future__ import absolute_import, division, print_function
import cmd
import os
import sys
import re
import glob
import textwrap
from . import ControlPortClient
from . import ControlPortException
from . import Manifest
from . import ManifestException

class EMANEShell(cmd.Cmd):
    def __init__(self,host,port):
        cmd.Cmd.__init__(self)
        self.prompt="[emanesh (%s:%d)] ## " % (host,port)
        self._client = ControlPortClient(host,port)
        self._manifest = {'emulator' : [(0,'all','nemmanager')]}
        self._mapping = {'emulator': {}}
        self._shims = set()
        self._pluginInfo = {}

        for (nem,components) in list(self._client.getManifest().items()):
            self._manifest[nem] = []
            self._mapping[nem] = {}
            shimcount = 0
            for component in components:
                name = component[1].lower()
                if name == 'shim':
                    name += str(shimcount)
                    shimcount+=1
                    self._shims.add(name)

                self._mapping[nem][name] = component[0]
                self._manifest[nem].append((component[0],name,component[2]))

        manifestpath = os.getenv('EMANEMANIFESTPATH','/usr/share/emane/manifest')

        for directory in manifestpath.split(':'):
            for manifestXML in glob.glob("%s/*.xml" % directory):
                try:
                    manifest = Manifest(manifestXML)
                    self._pluginInfo[manifest.getName()] = manifest
                except ManifestException:
                    pass

        if not len(self._pluginInfo):
            print("warning: no plugin manifest XML loaded. Check EMANEMANIFESTPATH.")

    def emptyline(self):
        pass

    def can_exit(self):
        return True

    def default(self,line):
        print("error: unknown command:",line.split()[0])

    def help_help(self):
        print("""
        The help command is used to get usage information for each
        command.

        usage: help <command>
               <command> ::= 'get' | 'clear' | 'show' | 'info' |
                             'exit' | 'help'
        """)

    def do_EOF(self,args):
        """
        The ^D (Ctrl+d) command is used to exit the shell.

        usage: ^D

        example:
          ## ^D
        """
        print()
        return self.do_exit(args)


    def do_exit(self,message):
        """
        The exit command is used to exit the shell.

        usage: exit

        example:
          ## exit
        """
        if message:
            print(message)
        self._client.stop()
        return True

    def do_show(self,args):
        """
        The show command is used to display manifest information provided
        by the connected control port server.

        The displayed information will include the NEM ids, component layer
        types present and the name of the component layer plugin.

        usage: show

        example:
          ## show
        """
        args = args.split()

        if not len(args):
            for (nem,components) in list(self._manifest.items()):
                if nem != 'emulator':
                    print("nem %-3d" % nem, end=' ')
                    for component in components:
                        print("%s(%s)" % (component[1],component[2]), end=' ')
                    print()
        else:
            print('error: too many arguements')

    def do_info(self,args):
        """
        The info command is used to display information loaded from plugin
        manifest files. There are four types of info commands: manifest,
        config, stat and table. The manifest info command is used to display
        the names of discovered plugins. The config, stat and table info
        commands are used to display plugin specific item descriptions.


        The config, stat and table info commands use a single optional
        plugin-specific name parameter:
        * config name specifies a configuration parameter name
        * stat name specifies a statistic element name
        * table name specifies a table name

        When no name is specified all known names are listed.

        usage: info <type> <plugin> [<plugin-specific>]
               <type>            ::=  'config' | 'stat' | 'table'
               <plugin>          ::=  plugin-name
               <plugin-specific> ::= <names>
               <names>           ::= <name> | <name> <names>
               <name>            ::= [A-Za-z0-9]+

               info 'manifest'

        example:
          ## info manifest
          ## info config rfpipemaclayer
          ## info config rfpipemaclayer neighbormetricdeletetime
          ## info stat emanephy processedEvents
        """
        args = args.split()

        if not len(args):
            print('error: missing info type')
            return
        else:
            command = args[0].lower()

            if command != 'config' and \
                    command != 'stat' and \
                    command != 'table' and \
                    command != 'manifest':
                print("error: invalid info command type:",args[0])
                return

            if command == "manifest":
                if len(args) == 1:
                    print()
                    print('   Loaded plugin manifests')
                    print()
                    names = list(self._pluginInfo.keys())
                    names.sort()
                    for name in names:
                        print('    ',name)
                    print()
                    return

                else:
                    print('error: too many arguements')
                    return

            if len(args) >= 2:
                plugin = args[1]

                if plugin not in self._pluginInfo:
                    print("error: invalid plugin name or missing manifest:",plugin)
                    return

                if len(args) == 3:
                    if command == 'config':
                        parameter = args[2]

                        try:
                            info = self._pluginInfo[plugin].getConfigurationInfo(parameter)
                            print()
                            for line in textwrap.wrap("Configuration parameter"
                                                      " information for %s %s" % (plugin,
                                                                                  parameter),
                                                      initial_indent='    ',
                                                      subsequent_indent='    ',
                                                      width=75):
                                print(line)
                            print()
                            for line in textwrap.wrap(info['description'],
                                                      initial_indent='    ',
                                                      subsequent_indent='    ',
                                                      width=75):
                                print(line)
                            print()
                            print('     default   :',info['default'])
                            print('     required  :',info['required'])
                            print('     modifiable:',info['modifiable'])

                            if 'numeric' in info:
                                print("     type      :",info['numeric']['type'])
                                print("     range     : [%s,%s]" % (info['numeric']['minValue'],
                                                                    info['numeric']['maxValue']))
                            else:
                                print("     type      :",info['nonnumeric']['type'])

                            print("     regex     : %s" % info['regex'])
                            print("     occurs    : [%d,%d]" % (info['minOccurs'],info['maxOccurs']))
                            print('     default   :', end=' ')
                            for value in info['values']:
                                print(value,' ', end=' ')
                            print()
                            print()

                        except:
                            print("error: invalid configuration parameter name:",parameter)
                            return

                    elif command == 'stat':
                        element = args[2]
                        try:
                            info = self._pluginInfo[plugin].getStatisticInfo(element)
                            print()
                            for line in textwrap.wrap("Statistic element information for"
                                                      " %s %s" % (plugin,
                                                                  element),
                                                      initial_indent='    ',
                                                      subsequent_indent='    ',
                                                      width=75):
                                print(line)
                            print()
                            for line in textwrap.wrap(info['description'],
                                                      initial_indent='    ',
                                                      subsequent_indent='    ',
                                                      width=75):
                                print(line)
                            print()
                            print('     clearable :',info['clearable'])
                            print('     type      :',info['type'])
                            print()

                        except:
                            print("error: invalid statistic element name:",element)
                            return


                    elif command == 'table':
                        table = args[2]
                        try:
                            info = self._pluginInfo[plugin].getTableInfo(table)
                            print()
                            for line in textwrap.wrap("Table information for %s %s" % (plugin,
                                                                                       table),
                                                      initial_indent='    ',
                                                      subsequent_indent='    ',
                                                      width=75):
                                print(line)
                            print()
                            for line in textwrap.wrap(info['description'],
                                                      initial_indent='    ',
                                                      subsequent_indent='    ',
                                                      width=75):
                                print(line)
                            print()
                            print('     clearable :',info['clearable'])
                            print()

                        except:
                            print("error: invalid statistic table name:",table)
                            return


                elif len(args) > 3:
                    print('error: too many arguements')
                    return

                else:
                    if command == 'config':
                        print()
                        print('   Available configuration parameters for',plugin)
                        print()
                        names = self._pluginInfo[plugin].getAllConfiguration()
                        names.sort()
                        for name in names:
                            print('    ',name)
                        print()

                    elif command == 'stat':
                        print()
                        print('   Available statistic elements for',plugin)
                        print()
                        names = self._pluginInfo[plugin].getAllStatistics()
                        names.sort()
                        for name in names:
                            print('    ',name)
                        print()

                    elif command == 'table':
                        print()
                        print('   Available statistic tables for',plugin)
                        print()
                        names = self._pluginInfo[plugin].getAllTables()
                        names.sort()
                        for name in names:
                            print('    ',name)
                        print()


            else:
                print("error: missing plugin name")
                return


    def complete_info(self, text, line, begidx, endidx):
        args = line.split()
        completions = {'info' : ['config','stat','table','manifest'],
                       'config' : list(self._pluginInfo.keys()),
                       'stat' : list(self._pluginInfo.keys()),
                       'table' : list(self._pluginInfo.keys())}

        if len(args) >= 3:
            if args[2] in self._pluginInfo:
                if args[1] == 'config':
                    completions[args[2]] = self._pluginInfo[args[2]].getAllConfiguration()
                elif args[1] == 'stat':
                    completions[args[2]] = self._pluginInfo[args[2]].getAllStatistics()
                elif args[1] == 'table':
                    completions[args[2]] = self._pluginInfo[args[2]].getAllTables()

        if text:
            return [
                item for item in completions[args[-2]]
                if item.startswith(args[-1])
            ]
        else:
            return completions[args[-1]]

    def do_get(self,args):
        """
        The get command is used to get configuration, statistic and
        statistic table values.

        All get types use optional target-specific name parameters:
        * 'config' names specify configuration parameter names
        * 'stat' names specify statistic element names
        * 'table' names specify table names

        When no names are specified all items are retrieved.

        usage: get <type> <targets> <layer> [<target-specific>]
               <type>            ::=  'config' | 'stat' | 'table'
               <targets>         ::=  <nem> | <nem> <target> | 'emulator' |
                                      'nems'| '*'
               <layer>           ::=  'all' | 'mac' | 'phy' | <shim>
               <nem>             ::=  [1-9] | [1-9][0-9]+
               <shim>            ::=  'shim'[0-9]+
               <target-specific> ::= <names> | ''
               <names>           ::= <name> | <name> <names>
               <name>            ::= [A-Za-z0-9]+

        example:

          Get all configuration info from all NEM layers and the emulator
          ## get config * all

          Get all statistic tables from all NEM mac layers
          ## get table nems mac

          Get two statistic items from the mac layers of NEM 1 and 2
          ## get stat 1 2 mac processedEvents processedDownstreamControl
        """
        return self._process('get',args)

    def complete_get(self, text, line, begidx, endidx):
        return self._completeMany('get',
                                  text, line, begidx, endidx,"",
                                  config='getAllConfiguration',
                                  stat='getAllStatistics',
                                  table='getAllTables')

    def do_clear(self,args):
        """
        The clear command is used to clear statistic elements that have
        been designated as clearable.

        Optional target-specific statistic element names can be specified.
        When no names are specified all clearable statistic elements will
        be cleared.

        usage: clear stat <targets> <layer> [<target-specific>]
               <type>            ::=  'stat' | 'table'
               <targets>         ::=  <nem> | <nem> <target> | 'emulator' |
                                      'nems'| '*'
               <layer>           ::=  'all' | 'mac' | 'phy' | <shim>
               <nem>             ::=  [1-9] | [1-9][0-9]+
               <shim>            ::=  'shim'[0-9]+
               <target-specific> ::= <names> | ''
               <names>           ::= <name> | <name> <names>
               <name>            ::= [A-Za-z0-9]+

        example:
          Clear all statistics from all NEM layers and the emulator
          ## clear stat * all

          Clear all statistics from all NEM mac layers
          ## clear stat nems mac

          Clear two statistic items from the phy layers of NEM 1 and 2
          ## clear stat 1 2 phy processedEvents processedDownstreamControl
        """
        return self._process('clear',args)


    def complete_clear(self, text, line, begidx, endidx):
        return self._completeMany('clear',
                                  text, line, begidx, endidx,"",
                                  stat='getClearableStatistics',
                                  table='getClearableTables')

    def _process(self,action,args):
        args = args.split()

        if(action == 'get'):
            if len(args):
                command = args[0].lower()
                if command == 'config':
                    command = 'configuration'
                elif command == 'stat':
                    command = 'statistic'
                elif command == 'table':
                    command = 'table'
                else:
                    print("error: invalid get command type:",args[0])
                    return
            else:
                print("error: missing get command type")
                return

        elif(action == 'clear'):
            if len(args):
                command = args[0].lower()
                if command == 'stat':
                    command = 'statistic'
                elif command == 'table':
                    command = 'table'
                else:
                    print("error: invalid clear command type:",args[0])
                    return
            else:
                print("error: missing clear command type")
                return

        index = 1
        targets = []

        if len(args) > index:
            for arg in args[index:]:
                try:
                    nem = int(arg)
                    if nem not in self._manifest:
                         print("error: invalid target:",nem)
                         return
                    else:
                        targets.append(nem)
                except:
                    if arg.lower() == 'nems':
                        targets.extend([x for x in list(self._manifest.keys()) if x != 'emulator'])

                    elif arg.lower() == '*':
                        targets.extend(list(self._manifest.keys()))

                    elif arg.lower() == 'emu' or arg.lower() == 'emulator':
                        targets.append('emulator')
                    else:
                        break

                index+=1

        if not len(targets):
            print("error: missing target(s)")
            return

        targets = list(set(targets))

        component = ""

        if len(args) > index:
            component = args[index].lower()
            if component != 'phy' and \
                    component != 'mac' and \
                    component != 'transport' and \
                    component != 'all' and \
                    not (re.match('^shim\d+$', component) and component in self._shims):
                print("error: invalid component layer:",args[index])
                return

            index+=1
        else:
            print("error: missing component layer")
            return


        if component != "all":
            for target in set(targets):
                if component not in self._mapping[target]:
                    if target != 'emulator':
                        print("error: component not present in target %d: %s" % (target,component))
                    else:
                        print("error: component not present in emulator: %s" % (component))
                    return

        names = []

        if len(args) > index:
            names = args[index:]

        for target in targets:
            for componentInfo in self._manifest[target]:
                if component == "all" or component == componentInfo[1]:
                    if action == "get":
                        if command == "configuration":
                            try:
                                entries =  self._client.getConfiguration(componentInfo[0],names)

                                for name in sorted(entries.keys()):
                                    values = entries[name]
                                    if target != 'emulator':
                                        print("nem %-3d %s "%(target,componentInfo[1]),name,"=",",".join([str(x[0]) for x in values]))
                                    else:
                                        print("emulator",name,"=",",".join([str(x[0]) for x in values]))

                            except ControlPortException as exp:
                                if exp.fatal():
                                    return self.do_exit(exp)
                                else:
                                    if target != 'emulator':
                                        print("nem %-3d %s "%(target,componentInfo[1]),exp)
                                    else:
                                        print("emulator",exp)


                        elif command == "statistic":
                            try:
                                statistics = self._client.getStatistic(componentInfo[0],names)

                                for name in sorted(statistics.keys()):
                                    value = statistics[name]
                                    if target != 'emulator':
                                        print("nem %-3d %s "%(target,componentInfo[1]),name,"=",value[0])
                                    else:
                                        print("emulator",name,"=",value[0])

                            except ControlPortException as exp:
                                if exp.fatal():
                                    return self.do_exit(exp)
                                else:
                                    if target != 'emulator':
                                        print("nem %-3d %s "%(target,componentInfo[1]),exp)
                                    else:
                                        print("emulator",exp)


                        elif command == "table":
                            try:
                                statisticTables = self._client.getStatisticTable(componentInfo[0],names)

                                for name in sorted(statisticTables.keys()):
                                    widths = []
                                    (labels,rows) = statisticTables[name]

                                    for label in labels:
                                      widths.append(len(label))

                                    for row in rows:
                                        i = 0
                                        for item in row:
                                            widths[i] = max(len(str(item[0])),widths[i])
                                            i += 1

                                    if target != 'emulator':
                                        print("nem %-3d %s %s"%(target,componentInfo[1],name))
                                    else:
                                        print("emulator", name)

                                    i = 0
                                    for label in labels:
                                        print('|',str(label).ljust(widths[i]), end=' ')
                                        i += 1
                                    print("|")
                                    if not len(rows):
                                        print()
                                    else:
                                        for row in rows:
                                            i = 0
                                            for item in row:
                                                print('|',str(item[0]).ljust(widths[i]), end=' ')
                                                i += 1
                                            print("|")
                                        print()
                            except ControlPortException as exp:
                                if exp.fatal():
                                    return self.do_exit(exp)
                                else:
                                    if target != 'emulator':
                                        print("nem %-3d %s "%(target,componentInfo[1]),exp)
                                    else:
                                        print("emulator",exp)

                    elif action =="clear":
                        if command == "statistic":
                            try:
                                self._client.clearStatistic(componentInfo[0],names)

                                if target != 'emulator':
                                    print("nem %-3d %s "%(target,componentInfo[1]),"statistics cleared")
                                else:
                                    print("emulator statistics cleared")

                            except ControlPortException as exp:
                                if exp.fatal():
                                    return self.do_exit(exp)
                                else:
                                    if target != 'emulator':
                                        print("nem %-3d %s "%(target,componentInfo[1]),exp)
                                    else:
                                        print("emulator",exp)

                        elif command == "table":
                            try:
                                self._client.clearTable(componentInfo[0],names)

                                if target != 'emulator':
                                    print("nem %-3d %s "%(target,componentInfo[1]),"tables cleared")
                                else:
                                    print("emulator tables cleared")

                            except ControlPortException as exp:
                                if exp.fatal():
                                    return self.do_exit(exp)
                                else:
                                    if target != 'emulator':
                                        print("nem %-3d %s "%(target,componentInfo[1]),exp)
                                    else:
                                        print("emulator",exp)


    def complete_set(self, text, line, begidx, endidx):
        return self._completeMany('set',
                                  text, line, begidx, endidx,"=",
                                  config='getModifiableConfiguration')


    def do_set(self,args):
        """
        The set command is used to set configuration elements that have
        been designated as modifiable.

        One or more configuration parameter value expressions can be
        specified.

        usage: set config <targets> <layer> <expressions>
               <targets>     ::=  <nem> | <nem> <target> | 'nems' |
                                  '*'
               <layer>       ::=  'all' | 'mac' | 'phy' | <shim>
               <nem>         ::=  [1-9] | [1-9][0-9]+
               <shim>        ::=  'shim'[0-9]+
               <nem>         ::=  [1-9] | [1-9][0-9]+
               <expressions> ::=  <expression> | <expression> <expressions>
               <expression>  ::= <name>'='<values>
               <name>        ::= [.A-Za-z0-9]+
               <values>      ::= <value> | <value>','<values>
               <value>       ::= value-string

        example:
          Set the txpower parameter for all NEM phy layers
          ## set config nems phy txpower=20

          Set the cwmin0 and cwmax0 parameters for NEM 1, 2 and 3 mac layers
          ## set config 1 2 3 mac cwmin0=100 cwmax0=200
        """
        args = args.split()

        if len(args):
            command = args[0].lower()
            if command == 'config':
                command = 'configuration'
            else:
                print("error: invalid get command type:",args[0])
                return
        else:
            print("error: missing set command type")
            return

        index = 1

        targets = []

        for arg in args[index:]:
            try:
                nem = int(args[index])

                if nem not in self._mapping:
                    print("error: invalid target:",target)
                    return
                else:
                    targets.append(nem)

            except:
                if args[index] == 'nems':
                    targets =  [x for x in list(self._mapping.keys()) if x != 'emulator']
                else:
                    break

            index+=1
        else:
            print("error: missing target")
            return

        component = None

        if len(args) > index:
            component = args[index].lower()
            if component != 'phy' and \
                    component != 'mac' and \
                    component != 'transport' and \
                    component != 'all' and \
                    not (re.match('^shim\d+$', component) and component in self._shims):
                print("error: invalid component layer:",args[index])
                return

            index+=1
        else:
            print("error: missing component layer")
            return

        for target in targets:
            if component != 'all' and  component not in self._mapping[target]:
                print("error: component not present in target %d: %s" % (target,component))
                return

        names = []

        for target in targets:
            updates = []
            if len(args) > index:
                for expression in args[index:]:
                    m = re.match('^([.0-9A-Za-z]+)=(.+)', expression)

                    def toBool(val):
                        val = val.lower()

                        if val in ('yes','on','enable','true','1'):
                            return True
                        elif  val in ('no','off','disable','false','0'):
                            return False
                        else:
                            raise ValueError()

                    convert = {'uint64' : (ControlPortClient.TYPE_UINT64,int),
                               'uint32' : (ControlPortClient.TYPE_UINT32,int),
                               'uint16' : (ControlPortClient.TYPE_UINT16,int),
                               'uint8' : (ControlPortClient.TYPE_UINT8,int),
                               'int64' : (ControlPortClient.TYPE_INT64,int),
                               'int32' : (ControlPortClient.TYPE_INT32,int),
                               'int16' : (ControlPortClient.TYPE_INT16,int),
                               'int8' : (ControlPortClient.TYPE_INT8,int),
                               'bool' : (ControlPortClient.TYPE_BOOLEAN,toBool),
                               'string': (ControlPortClient.TYPE_STRING,str),
                               'inetaddr' : (ControlPortClient.TYPE_INETADDR,str),
                               'float' : (ControlPortClient.TYPE_FLOAT,float),
                               'double' : (ControlPortClient.TYPE_DOUBLE,float)}

                    if m:
                        name = m.group(1)
                        items = m.group(2).split(',')
                        dataType = None
                        for (_,layer,plugin) in self._manifest[target]:
                            if component == 'all' or layer == component:
                                try:
                                    info = self._pluginInfo[plugin].getConfigurationInfo(name)

                                    if 'numeric' in info:
                                        dataType = info['numeric']['type']
                                        break
                                    else:
                                        dataType = info['nonnumeric']['type']
                                        break
                                except:
                                    print("error: nem %hu %s unknown configuration paramater: %s" % (target,
                                                                                                     layer,
                                                                                                     name))
                                    return

                        values = []

                        try:
                            for item in items:
                                values.append(convert[dataType][1](item))

                        except:
                            print("error: invalid conversion %s type %s : %s" % (name,dataType,item))
                            return

                        updates.append((name,convert[dataType][0],tuple(values)))

                    else:
                        print("error: invalid configuration parameter format:", expression)
                        return
            else:
                print("error: missing configration items")
                return



            buildId = self._mapping[target][component]

            try:
                self._client.updateConfiguration(buildId,updates)
                print("nem %-3d %s "%(target,component),"configuration updated")

            except ControlPortException as exp:
                if exp.fatal():
                    return self.do_exit(exp)
                else:
                    print("nem %-3d %s "%(target,component),exp)


    def _completeMany(self,action,text, line, begidx, endidx,trailer,**subactions):
        try:
            layer = None
            args = line.split()

            completions = {}

            if len(args) == 1:
                completions = {action : list(subactions.keys())}

            elif len(args) == 2:
                if text:
                    completions = {action : list(subactions.keys())}
                else:
                    if args[1] in subactions:
                        subaction = args[1]
                        completions[subaction] = [str(x) for x in list(self._mapping.keys())]
                        completions[subaction].extend(['*','nems'])

            elif len(args) > 2 and args[1] in subactions:
                nems = set()
                layers = set()
                skip = False

                for arg in args[2:]:
                    try:
                        nems.add(int(arg))
                    except:
                        if arg == 'nems':
                            nems |= set([x for x in list(self._mapping.keys()) if x != 'emulator'])

                        elif arg == '*':
                            nems |= set(self._mapping.keys())

                        elif arg == 'emulator':
                            nems.add('emulator')

                        elif arg == 'phy' or \
                                arg == 'mac' or \
                                arg == 'transport' or \
                                arg == 'all' or \
                                (re.match('^shim\d+$', arg) and arg in self._shims):
                            layer = arg
                            skip = True
                            break

                for nem in nems:
                    if not len(layers):
                        layers = set(self._mapping[nem].keys())
                    else:
                        layers = layers & set(self._mapping[nem].keys())

                layers.add('all')

                if not layer:
                    completions['nems'] = list(layers)
                    completions['*'] = ['all']

                if not skip:
                    remaining = []

                    possibilities = [x for x in list(self._mapping.keys()) if x != 'emulator']

                    if '*' not in args:
                        if not ('nems' in args and 'emulator' in args):
                            possibilities.extend(['*'])

                    if 'nems' not in args:
                        if '*' not in args:
                            possibilities.extend(['nems'])

                    if 'emulator' not in args:
                        if '*' not in args:
                            possibilities.extend(['emulator'])

                    remaining = list(set(possibilities) - nems)

                    if text:
                        index = -2
                    else:
                        index = -1

                    if args[index] not in layers and  args[index] != '*':
                        completions[args[index]] = [str(x) for x in remaining]
                        completions[args[index]].extend(list(layers))

                params = set()

                for nem in nems:
                    for (_,l,plugin) in self._manifest[nem]:
                        if layer == 'all' or l == layer:
                            method = getattr(self._pluginInfo[plugin],subactions[args[1]])
                            items = ["".join([x,trailer]) for x in method()]
                            if not len(params):
                                params=set(items)
                            else:
                                params = params & set(items)

                if layer in layers:
                    completions[layer] = list(params)
                else:
                    layer = None


            if text or line[-1] != ' ':
                if layer:
                    return [
                        item for item in params
                        if item.startswith(args[-1])
                        ]
                else:
                    return [
                        item for item in completions[args[-2]]
                        if item.startswith(args[-1])
                        ]
            else:
                if layer:
                    return list(params)
                else:
                    return completions[args[-1]]

        except Exception as err:
            pass

    def complete_loglevel(self, text, line, begidx, endidx):
        args = line.split()
        completions = {'loglevel' :  ['0','1','2','3','4']}

        if text:
            return [
                item for item in completions[args[-2]]
                if item.startswith(args[-1])
            ]
        else:
            return completions[args[-1]]

    def do_loglevel(self,args):
        """
        The loglevel command is used to set the emulator loglevel.

          0 - No log messages
          1 - Abort log messages
          2 - Error log messages
          3 - Info log messages
          4 - Debug log messages

        usage: loglevel [0,4]

        example:
          ## loglevel 4
        """
        args = args.split()

        if(len(args) != 1):
            print('error: invalid number of arguments')
            return

        try:
            self._client.setLogLevel(int(args[0]))
            print("log level updated")

        except ControlPortException as exp:
            if exp.fatal():
                return self.do_exit(exp)
            else:
                print("error: ",exp)
        except:
            print("error: invalid log level")
