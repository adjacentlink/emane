#!/bin/bash -
#
# Copyright (c) 2015 - Adjacent Link LLC, Bridgewater, New Jersey
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

do_clean=0

while getopts ':c' OPTION
do
    case $OPTION in
        c)
            do_clean=1
            ;;
        ?)
        printf "usage: %s: [-c]\n" $(basename $0) >&2
        echo
        echo "Bootstrap or clean autotools project"
        echo
        echo "options:"
        echo "  -c     clean autogen files"
        exit 1
        ;;
    esac
done

shift $(($OPTIND - 1))

if [ $do_clean -eq 1 ]
then
    rm -rf aclocal.m4 \
       autom4te.cache \
       compile \
       config.guess \
       config.log \
       config.status \
       config.sub \
       configure \
       depcomp \
       install-sh \
       libtool \
       ltmain.sh \
       Makefile \
       Makefile.in \
       missing 
    find m4 -type f ! -name 'm4_ax_cxx_compile_stdcxx_11.m4' -delete
    find . -name 'Makefile.in' -delete
    
else
    libtoolize --force --copy

    aclocal --force

    automake --add-missing --copy

    autoreconf --force
fi
