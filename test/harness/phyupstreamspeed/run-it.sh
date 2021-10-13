#!/bin/bash -
#
# Copyright (c) 2020 - Adjacent Link LLC, Bridgewater, New Jersey
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

iterations=10
pool_size=8
browser=firefox
type_name=omni
fading=0
with_profile=
with_fading=
fading_name=
with_receivepowertable=
receivepowertable_name=

while getopts ':pfr' OPTION
do
    case $OPTION in
        p)
            with_profile=-p
            type_name=profile
            ;;
        f)
            with_fading=-f
            fading_name=fading-
            ;;
        r)
            with_receivepowertable=-r
            receivepowertable_name=rxpowertable-
            ;;
        ?)
        printf "usage: %s: \n" $(basename $0) >&2
        echo
        echo "options:" >&2
        echo "         -f        enable Nakagami fading" >&2
        echo "         -p        use profile define antennas" >&2
        echo "         -r        enable receive power table" >&2
        echo
        exit 1
        ;;
    esac
done

shift $(($OPTIND - 1))

prefix=$type_name-${fading_name}${receivepowertable_name}pool-${pool_size}

out_dir=$prefix-$(date "+%Y%m%d.%H%M%S")

mkdir -p $out_dir

csv_files=""

for i in $(seq 1 $iterations)
do
    sleep 1

    csv_file=$prefix-$(date "+%Y%m%d.%H%M%S").csv

    echo phyupstreamspeed -o $out_dir/$csv_file -n $pool_size $with_fading $with_profile $with_receivepowertable
    ./phyupstreamspeed -o $out_dir/$csv_file -n $pool_size $with_fading $with_profile $with_receivepowertable

    csv_files="$csv_files $out_dir/$csv_file"
done

echo ./generate-graphs --pool $pool_size --out-dir $out_dir $csv_files
./generate-graphs --pool $pool_size --out-dir $out_dir $csv_files

$browser -o $out_dir/*.png
