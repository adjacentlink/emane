#!/bin/bash -

if [ $# -ne 1 ]; then
    echo "usage: emanegenmanifests.sh OUTPUT_DIR"
    exit 1
fi

outdir=$1

if [ ! -d $outdir ]; then
    echo "directory does not exist: $outdir"
    exit 1
fi

ld_library_path_orig=$LD_LIBRARY_PATH

for plugin in eelgenerator \
    transraw \
    transvirtual \
    gpsdlocationagent \
    bypassphylayer \
    rfpipemaclayer \
    ieee80211abgmaclayer \
    bypassmaclayer \
    commeffectshim \
    timinganalysisshim \
    phyapitestshim \
    emanephy \
    nemmanager \
    transportmanager \
    eventgeneratormanager \
    eventagentmanager\
    tdmaeventschedulerradiomodel;
do
    LD_LIBRARY_PATH=$ld_library_path_orig
    
    for location in $(find $PWD -name lib$plugin.so -exec dirname {} \;)
    do
	      LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$location
    done

    export LD_LIBRARY_PATH
    
    if $(! ./src/applications/emaneinfo/emaneinfo $plugin -m > $outdir/$plugin.xml); then
        rm -f  $outdir/$plugin.xml
    fi
done
