#!/bin/bash

(
cat << EOF
antennapattern.h
antennaprofileexception.h
antennaprofilemanifest.h
eventtablepublisher.h
fadingalgorithm.h
fadingmanager.h
freespacepropagationmodelalgorithm.h
gainmanager.h
locationinfo.h
locationinfo.inl
locationinfoformatter.h
locationmanager.h
nakagamifadingalgorithm.h
noiserecorder.h
positionecef.h
positionecef.inl
positionneu.h
positionneu.inl
positionneuformatter.h
positionorientationvelocity.h
positionorientationvelocity.inl
positionorientationvelocityformatter.h
positionutils.h
precomputedpropagationmodelalgorithm.h
propagationmodelalgorithm.h
receivepowertablepublisher.h
spectrummonitor.h
tworaypropagationmodelalgorithm.h
wheel.h
wheel.inl
EOF
) | while read f
do
   echo "file = $f"

   git --no-pager grep -l "\"$f" | while read code
   do
      echo "editing file $code"
      sed -i "s%\"$f%\"emane/models/frameworkphy/$f%" $code
   done

done
